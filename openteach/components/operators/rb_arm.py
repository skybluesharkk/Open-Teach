import csv
import time
import numpy as np
import zmq
from collections import deque
from copy import deepcopy as copy
from pathlib import Path

from scipy.spatial.transform import Rotation, Slerp

from openteach.constants import (
    VR_FREQ, ARM_TELEOP_STOP, ARM_TELEOP_CONT,
    ARM_HIGH_RESOLUTION, ARM_LOW_RESOLUTION,
)
from openteach.utils.timer import FrequencyTimer
from openteach.utils.network import ZMQKeypointSubscriber
from openteach.robot.rb_arm import RBArm
from .operator import Operator

np.set_printoptions(precision=2, suppress=True)

# Quest(Unity 왼손계) → Robot base(오른손계) 좌표 변환
# Robot: X+전방  Y+좌  Z+상
# Quest: Z+전방  X-좌  Y+상
R_QUEST_TO_ROBOT = np.array([
    [ 0,  0,  1],
    [-1,  0,  0],
    [ 0,  1,  0],
], dtype=float)


class Filter:
    """위치/자세에 대한 지수 이동평균 필터"""
    def __init__(self, state, comp_ratio=0.6):
        self.pos_state = state[:3]
        self.ori_state = state[3:7]
        self.comp_ratio = comp_ratio

    def __call__(self, next_state):
        self.pos_state = (self.pos_state * self.comp_ratio
                          + next_state[:3] * (1 - self.comp_ratio))
        slerp = Slerp([0, 1], Rotation.from_quat(
            np.stack([self.ori_state, next_state[3:7]])))
        self.ori_state = slerp([1 - self.comp_ratio])[0].as_quat()
        return np.concatenate([self.pos_state, self.ori_state])


class RBArmOperator(Operator):
    """
    FrankaArmOperator와 동일한 retargeting 로직,
    로봇 컨트롤러만 RBArm(rbpodo)으로 교체.
    """

    def __init__(
        self,
        host: str,
        transformed_keypoints_port: int,
        robot_ip: str,
        use_filter: bool = True,
        arm_resolution_port: int = None,
        teleoperation_reset_port: int = None,
        dead_zone_mm: float = 5.0,
        latency_frames: int = 0,
        wrist_only: bool = True,
        control_hz: int = 20,
        move_scale: float = 0.3,
        tap_to_move: bool = False,
    ):
        self.notify_component_start('rb arm operator')

        self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=transformed_keypoints_port,
            topic='transformed_hand_coords',
        )
        self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=transformed_keypoints_port,
            topic='transformed_hand_frame',
        )

        self._robot = RBArm(robot_ip=robot_ip)

        self.resolution_scale = move_scale  # 손 이동량 → 로봇 이동량 비율
        self.arm_teleop_state = ARM_TELEOP_STOP
        self.is_first_frame = True

        self._arm_resolution_subscriber = ZMQKeypointSubscriber(
            host=host, port=arm_resolution_port, topic='button',
        )
        self._arm_teleop_state_subscriber = ZMQKeypointSubscriber(
            host=host, port=teleoperation_reset_port, topic='pause',
        )

        self.robot_init_H = self.robot.get_pose()['position']

        # dead zone: 이 거리(m) 이상 움직일 때만 로봇 이동
        self.dead_zone_m = dead_zone_mm / 1000.0

        # latency buffer: latency_frames 만큼 이전 명령을 실행
        self.latency_frames = latency_frames
        self._cmd_buffer: deque = deque(maxlen=max(latency_frames + 1, 1))

        # wrist_only: 손목 위치만 추적, 회전 무시
        self.wrist_only = wrist_only
        self._robot_init_orientation: np.ndarray = None  # 초기 로봇 방향(쿼터니언) 고정용

        # dead_zone 비교용: get_pose() 대신 마지막 송신 위치 캐싱
        self._last_sent_pos: np.ndarray = None

        # tap_to_move: Middle Pinch 한 번에 한 번만 move_l 실행
        self.tap_to_move = tap_to_move
        self._tap_executed = False  # 현재 CONT 세션에서 이미 이동했는지
        self._write_tap_status('WAITING', np.zeros(3))

        self.use_filter = use_filter
        if use_filter:
            robot_init_cart = self._homo2cart(self.robot_init_H)
            self.comp_filter = Filter(robot_init_cart, comp_ratio=0.8)

        self._timer = FrequencyTimer(control_hz)  # 오퍼레이터 루프 Hz

        # CSV 로그 (손 delta + 로봇 target 기록)
        log_path = Path('/tmp/teleop_axis_log.csv')
        self._log_file = open(log_path, 'w', newline='', buffering=1)
        self._log_writer = csv.writer(self._log_file)
        self._log_writer.writerow([
            'time', 'label',
            'hand_dX_cm', 'hand_dY_cm', 'hand_dZ_cm',
            'robot_tX_mm', 'robot_tY_mm', 'robot_tZ_mm',
        ])
        self._log_start = time.time()
        print(f'[RBArmOperator] 로그 파일: {log_path}')
        print('[RBArmOperator] 세션 중 손을 각 방향으로 천천히 움직여주세요.')

    # ── 프로퍼티 ─────────────────────────────────────────────────────────

    @property
    def timer(self):
        return self._timer

    @property
    def robot(self):
        return self._robot

    @property
    def transformed_hand_keypoint_subscriber(self):
        return self._transformed_hand_keypoint_subscriber

    @property
    def transformed_arm_keypoint_subscriber(self):
        return self._transformed_arm_keypoint_subscriber

    # ── 내부 헬퍼 ────────────────────────────────────────────────────────

    def _get_hand_frame(self):
        for _ in range(10):
            data = self.transformed_arm_keypoint_subscriber.recv_keypoints(
                flags=zmq.NOBLOCK)
            if data is not None:
                break
        if data is None:
            return None
        return np.asanyarray(data).reshape(4, 3)

    def _get_resolution_scale_mode(self):
        data = self._arm_resolution_subscriber.recv_keypoints()
        return np.asanyarray(data).reshape(1)[0]

    def _get_arm_teleop_state(self):
        data = self._arm_teleop_state_subscriber.recv_keypoints()
        return np.asanyarray(data).reshape(1)[0]

    def _turn_frame_to_homo_mat(self, frame):
        t = frame[0]
        R = frame[1:]
        H = np.zeros((4, 4))
        H[:3, :3] = np.transpose(R)
        H[:3, 3] = t
        H[3, 3] = 1
        return H

    def _homo2cart(self, homo_mat):
        t = homo_mat[:3, 3]
        R = Rotation.from_matrix(homo_mat[:3, :3]).as_quat()
        return np.concatenate([t, R])

    def _get_scaled_cart_pose(self, moving_robot_homo_mat):
        unscaled_cart = self._homo2cart(moving_robot_homo_mat)
        current_cart = self._homo2cart(copy(self.robot.get_pose()['position']))
        diff = unscaled_cart[:3] - current_cart[:3]
        scaled = np.zeros(7)
        scaled[3:] = unscaled_cart[3:]
        scaled[:3] = current_cart[:3] + diff * self.resolution_scale
        return scaled

    def _reset_teleop(self):
        print('****** RESETTING TELEOP ******')
        self.robot_init_H = self.robot.get_pose()['position']
        self._wrist_init_for_log = True  # 로그 활성화 플래그
        # wrist_only 모드: 초기 로봇 방향을 고정값으로 저장
        if self.wrist_only:
            self._robot_init_orientation = Rotation.from_matrix(
                self.robot_init_H[:3, :3]
            ).as_quat()

        first_hand_frame = self._get_hand_frame()
        while first_hand_frame is None:
            first_hand_frame = self._get_hand_frame()
        self.hand_init_H = self._turn_frame_to_homo_mat(first_hand_frame)
        self.hand_init_t = copy(self.hand_init_H[:3, 3])
        self.is_first_frame = False

        # 초기 상태 출력
        robot_pos_m = self.robot_init_H[:3, 3]
        hand_pos_m  = self.hand_init_H[:3, 3]
        np.set_printoptions(precision=3, suppress=True)
        print(f'[기준점] 로봇 TCP (m):  X={robot_pos_m[0]:.3f}  Y={robot_pos_m[1]:.3f}  Z={robot_pos_m[2]:.3f}')
        print(f'[기준점] 손목 위치 (m): X={hand_pos_m[0]:.3f}  Y={hand_pos_m[1]:.3f}  Z={hand_pos_m[2]:.3f}')
        print(f'→ 지금 손목 위치가 로봇 TCP 기준점입니다. 손이 움직인 만큼 로봇이 이동합니다.')

        return first_hand_frame

    # ── 핵심 retargeting ─────────────────────────────────────────────────

    def _apply_retargeted_angles(self, log=False):
        new_state = self._get_arm_teleop_state()

        if self.is_first_frame:
            # 최초 1회만 기준점 설정
            moving_hand_frame = self._reset_teleop()
        elif self.tap_to_move and self.arm_teleop_state == ARM_TELEOP_STOP and new_state == ARM_TELEOP_CONT:
            # tap 모드: STOP→CONT 전환 시 hand_init은 유지, 현재 프레임만 가져옴
            moving_hand_frame = self._get_hand_frame()
        elif not self.tap_to_move and self.arm_teleop_state == ARM_TELEOP_STOP and new_state == ARM_TELEOP_CONT:
            # servo 모드: 기준점 리셋
            moving_hand_frame = self._reset_teleop()
        else:
            moving_hand_frame = self._get_hand_frame()
        # STOP 상태로 돌아오면 다음 tap 허용
        if new_state == ARM_TELEOP_STOP:
            self._tap_executed = False
        self.arm_teleop_state = new_state

        # Quest 버튼으로 추가 스케일 조절 (move_scale 위에 곱해짐)
        scale_mode = self._get_resolution_scale_mode()
        btn_scale = 1.0 if scale_mode == ARM_HIGH_RESOLUTION else 0.5
        effective_scale = self.resolution_scale * btn_scale

        if moving_hand_frame is None:
            return

        self.hand_moving_H = self._turn_frame_to_homo_mat(moving_hand_frame)

        if self.wrist_only:
            # 손목 위치 delta만 추출 — 회전 완전 무시
            wrist_init = self.hand_init_H[:3, 3]        # 초기 손목 위치 (Unity m)
            wrist_now  = self.hand_moving_H[:3, 3]      # 현재 손목 위치 (Unity m)
            delta_hand = wrist_now - wrist_init          # 손목 이동량 Quest 좌표계 (m)
            delta_robot = R_QUEST_TO_ROBOT @ delta_hand  # Robot base 좌표계로 변환 (m)

            robot_init_pos = self.robot_init_H[:3, 3]   # 초기 로봇 TCP 위치 (m)
            # 한 번의 tap에서 최대 100mm 이동으로 클램프 (안전)
            delta_clamped = np.clip(delta_robot, -0.1, 0.1)
            target_pos = robot_init_pos + delta_clamped * effective_scale

            final_pose = np.concatenate([target_pos, self._robot_init_orientation])
        else:
            H_HI_HH = copy(self.hand_init_H)
            H_HT_HH = copy(self.hand_moving_H)
            H_RI_RH = copy(self.robot_init_H)

            H_A_R = np.array([
                [1/np.sqrt(2),  1/np.sqrt(2), 0, 0],
                [-1/np.sqrt(2), 1/np.sqrt(2), 0, 0],
                [0,             0,            1, -0.06],
                [0,             0,            0,  1],
            ])

            H_HT_HI = np.linalg.pinv(H_HI_HH) @ H_HT_HH
            H_RT_RH = H_RI_RH @ H_A_R @ H_HT_HI @ np.linalg.pinv(H_A_R)
            self.robot_moving_H = copy(H_RT_RH)
            final_pose = self._get_scaled_cart_pose(self.robot_moving_H)

        if self.use_filter:
            final_pose = self.comp_filter(final_pose)

        # dead zone: 마지막 송신 위치와 비교 (get_pose() 네트워크 호출 제거)
        if self._last_sent_pos is not None:
            delta_m = np.linalg.norm(final_pose[:3] - self._last_sent_pos)
            if delta_m < self.dead_zone_m:
                return

        # latency buffer: N 프레임 전 명령을 실행
        self._cmd_buffer.append(final_pose)
        if len(self._cmd_buffer) <= self.latency_frames:
            return
        delayed_pose = self._cmd_buffer[0]

        if self.tap_to_move:
            # Middle Pinch → 한 번만 move_l 실행 후 대기
            if not self._tap_executed:
                target_mm = delayed_pose[:3] * 1000  # m → mm
                euler = Rotation.from_quat(delayed_pose[3:]).as_euler('xyz', degrees=True)
                tcp_target = np.concatenate([target_mm, euler])
                _, tcp_before = self.robot._robot.get_tcp_info(self.robot._rc)
                delta_mm = tcp_target[:3] - tcp_before[:3]
                print(f'[tap] 현재 TCP: {np.round(tcp_before[:3], 1)}')
                print(f'[tap] 목표 TCP: {np.round(tcp_target[:3], 1)}  (Δ={np.round(delta_mm, 1)} mm)')
                self._write_tap_status('MOVING', tcp_target[:3])
                self.robot._robot.enable_waiting_ack(self.robot._rc)
                self.robot._robot.move_l(
                    self.robot._rc, tcp_target, 150, 300  # 속도 대폭 증가
                )
                started = self.robot._robot.wait_for_move_started(self.robot._rc, 3.0)
                if started.is_success():
                    self.robot._robot.wait_for_move_finished(self.robot._rc)
                    _, tcp_after = self.robot._robot.get_tcp_info(self.robot._rc)
                    print(f'[tap] 이동 완료. 실제 TCP: {np.round(tcp_after[:3], 1)}')
                    self._write_tap_status('DONE', tcp_after[:3])
                else:
                    print('[tap] 이동 실패 (도달불가 또는 타임아웃)')
                    self._write_tap_status('FAIL', tcp_target[:3])
                self.robot._robot.disable_waiting_ack(self.robot._rc)
                self.robot._servo_mode = True
                # 기준점 갱신 — 다음 tap은 현재 위치에서 출발
                self.robot_init_H = self.robot.get_pose()['position']
                if self.wrist_only:
                    self._robot_init_orientation = Rotation.from_matrix(
                        self.robot_init_H[:3, :3]
                    ).as_quat()
                self.hand_init_H = copy(self.hand_moving_H)
                self._tap_executed = True
        else:
            self._last_sent_pos = delayed_pose[:3].copy()
            self.robot.arm_control(delayed_pose)

        # 로그 기록
        if self.wrist_only and hasattr(self, '_wrist_init_for_log'):
            wrist_init = self.hand_init_H[:3, 3]
            wrist_now  = self.hand_moving_H[:3, 3]
            delta_cm   = (R_QUEST_TO_ROBOT @ (wrist_now - wrist_init)) * 100  # Robot frame
            target_mm  = delayed_pose[:3] * 1000
            elapsed    = time.time() - self._log_start
            self._log_writer.writerow([
                f'{elapsed:.3f}', 'teleop',
                f'{delta_cm[0]:.2f}', f'{delta_cm[1]:.2f}', f'{delta_cm[2]:.2f}',
                f'{target_mm[0]:.1f}', f'{target_mm[1]:.1f}', f'{target_mm[2]:.1f}',
            ])

    def _write_tap_status(self, status: str, tcp_mm: np.ndarray):
        """Quest 비주얼라이저가 읽을 수 있도록 상태 파일에 기록"""
        try:
            with open('/tmp/tap_status.txt', 'w') as f:
                f.write(f"{status}|{tcp_mm[0]:.1f},{tcp_mm[1]:.1f},{tcp_mm[2]:.1f}")
        except Exception:
            pass

    def return_real(self):
        return True

    def stream(self):
        self.notify_component_start('rb arm control')
        print("RB 로봇 텔레오퍼레이션 시작 (Oculus)\n")
        while True:
            try:
                if self.robot.get_joint_position() is not None:
                    self.timer.start_loop()
                    self._apply_retargeted_angles()
                    self.timer.end_loop()
            except KeyboardInterrupt:
                break

        self.transformed_arm_keypoint_subscriber.stop()
        self.transformed_hand_keypoint_subscriber.stop()
        self._log_file.close()
        print('텔레오퍼레이터 종료')
        print('[RBArmOperator] 로그 저장 완료: /tmp/teleop_axis_log.csv')
