import numpy as np
import zmq
from collections import deque
from copy import deepcopy as copy

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

        self.resolution_scale = 1
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

        self.use_filter = use_filter
        if use_filter:
            robot_init_cart = self._homo2cart(self.robot_init_H)
            self.comp_filter = Filter(robot_init_cart, comp_ratio=0.8)

        self._timer = FrequencyTimer(VR_FREQ)

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
        if self.is_first_frame or (
            self.arm_teleop_state == ARM_TELEOP_STOP
            and new_state == ARM_TELEOP_CONT
        ):
            moving_hand_frame = self._reset_teleop()
        else:
            moving_hand_frame = self._get_hand_frame()
        self.arm_teleop_state = new_state

        scale_mode = self._get_resolution_scale_mode()
        self.resolution_scale = 1 if scale_mode == ARM_HIGH_RESOLUTION else 0.6

        if moving_hand_frame is None:
            return

        self.hand_moving_H = self._turn_frame_to_homo_mat(moving_hand_frame)

        if self.wrist_only:
            # 손목 위치 delta만 추출 — 회전 완전 무시
            wrist_init = self.hand_init_H[:3, 3]        # 초기 손목 위치 (Unity m)
            wrist_now  = self.hand_moving_H[:3, 3]      # 현재 손목 위치 (Unity m)
            delta_hand = wrist_now - wrist_init          # 손목 이동량 (m)

            robot_init_pos = self.robot_init_H[:3, 3]   # 초기 로봇 TCP 위치 (m)
            target_pos = robot_init_pos + delta_hand * self.resolution_scale

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

        # dead zone: 목표와 현재 위치 차이가 임계값 미만이면 무시
        current_cart = self._homo2cart(copy(self.robot.get_pose()['position']))
        delta_m = np.linalg.norm(final_pose[:3] - current_cart[:3])
        if delta_m < self.dead_zone_m:
            return

        # latency buffer: N 프레임 전 명령을 실행
        self._cmd_buffer.append(final_pose)
        if len(self._cmd_buffer) <= self.latency_frames:
            return
        delayed_pose = self._cmd_buffer[0]

        self.robot.arm_control(delayed_pose)

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
        print('텔레오퍼레이터 종료')
