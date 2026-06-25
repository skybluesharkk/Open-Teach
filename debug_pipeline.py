"""
파이프라인 디버그 스크립트 - teleop.py 없이 각 단계를 순서대로 검증.
"""
import sys
import time
import pickle
import threading
import numpy as np
import zmq

sys.path.insert(0, '/home/shimyoungchan/Open-Teach')

HOST = '192.168.50.178'

# ── 1단계: OculusVRHandDetector 흉내 ─────────────────────────────────────
def run_detector(ctx, kp_token: bytes, stop_event: threading.Event):
    """mock_quest 데이터를 받아서 파싱 후 재발행"""
    from openteach.components.detector.oculus import OculusVRHandDetector
    from openteach.utils.network import create_pull_socket, ZMQKeypointPublisher

    pull_kp  = create_pull_socket(HOST, 8087)
    pull_btn = create_pull_socket(HOST, 8095)
    pull_rst = create_pull_socket(HOST, 8100)
    pub_kp   = ZMQKeypointPublisher(HOST, 8088)
    pub_btn  = ZMQKeypointPublisher(HOST, 8093)
    pub_rst  = ZMQKeypointPublisher(HOST, 8102)

    pull_kp.setsockopt(zmq.RCVTIMEO, 2000)
    pull_btn.setsockopt(zmq.RCVTIMEO, 2000)
    pull_rst.setsockopt(zmq.RCVTIMEO, 2000)

    print('[detector] 시작 - mock_quest 대기 중...')
    count = 0
    while not stop_event.is_set():
        try:
            raw_kp  = pull_kp.recv()
            raw_btn = pull_btn.recv()
            raw_rst = pull_rst.recv()
        except zmq.Again:
            print('[detector] 타임아웃 - 데이터 없음')
            continue

        # 파싱
        data = raw_kp.decode().strip()
        kp_vals = [0]  # absolute
        for vec_str in data.split(':')[1].strip().split('|'):
            for v in vec_str.split(',')[:3]:
                kp_vals.append(float(v))

        pub_kp.pub_keypoints(kp_vals, 'right')
        pub_btn.pub_keypoints(1, 'button')   # HIGH_RESOLUTION
        pub_rst.pub_keypoints(1, 'pause')    # ARM_TELEOP_CONT

        count += 1
        if count % 60 == 1:
            print(f'[detector] {count}번째 keypoint 발행')

    print('[detector] 종료')


# ── 2단계: TransformHandPositionCoords 흉내 ──────────────────────────────
def run_transform(ctx, stop_event: threading.Event):
    from openteach.components.detector.keypoint_transform import TransformHandPositionCoords
    from openteach.utils.network import ZMQKeypointSubscriber, ZMQKeypointPublisher
    from openteach.constants import OCULUS_NUM_KEYPOINTS, OCULUS_JOINTS, VR_FREQ
    from openteach.utils.vectorops import normalize_vector

    sub = ZMQKeypointSubscriber(HOST, 8088, 'right')
    pub = ZMQKeypointPublisher(HOST, 8089)

    print('[transform] 시작')
    count = 0
    while not stop_event.is_set():
        data = sub.recv_keypoints()
        if data is None:
            continue
        data_arr = np.asanyarray(data[1:]).reshape(OCULUS_NUM_KEYPOINTS, 3)

        # 손목 기준 정규화
        translated = data_arr - data_arr[0]
        knuckle_pts = (OCULUS_JOINTS['knuckles'][0], OCULUS_JOINTS['knuckles'][-1])
        idx_k = translated[knuckle_pts[0]]
        pnk_k = translated[knuckle_pts[1]]

        palm_normal    = normalize_vector(np.cross(idx_k, pnk_k))
        palm_direction = normalize_vector(idx_k + pnk_k)
        cross_product  = normalize_vector(np.cross(palm_direction, palm_normal))

        frame = [cross_product, palm_direction, palm_normal]
        R = np.linalg.solve(frame, np.eye(3)).T
        transformed = (R @ translated.T).T

        # 팔 방향 프레임
        hand_dir = [
            data_arr[0],
            normalize_vector(idx_k - pnk_k),
            palm_normal,
            palm_direction,
        ]

        pub.pub_keypoints(transformed, 'transformed_hand_coords')
        pub.pub_keypoints(hand_dir,    'transformed_hand_frame')

        count += 1
        if count % 60 == 1:
            print(f'[transform] {count}번째 변환 발행. 손목 위치: {np.round(data_arr[0], 3)}')

    print('[transform] 종료')


# ── 3단계: RBArmOperator 핵심 로직만 ────────────────────────────────────
def run_operator(ctx, stop_event: threading.Event):
    from openteach.utils.network import ZMQKeypointSubscriber
    from openteach.robot.rb_arm import RBArm
    from scipy.spatial.transform import Rotation
    from copy import deepcopy as copy

    sub_frame = ZMQKeypointSubscriber(HOST, 8089, 'transformed_hand_frame')
    sub_rst   = ZMQKeypointSubscriber(HOST, 8102, 'pause')

    print('[operator] RBArm 연결 중...')
    arm = RBArm(robot_ip='10.0.2.7')
    print('[operator] 연결 OK')
    robot_init_H = arm.get_pose()['position']
    print(f'[operator] 초기 pose:\n{np.round(robot_init_H, 3)}')

    is_first = True
    hand_init_H = None

    def get_frame():
        for _ in range(10):
            data = sub_frame.recv_keypoints(flags=zmq.NOBLOCK)
            if data is not None:
                return np.asanyarray(data).reshape(4, 3)
        return None

    def frame_to_homo(frame):
        H = np.zeros((4, 4))
        H[:3, :3] = np.transpose(frame[1:])
        H[:3, 3]  = frame[0]
        H[3, 3]   = 1
        return H

    def homo2cart(H):
        t = H[:3, 3]
        q = Rotation.from_matrix(H[:3, :3]).as_quat()
        return np.concatenate([t, q])

    H_A_R = np.array([
        [1/np.sqrt(2),  1/np.sqrt(2), 0,  0],
        [-1/np.sqrt(2), 1/np.sqrt(2), 0,  0],
        [0,             0,            1, -0.06],
        [0,             0,            0,  1],
    ])

    count = 0
    while not stop_event.is_set():
        # pause 상태 수신 (non-blocking으로 변경)
        try:
            rst_data = sub_rst.recv_keypoints(flags=zmq.NOBLOCK)
            teleop_state = int(np.asanyarray(rst_data).reshape(1)[0]) if rst_data is not None else 1
        except Exception:
            teleop_state = 1  # 데이터 없으면 CONT로 간주

        moving_frame = get_frame()
        if moving_frame is None:
            time.sleep(1/60)
            continue

        if is_first:
            robot_init_H = arm.get_pose()['position']
            hand_init_H  = frame_to_homo(moving_frame)
            is_first = False
            print('[operator] 기준점 설정 완료')
            continue

        H_HT_HH = frame_to_homo(moving_frame)
        H_HT_HI = np.linalg.pinv(hand_init_H) @ H_HT_HH
        H_RT_RH = robot_init_H @ H_A_R @ H_HT_HI @ np.linalg.pinv(H_A_R)

        cart = homo2cart(H_RT_RH)
        arm.arm_control(cart)

        count += 1
        if count % 60 == 1:
            _, tcp = arm._robot.get_tcp_info(arm._rc)
            print(f'[operator] {count}번째 servo 명령. TCP: {np.round(tcp[:3], 1)}')

    print('[operator] 종료')


if __name__ == '__main__':
    stop = threading.Event()
    ctx  = zmq.Context()

    threads = [
        threading.Thread(target=run_detector,  args=(ctx, None, stop), daemon=True),
        threading.Thread(target=run_transform, args=(ctx, stop),        daemon=True),
        threading.Thread(target=run_operator,  args=(ctx, stop),        daemon=True),
    ]

    for t in threads:
        t.start()

    print('\nmock_quest를 다른 터미널에서 실행하거나 자동 시작...')
    time.sleep(1)

    # mock_quest 내장 실행
    import subprocess
    mq = subprocess.Popen(
        ['conda', 'run', '-n', 'rbpodo', 'python', 'mock_quest.py',
         '--move', '--host', HOST],
        cwd='/home/shimyoungchan/Open-Teach'
    )

    try:
        time.sleep(15)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        mq.terminate()
        print('\n=== 종료 ===')
