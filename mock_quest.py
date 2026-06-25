"""
Mock Quest 스크립트 — Meta Quest APK 없이 ZMQ keypoint 스트리밍을 시뮬레이션.

사용법:
    python mock_quest.py              # 정적 손 자세 (기본)
    python mock_quest.py --move       # 손목 위치를 천천히 이동시켜 팔 이동 테스트
"""

import argparse
import time
import zmq
import numpy as np

# configs/network.yaml 기본값과 맞춰야 함
HOST = "localhost"
KEYPOINT_PORT = 8087       # oculus_reciever_port
BUTTON_PORT = 8095         # resolution_button_port
RESET_PORT = 8100          # teleop_reset_port

VR_FREQ = 60
DT = 1.0 / VR_FREQ

# ── 기본 손 자세 (24 keypoints, 단위: m) ─────────────────────────────────
# 인덱스 0 = 손목, 이후 각 손가락 관절
def make_hand_keypoints(wrist_offset=np.zeros(3)):
    kps = np.zeros((24, 3))

    # 손목
    kps[0] = [0.0, 0.0, 0.0]

    # metacarpals (2,6,9,12,15)
    kps[2]  = [0.02, 0.01, 0.0]   # 엄지
    kps[6]  = [0.01, 0.08, 0.0]   # 검지
    kps[9]  = [0.0,  0.09, 0.0]   # 중지
    kps[12] = [-0.01, 0.08, 0.0]  # 약지
    kps[15] = [-0.02, 0.07, 0.0]  # 새끼

    # knuckles (6,9,12,16)
    kps[16] = [-0.015, 0.075, 0.0]

    # 나머지 관절은 metacarpal 위치에서 약간 오프셋
    for i in range(24):
        if np.all(kps[i] == 0) and i != 0:
            kps[i] = kps[max(0, i-1)] + np.array([0.0, 0.01, 0.0])

    kps += wrist_offset
    return kps


def keypoints_to_token(kps: np.ndarray) -> bytes:
    """numpy (24,3) → Quest APK 포맷 바이트 문자열"""
    parts = "|".join(f"{x:.6f},{y:.6f},{z:.6f}" for x, y, z in kps)
    return f"absolute:{parts}".encode()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--move", action="store_true",
                        help="손목 위치를 서서히 이동시켜 arm retargeting 테스트")
    parser.add_argument("--host", default=HOST)
    args = parser.parse_args()

    ctx = zmq.Context()

    kp_sock = ctx.socket(zmq.PUSH)
    kp_sock.connect(f"tcp://{args.host}:{KEYPOINT_PORT}")

    btn_sock = ctx.socket(zmq.PUSH)
    btn_sock.connect(f"tcp://{args.host}:{BUTTON_PORT}")

    reset_sock = ctx.socket(zmq.PUSH)
    reset_sock.connect(f"tcp://{args.host}:{RESET_PORT}")

    print(f"[mock_quest] 연결: {args.host}")
    print(f"  keypoint → :{KEYPOINT_PORT}")
    print(f"  button   → :{BUTTON_PORT}")
    print(f"  reset    → :{RESET_PORT}")
    print("Ctrl+C로 종료\n")

    t = 0.0
    try:
        while True:
            if args.move:
                # 손목을 앞뒤로 5cm 진동 (arm 이동 테스트용)
                offset = np.array([0.05 * np.sin(2 * np.pi * 0.2 * t), 0.0, 0.0])
            else:
                offset = np.zeros(3)

            kps = make_hand_keypoints(wrist_offset=offset)
            token = keypoints_to_token(kps)

            kp_sock.send(token)
            btn_sock.send(b'High')    # High resolution
            reset_sock.send(b'High')  # CONT (텔레오퍼레이션 활성)

            t += DT
            time.sleep(DT)

    except KeyboardInterrupt:
        print("\n[mock_quest] 종료")
    finally:
        kp_sock.close()
        btn_sock.close()
        reset_sock.close()
        ctx.term()


if __name__ == "__main__":
    main()
