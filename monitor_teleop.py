"""
텔레오퍼레이션 실시간 모니터링
teleop.py와 별도 터미널에서 동시에 실행.

표시 내용:
  손목 delta (Unity m)  →  로봇 TCP 수신값 (mm)
  손의 이동량이 어느 방향인지, 로봇이 실제로 어디 있는지 한눈에 확인

사용법:
    python monitor_teleop.py
"""
import sys, time
import numpy as np
import zmq
import rbpodo as rb

sys.path.insert(0, '/home/shimyoungchan/Open-Teach')
from openteach.utils.network import ZMQKeypointSubscriber

HOST    = '192.168.50.49'
ROBOT_IP = '10.0.2.7'
FREQ    = 10  # 초당 출력 횟수

def main():
    # ZMQ: 변환된 손 프레임 구독
    sub = ZMQKeypointSubscriber(HOST, 8089, 'transformed_hand_frame')

    # 로봇 TCP 조회
    robot = rb.Cobot(ROBOT_IP)
    rc    = rb.ResponseCollector()

    print("텔레오퍼레이션 모니터링 시작 (Ctrl+C 종료)")
    print("teleop.py + Quest Stream 활성화 후 Middle Pinch 해주세요\n")

    header = (f"{'손목 dX':>8} {'손목 dY':>8} {'손목 dZ':>8}  |  "
              f"{'TCP X':>8} {'TCP Y':>8} {'TCP Z':>8}  |  "
              f"{'원점거리':>8}")
    print(header)
    print("-" * len(header))

    wrist_init = None
    last_print = 0

    try:
        while True:
            # 손목 위치 수신 (non-blocking)
            data = sub.recv_keypoints(flags=zmq.NOBLOCK)
            if data is not None:
                frame = np.array(data).reshape(4, 3)
                wrist = frame[0]  # 손목 절대 위치 (Unity m)

                if wrist_init is None:
                    wrist_init = wrist.copy()
                    print(f"\n[기준점 설정] 손목 초기 위치: "
                          f"X={wrist[0]:.3f} Y={wrist[1]:.3f} Z={wrist[2]:.3f} (m)\n")

                delta = wrist - wrist_init  # 이동량 (m)
            else:
                delta = None

            # 로봇 TCP 수신
            _, tcp = robot.get_tcp_info(rc)
            dist = np.linalg.norm(tcp[:3])

            now = time.time()
            if now - last_print >= 1 / FREQ:
                if delta is not None:
                    # 가장 많이 움직인 축 표시
                    axes = ['X', 'Y', 'Z']
                    dom_idx = int(np.argmax(np.abs(delta)))
                    dom = f"손목 {axes[dom_idx]}{'+'if delta[dom_idx]>0 else '-'}"

                    print(
                        f"\r{delta[0]*100:>+7.1f}cm {delta[1]*100:>+7.1f}cm {delta[2]*100:>+7.1f}cm"
                        f"  [{dom:^8s}]  "
                        f"{tcp[0]:>8.1f} {tcp[1]:>8.1f} {tcp[2]:>8.1f}mm  "
                        f"{dist:>8.1f}mm",
                        end="", flush=True
                    )
                else:
                    print(
                        f"\r{'Quest 미연결':^32s}  "
                        f"{tcp[0]:>8.1f} {tcp[1]:>8.1f} {tcp[2]:>8.1f}mm  "
                        f"{dist:>8.1f}mm",
                        end="", flush=True
                    )
                last_print = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\n종료")

if __name__ == "__main__":
    main()
