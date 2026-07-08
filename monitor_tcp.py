"""
텔레오퍼레이션 중 로봇 TCP 위치 실시간 모니터링.
teleop.py와 별도 터미널에서 동시에 실행.

사용법:
    python monitor_tcp.py
"""
import sys
import time
import numpy as np
import rbpodo as rb

ROBOT_IP = "10.0.2.7"
FREQ = 10  # 초당 출력 횟수


def main():
    robot = rb.Cobot(ROBOT_IP)
    rc = rb.ResponseCollector()

    print("TCP 실시간 모니터링 시작 (Ctrl+C 종료)\n")
    print(f"{'X(mm)':>10} {'Y(mm)':>10} {'Z(mm)':>10} | "
          f"{'Rx(deg)':>9} {'Ry(deg)':>9} {'Rz(deg)':>9} | "
          f"{'원점거리(mm)':>12}")
    print("-" * 90)

    try:
        while True:
            _, tcp = robot.get_tcp_info(rc)
            dist = np.linalg.norm(tcp[:3])  # 로봇 베이스 원점으로부터 거리
            print(
                f"\r{tcp[0]:>10.1f} {tcp[1]:>10.1f} {tcp[2]:>10.1f} | "
                f"{tcp[3]:>9.2f} {tcp[4]:>9.2f} {tcp[5]:>9.2f} | "
                f"{dist:>12.1f}",
                end="", flush=True
            )
            time.sleep(1 / FREQ)
    except KeyboardInterrupt:
        print("\n\n종료")


if __name__ == "__main__":
    main()
