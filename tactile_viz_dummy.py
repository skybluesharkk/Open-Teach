#!/usr/bin/env python3
"""
XHand 택타일 센서 시각화 — 더미 데이터 퍼블리셔

로봇 미연결 상태에서 Quest 손 위 택타일 시각화(TactileOverlay.cs)를
테스트하기 위한 더미 데이터 생성기.

세 가지 모드 (실행 중 키보드 1/2/3 으로 전환):
    F1  : 힘 크기 스칼라 → 손끝 색상 히트맵 (없음→파랑→빨강)
    F2A : 손끝당 합산 벡터 [Fx,Fy,Fz] → 화살표 5개
    F2B : 손끝당 NxM 그리드 벡터장 → TactAR식 deformation field

패킷 형식 (UTF-8 텍스트, ZMQ PUB, 프레임 1개):
    F1:rows,cols;v,v,...|v,v,...          — 손끝 5개 × (rows*cols) 택셀 스칼라(0~1)
    F2A:fx,fy,fz|fx,fy,fz|...             — 손끝 5개 벡터 (N, 손끝 로컬 좌표)
    F2B:rows,cols;v,v,v|v,v,v|...         — 손끝 5개 × (rows*cols) 벡터
                                            (손끝 순서: 엄지,검지,중지,약지,소지 연속)

사용법:
    conda activate rbpodo
    python tactile_viz_dummy.py --host 192.168.50.49 --port 15002 --mode f1
"""

import argparse
import math
import select
import sys
import time

import numpy as np
import zmq

DEFAULT_HOST = "192.168.50.49"
DEFAULT_PORT = 15002
SEND_HZ      = 30

# F2B 그리드 (XHand 실센서는 손끝당 120점 — 렌더 부하 고려해 더미는 4x6=24점)
GRID_ROWS = 4
GRID_COLS = 6

FINGERS = ["thumb", "index", "middle", "ring", "pinky"]

GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
RESET  = "\033[0m"


def make_pub_socket(ctx, host, port):
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 5)
    sock.bind(f"tcp://{host}:{port}")
    return sock


# ── 더미 데이터 생성 ────────────────────────────────────────────────────────
# 시나리오: 손가락이 차례로 눌리는 패턴 + 전체 파도. 시간에 따라 부드럽게 변화.

def finger_intensity(t, idx):
    """손끝 idx의 힘 크기 (0~1). 손가락별 위상차를 둔 파도 패턴."""
    phase  = idx * 1.1
    wave   = 0.5 * (1 + math.sin(t * 1.2 - phase))          # 0~1 큰 파도
    pulse  = max(0.0, math.sin(t * 0.35 - idx * 1.3)) ** 3  # 손가락 순차 강조
    v = 0.55 * wave * pulse + 0.25 * wave
    return float(np.clip(v, 0.0, 1.0))


def finger_vector(t, idx):
    """손끝 idx의 합산 힘 벡터 (N). 손끝 로컬 기준: +Z 누르는 방향 위주."""
    mag = finger_intensity(t, idx) * 5.0          # 최대 ~5N
    ang = t * 0.8 + idx * 0.7
    fx  = 0.35 * mag * math.cos(ang)              # 접선 방향 성분
    fy  = 0.35 * mag * math.sin(ang * 0.9)
    fz  = mag                                     # 법선(누름) 성분
    return fx, fy, fz


def _contact_center(t, idx, rows, cols):
    """시간에 따라 그리드 위를 이동하는 접촉 중심 (r, c)."""
    cr = (rows - 1) / 2 + (rows / 3) * math.sin(t * 0.9 + idx)
    cc = (cols - 1) / 2 + (cols / 3) * math.cos(t * 0.7 + idx * 0.5)
    return cr, cc


def finger_grid_scalar(t, idx, rows, cols):
    """손끝 idx의 택셀별 힘 크기 (0~1). 접촉 중심 가우시안 분포."""
    peak = finger_intensity(t, idx)          # 0~1
    cr, cc = _contact_center(t, idx, rows, cols)
    out = []
    for r in range(rows):
        for c in range(cols):
            d2 = (r - cr) ** 2 + (c - cc) ** 2
            out.append(min(1.0, peak * math.exp(-d2 / 3.0)))
    return out


def finger_grid(t, idx, rows, cols):
    """손끝 idx의 그리드 벡터장. 접촉 중심이 그리드 위를 이동하는 가우시안."""
    mag = finger_intensity(t, idx) * 5.0
    cr, cc = _contact_center(t, idx, rows, cols)
    ang = t * 0.8 + idx * 0.7
    out = []
    for r in range(rows):
        for c in range(cols):
            d2 = (r - cr) ** 2 + (c - cc) ** 2
            w  = math.exp(-d2 / 3.0)              # 가우시안 접촉 분포
            fz = mag * w
            fx = 0.4 * fz * math.cos(ang)
            fy = 0.4 * fz * math.sin(ang)
            out.extend((fx, fy, fz))
    return out


# ── 패킷 직렬화 ─────────────────────────────────────────────────────────────

def packet_f1(t):
    # 손끝당 택셀 그리드 스칼라 (F2B와 같은 rows,cols; 구조, 값은 스칼라)
    tips = []
    for i in range(5):
        grid = finger_grid_scalar(t, i, GRID_ROWS, GRID_COLS)
        tips.append(",".join(f"{v:.3f}" for v in grid))
    return f"F1:{GRID_ROWS},{GRID_COLS};" + "|".join(tips)


def packet_f2a(t):
    vecs = []
    for i in range(5):
        fx, fy, fz = finger_vector(t, i)
        vecs.append(f"{fx:.3f},{fy:.3f},{fz:.3f}")
    return "F2A:" + "|".join(vecs)


def packet_f2b(t):
    tips = []
    for i in range(5):
        grid = finger_grid(t, i, GRID_ROWS, GRID_COLS)
        tips.append(",".join(f"{v:.2f}" for v in grid))
    return f"F2B:{GRID_ROWS},{GRID_COLS};" + "|".join(tips)


PACKERS = {"f1": packet_f1, "f2a": packet_f2a, "f2b": packet_f2b}
MODE_KEYS = {"1": "f1", "2": "f2a", "3": "f2b"}


def read_key_nonblocking():
    """stdin에서 키 입력 논블로킹 확인 (엔터 필요)."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip().lower()
    return None


def main():
    parser = argparse.ArgumentParser(description="XHand 택타일 더미 퍼블리셔")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--mode", choices=["f1", "f2a", "f2b"], default="f1")
    parser.add_argument("--hz",   type=float, default=SEND_HZ)
    args = parser.parse_args()

    ctx  = zmq.Context()
    sock = make_pub_socket(ctx, args.host, args.port)
    mode = args.mode

    print(f"{GREEN}택타일 더미 퍼블리셔 시작{RESET}  tcp://{args.host}:{args.port}  {args.hz:.0f}Hz")
    print(f"모드 전환: {CYAN}1{RESET}=F1(히트맵)  {CYAN}2{RESET}=F2A(손끝 벡터)  {CYAN}3{RESET}=F2B(벡터장)  + 엔터")
    print(f"현재 모드: {YELLOW}{mode.upper()}{RESET}\n")

    period = 1.0 / args.hz
    t0 = time.time()
    sent = 0
    last_report = t0

    try:
        while True:
            now = time.time()
            t = now - t0

            key = read_key_nonblocking()
            if key in MODE_KEYS:
                mode = MODE_KEYS[key]
                print(f"\n모드 변경 → {YELLOW}{mode.upper()}{RESET}")
            elif key in ("q", "quit", "exit"):
                break

            packet = PACKERS[mode](t)
            sock.send_string(packet)
            sent += 1

            if now - last_report >= 1.0:
                peak = max(finger_intensity(t, i) for i in range(5))
                bars = " ".join(
                    f"{FINGERS[i][:2]}:{'█' * int(finger_intensity(t, i) * 8):<8s}"
                    for i in range(5)
                )
                print(f"\r[{mode.upper():3s}] {sent}pkt/s  {bars}  peak={peak:.2f}",
                      end="", flush=True)
                sent = 0
                last_report = now

            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n종료")
        sock.close(0)
        ctx.term()


if __name__ == "__main__":
    main()
