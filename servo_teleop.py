#!/usr/bin/env python3
"""
Quest → RB Arm 실시간 servo 텔레오퍼레이션

Open-Teach 파이프라인 없이 Quest 키포인트를 직접 수신,
move_servo_l로 20Hz 연속 추적 + Quest 패널 시각화.

사용법:
    conda activate rbpodo
    python servo_teleop.py --host 192.168.50.49 --robot-ip 10.0.2.7

모드:
    Index Pinch  (초록/Hand): 손 이동만, 로봇 대기
    Middle Pinch (파란/Arm) : 추적 시작 — 첫 프레임 기준으로 손 이동량만큼 로봇 이동
    Index Pinch  복귀       : 추적 정지
"""

import argparse
import csv
import io
import re
import signal
import threading
import time
from pathlib import Path

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import zmq
import rbpodo as rb
from scipy.spatial.transform import Rotation

# ── 기본값 ────────────────────────────────────────────────────────────────
DEFAULT_HOST          = "192.168.50.49"
DEFAULT_ROBOT_IP      = "10.0.2.7"
DEFAULT_KEYPOINT_PORT = 8087
DEFAULT_BUTTON_PORT   = 8095
DEFAULT_TELEOP_PORT   = 8100
DEFAULT_GRAPH_PORT    = 15001   # Quest GraphStream.cs 구독 포트
DEFAULT_CAM_PORT      = 10505   # Quest CameraOneStreamer.cs 구독 포트 (빈 이미지 전송해 빨간 테두리 제거)

NUM_KEYPOINTS  = 24
WRIST_INDEX    = 0

CONTROL_HZ     = 20      # servo 명령 주기 (Hz)
VIZ_HZ         = 10      # Quest 패널 갱신 주기 (Hz)
SERVO_T1       = 0.06    # look-ahead time: 1/CONTROL_HZ(=0.05s)보다 약간 크게
SERVO_T2       = 0.20    # 스무딩 시간
SERVO_GAIN     = 1.0
SERVO_ALPHA    = 1.0

SPEED_BAR      = 0.15
MOVE_SCALE     = 1.0
MAX_FRAME_M    = 0.03    # 프레임당 최대 이동 30mm (속도 제한, 총 이동 제한 아님)
DEAD_ZONE_M    = 0.003   # 3mm 미만 손 떨림 무시
JUMP_GUARD_M   = 0.10    # 프레임 간 손목 10cm 이상 이동 시 트래킹 손실로 판단
QUEST_TIMEOUT  = 0.5

# TCP 작업공간 제한 (m, 로봇 base 기준) — RB5-850 reach 927.7mm 내 보수적 범위
# rb5_workspace.html 시각화 참고
WORKSPACE = {
    'x': (0.050, 0.500),    # 전후: 기저부 뒤 침범 방지 ~ 앞 500mm
    'y': (-0.300, 0.100),   # 좌우
    'z': (0.200, 0.700),    # 상하: 바닥/과상승 방지
}
R_MIN_XY       = 0.250   # base 기둥 주변 특이점 회피: 수평 반경 최소 250mm
LOST_ACCEPT_S  = 1.0     # 점프 상태가 이 시간 이상 지속되면 새 좌표계로 수용 (Quest 재시작 대응)

# 원통 표면 밀어내기 시 박스를 함께 만족하는 각도 범위 (X축 기준, rad)
# cosθ ≥ xmin/R (X 하한), sinθ ≤ ymax/R (Y 상한), sinθ ≥ ymin/R (Y 하한)
_th_lim   = np.arccos(np.clip(WORKSPACE['x'][0] / R_MIN_XY, -1.0, 1.0))
THETA_MAX = min(_th_lim,  np.arcsin(np.clip(WORKSPACE['y'][1] / R_MIN_XY, -1.0, 1.0)))
THETA_MIN = max(-_th_lim, np.arcsin(np.clip(WORKSPACE['y'][0] / R_MIN_XY, -1.0, 1.0)))

# 홈 TCP 위치 (mm) — 수평 반경 √(255²+120²)≈282mm > R_MIN_XY, 원통 밖에서 시작
HOME_TCP_POS   = np.array([255.0, -120.0, 553.0])

R_QUEST_TO_ROBOT = np.array([
    [ 0,  0,  1],
    [-1,  0,  0],
    [ 0,  1,  0],
], dtype=float)

# ANSI 컬러 (터미널용)
GREEN  = "\033[92m"
BLUE   = "\033[94m"
YELLOW = "\033[93m"
RED    = "\033[91m"
RESET  = "\033[0m"


def color(text, code):
    return f"{code}{text}{RESET}"


# ── ZMQ 헬퍼 ──────────────────────────────────────────────────────────────

def make_pull_socket(ctx, host, port, name):
    sock = ctx.socket(zmq.PULL)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt(zmq.RCVTIMEO, 1)
    endpoint = f"tcp://{host}:{port}"
    try:
        sock.bind(endpoint)
    except zmq.ZMQError as exc:
        raise RuntimeError(
            f"{name} 소켓 바인드 실패 ({endpoint}). "
            f"teleop.py나 다른 모니터가 실행 중이지 않은지 확인하세요."
        ) from exc
    return sock


def make_pub_socket(ctx, host, port):
    sock = ctx.socket(zmq.PUB)
    sock.bind(f"tcp://{host}:{port}")
    return sock


# ── 키포인트 파싱 ──────────────────────────────────────────────────────────

def parse_keypoints(packet):
    text = packet.decode().strip()
    if ":" not in text:
        raise ValueError(f"예상치 못한 패킷: {text[:80]!r}")
    _, payload = text.split(":", 1)
    tokens = re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", payload)
    values = [float(v) for v in tokens]
    if len(values) < NUM_KEYPOINTS * 3:
        raise ValueError(f"키포인트 부족: {len(values)//3}/{NUM_KEYPOINTS}")
    return np.asarray(values[:NUM_KEYPOINTS * 3], dtype=float).reshape(NUM_KEYPOINTS, 3)


def decode_mode(packet, previous):
    if packet is None:
        return previous
    token = packet.decode(errors="ignore").strip()
    if token == "Low":
        return "STOP"
    if token == "High":
        return "CONT"
    return previous  # "None" or unknown → keep previous state


# ── 좌표 변환 ──────────────────────────────────────────────────────────────

def tcp_to_pos_quat(tcp):
    pos  = tcp[:3] / 1000.0
    quat = Rotation.from_euler('xyz', tcp[3:], degrees=True).as_quat()
    return pos, quat


def pos_quat_to_tcp(pos, quat):
    t_mm      = pos * 1000.0
    euler_deg = Rotation.from_quat(quat).as_euler('xyz', degrees=True)
    return np.concatenate([t_mm, euler_deg])


# ── Quest 패널 시각화 ──────────────────────────────────────────────────────

def render_frame(tracking, delta_robot, tcp_current, homing=False, notice=None):
    """
    matplotlib 으로 이미지를 그려 JPEG bytes 반환.
    tracking   : bool
    delta_robot: ndarray(3,) 또는 None — Robot 좌표계 손목 delta (m)
    tcp_current: ndarray(6,) 또는 None — 현재 로봇 TCP [mm, mm, mm, deg, deg, deg]
    """
    C = dict(
        bg     = '#1a1a2e',
        white  = '#ffffff',
        cyan   = '#00d4ff',
        yellow = '#ffd700',
        green  = '#00ff88',
        gray   = '#888888',
        red    = '#ff4444',
        orange = '#ff9900',
    )

    fig, ax = plt.subplots(figsize=(5, 4), dpi=100)
    fig.patch.set_facecolor(C['bg'])
    ax.set_facecolor(C['bg'])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis('off')

    # 제목
    ax.text(0.5, 0.97, 'RB Arm  —  Servo Teleop',
            ha='center', va='top', color=C['white'],
            fontsize=13, fontweight='bold', transform=ax.transAxes)

    if homing:
        status_txt   = '⟳ HOMING  (Moving to working pose...)'
        status_color = C['orange']
    elif tracking:
        status_txt   = '● TRACKING  (Index Pinch to stop)'
        status_color = C['green']
    else:
        status_txt   = '○ STANDBY  (Middle Pinch to start)'
        status_color = C['gray']
    ax.text(0.5, 0.88, status_txt,
            ha='center', va='top', color=status_color,
            fontsize=10, fontweight='bold', transform=ax.transAxes)

    if notice:
        ax.text(0.5, 0.81, notice,
                ha='center', va='top', color=C['red'],
                fontsize=8, fontweight='bold', transform=ax.transAxes)

    ax.axhline(y=0.81, color=C['gray'], linewidth=0.5)

    # ── 손목 delta ──────────────────────────────────────────
    ax.text(0.05, 0.78, 'Wrist Delta  (Robot frame, cm)',
            ha='left', va='top', color=C['cyan'],
            fontsize=9, fontweight='bold', transform=ax.transAxes)

    if delta_robot is not None:
        dx, dy, dz = delta_robot * 100
        dom = int(np.argmax(np.abs([dx, dy, dz])))
        val_colors = [C['white'], C['white'], C['white']]
        val_colors[dom] = C['yellow']

        for i, (lbl, val, xpos) in enumerate(zip(['X', 'Y', 'Z'],
                                                   [dx, dy, dz],
                                                   [0.05, 0.38, 0.71])):
            ax.text(xpos, 0.72, f'{lbl}: {val:+.1f}',
                    ha='left', va='top', color=val_colors[i],
                    fontsize=12, fontfamily='monospace', transform=ax.transAxes)

        # 축별 바
        bar_tops = [0.60, 0.52, 0.44]
        for i, (lbl, val, by) in enumerate(zip(['X', 'Y', 'Z'],
                                                [dx, dy, dz], bar_tops)):
            norm     = np.clip(val / 15, -1, 1)   # ±15cm 기준
            bar_clr  = C['green'] if abs(val) > 1 else C['gray']
            bar_len  = abs(norm) * 0.34
            bar_x    = 0.5 if norm >= 0 else 0.5 - bar_len
            rect = mpatches.FancyBboxPatch(
                (bar_x, by - 0.014), bar_len, 0.028,
                boxstyle="round,pad=0.002",
                color=bar_clr, alpha=0.85,
                transform=ax.transAxes,
            )
            ax.add_patch(rect)
            ax.axvline(x=0.5, color=C['gray'], linewidth=0.6,
                       ymin=by - 0.02, ymax=by + 0.02)
            ax.text(0.14, by, lbl, ha='center', va='center',
                    color=C['white'], fontsize=9, transform=ax.transAxes)
    else:
        ax.text(0.5, 0.58, 'Quest not connected',
                ha='center', va='center', color=C['gray'],
                fontsize=11, transform=ax.transAxes)

    ax.axhline(y=0.36, color=C['gray'], linewidth=0.5)

    # ── 로봇 TCP ────────────────────────────────────────────
    ax.text(0.05, 0.33, 'Robot TCP  (mm)',
            ha='left', va='top', color=C['cyan'],
            fontsize=9, fontweight='bold', transform=ax.transAxes)

    if tcp_current is not None:
        for i, (lbl, val, xpos) in enumerate(zip(['X', 'Y', 'Z'],
                                                   tcp_current[:3],
                                                   [0.05, 0.38, 0.71])):
            ax.text(xpos, 0.25, f'{lbl}: {val:+.1f}',
                    ha='left', va='top', color=C['white'],
                    fontsize=12, fontfamily='monospace', transform=ax.transAxes)
        dist = np.linalg.norm(tcp_current[:3])
        ax.text(0.5, 0.12, f'Dist from origin  {dist:.1f} mm',
                ha='center', va='top', color=C['yellow'],
                fontsize=10, transform=ax.transAxes)
    else:
        ax.text(0.5, 0.22, 'Robot not connected',
                ha='center', va='center', color=C['gray'],
                fontsize=11, transform=ax.transAxes)

    buf = io.BytesIO()
    fig.savefig(buf, format='png', bbox_inches='tight',
                facecolor=fig.get_facecolor())
    buf.seek(0)
    img_arr = np.frombuffer(buf.read(), dtype=np.uint8)
    img_bgr = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
    plt.close(fig)
    return img_bgr


class VizState:
    """메인 루프 ↔ 시각화 스레드 공유 상태"""
    def __init__(self):
        self.tracking     = False
        self.homing       = False  # 홈포지션 이동 중
        self.notice       = None   # Quest 패널에 표시할 경고/알림 문구
        self.delta_robot  = None   # ndarray(3,) Robot 좌표계 delta (m)
        self.tcp_current  = None   # ndarray(6,) 현재 TCP
        self.running      = True


def cam_blank_thread_fn(host: str, cam_port: int):
    """Quest CameraOneStreamer 패널에 검정 이미지를 계속 전송 — 빨간 테두리 제거."""
    ctx  = zmq.Context()
    sock = make_pub_socket(ctx, host, cam_port)
    blank = np.zeros((360, 640, 3), dtype=np.uint8)
    _, buf = cv2.imencode('.jpg', blank, [cv2.IMWRITE_JPEG_QUALITY, 50])
    data = buf.tobytes()
    while True:
        try:
            sock.send(data)
            time.sleep(0.1)   # 10Hz면 충분
        except Exception:
            break
    sock.close(0)
    ctx.term()


def viz_thread_fn(state: VizState, host: str, graph_port: int):
    ctx  = zmq.Context()
    sock = make_pub_socket(ctx, host, graph_port)
    period = 1.0 / VIZ_HZ
    last_t = 0.0

    while state.running:
        now = time.time()
        if now - last_t < period:
            time.sleep(0.005)
            continue
        last_t = now

        try:
            img = render_frame(state.tracking, state.delta_robot, state.tcp_current,
                               homing=state.homing, notice=state.notice)
            if img is not None:
                _, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                sock.send(buf.tobytes())
        except Exception as e:
            print(f"\n[Viz] 렌더 오류: {e}")

    sock.close(0)
    ctx.term()


# ── 메인 ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Quest → RB Arm servo 텔레오퍼레이션")
    parser.add_argument("--host",          default=DEFAULT_HOST)
    parser.add_argument("--robot-ip",      default=DEFAULT_ROBOT_IP)
    parser.add_argument("--keypoint-port", type=int, default=DEFAULT_KEYPOINT_PORT)
    parser.add_argument("--button-port",   type=int, default=DEFAULT_BUTTON_PORT)
    parser.add_argument("--teleop-port",   type=int, default=DEFAULT_TELEOP_PORT)
    parser.add_argument("--graph-port",    type=int, default=DEFAULT_GRAPH_PORT)
    parser.add_argument("--cam-port",      type=int, default=DEFAULT_CAM_PORT)
    parser.add_argument("--scale",  type=float, default=MOVE_SCALE,  help="손→로봇 이동 배율")
    parser.add_argument("--speed",  type=float, default=SPEED_BAR,   help="speed_bar (0~1)")
    parser.add_argument("--hz",     type=int,   default=CONTROL_HZ,  help="servo 명령 주기")
    parser.add_argument("--log",    type=str,   default="/tmp/servo_teleop_log.csv", help="이동 로그 CSV 경로")
    args = parser.parse_args()

    loop_period = 1.0 / args.hz
    t1 = max(SERVO_T1, loop_period * 1.2)

    # ── 로봇 초기화 ──────────────────────────────────────────────────────
    print("로봇 연결 중...")
    rc    = rb.ResponseCollector()
    robot = rb.Cobot(args.robot_ip)
    robot.set_operation_mode(rc, rb.OperationMode.Real)
    robot.set_speed_bar(rc, args.speed)
    robot.set_collision_onoff(rc, True)
    robot.flush(rc)
    print(color("로봇 연결 완료", GREEN))

    # ── ZMQ 소켓 ─────────────────────────────────────────────────────────
    ctx      = zmq.Context()
    key_sock = make_pull_socket(ctx, args.host, args.keypoint_port, "keypoint")
    btn_sock = make_pull_socket(ctx, args.host, args.button_port,   "button")
    tel_sock = make_pull_socket(ctx, args.host, args.teleop_port,   "teleop")

    # ── 시각화 스레드 먼저 시작 (homing 상태 표시를 위해) ─────────────────
    viz_state = VizState()
    vt = threading.Thread(
        target=viz_thread_fn,
        args=(viz_state, args.host, args.graph_port),
        daemon=True,
    )
    vt.start()
    print(color(f"Quest viz panel started (port {args.graph_port})", GREEN))

    # 빨간 테두리 제거: 카메라 패널에 검정 이미지 전송
    ct = threading.Thread(
        target=cam_blank_thread_fn,
        args=(args.host, args.cam_port),
        daemon=True,
    )
    ct.start()

    # ── 홈포지션 확인 및 이동 ─────────────────────────────────────────────
    working_pose = np.array([0.0, -50.0, 135.0, -70.0, 85.0, -5.0])
    joint_vars = [
        rb.SystemVariable.SD_J0_ANG, rb.SystemVariable.SD_J1_ANG,
        rb.SystemVariable.SD_J2_ANG, rb.SystemVariable.SD_J3_ANG,
        rb.SystemVariable.SD_J4_ANG, rb.SystemVariable.SD_J5_ANG,
    ]
    current_joints = np.array([robot.get_system_variable(rc, v)[1] for v in joint_vars])
    joint_error = np.max(np.abs(current_joints - working_pose))
    if joint_error > 5.0:
        print(f"Home diff {joint_error:.1f} deg > 5 deg — moving to working pose...")
        viz_state.homing = True
        robot.enable_waiting_ack(rc)
        robot.move_j(rc, working_pose, 30, 60)
        if robot.wait_for_move_started(rc, 2.0).is_success():
            robot.wait_for_move_finished(rc)
        robot.disable_waiting_ack(rc)
        viz_state.homing = False
        print(color("Working pose reached", GREEN))
    else:
        print(color(f"Already at working pose (max diff {joint_error:.1f} deg) — skipping", GREEN))

    _, tcp_init = robot.get_tcp_info(rc)

    # 홈 TCP로 이동 (관절 홈포지션의 TCP는 원통 안이므로, 정의된 원통 밖 홈 TCP로)
    if np.linalg.norm(np.asarray(tcp_init[:3]) - HOME_TCP_POS) > 5.0:
        tcp_home = np.asarray(tcp_init, dtype=float).copy()
        tcp_home[:3] = HOME_TCP_POS
        print(f"홈 TCP ({HOME_TCP_POS[0]:.0f}, {HOME_TCP_POS[1]:.0f}, {HOME_TCP_POS[2]:.0f})mm 로 이동...")
        viz_state.homing = True
        robot.enable_waiting_ack(rc)
        robot.move_l(rc, tcp_home, 50, 100)
        if robot.wait_for_move_started(rc, 2.0).is_success():
            robot.wait_for_move_finished(rc)
        robot.disable_waiting_ack(rc)
        viz_state.homing = False
        _, tcp_init = robot.get_tcp_info(rc)
        print(color("홈 TCP 도달", GREEN))

    # 안전망: 그래도 TCP가 특이점 금지 원통 안이면 반경 방향으로 밀어냄
    # (추후 홈 TCP를 바꿀 때 실수로 원통 안에 정의해도 여기서 걸러짐)
    r0_mm = float(np.hypot(tcp_init[0], tcp_init[1]))
    r_min_mm = R_MIN_XY * 1000.0
    if r0_mm < r_min_mm:
        push = (r_min_mm + 10.0) / r0_mm   # 10mm 여유
        tcp_home = np.asarray(tcp_init, dtype=float).copy()
        tcp_home[0] *= push
        tcp_home[1] *= push
        msg = f"WARN: start TCP r={r0_mm:.0f}mm inside singularity cylinder — pushing out"
        print(color(f"경고: 시작 TCP 수평 반경 {r0_mm:.0f}mm < {r_min_mm:.0f}mm — 원통 밖으로 밀어내는 중...", YELLOW))
        viz_state.homing = True
        viz_state.notice = msg
        robot.enable_waiting_ack(rc)
        robot.move_l(rc, tcp_home, 50, 100)
        if robot.wait_for_move_started(rc, 2.0).is_success():
            robot.wait_for_move_finished(rc)
        robot.disable_waiting_ack(rc)
        viz_state.homing = False
        _, tcp_init = robot.get_tcp_info(rc)
        print(color("특이점 원통 밖 시작 위치 도달", GREEN))

    viz_state.tcp_current = tcp_init.copy()
    print(f"초기 TCP: X={tcp_init[0]:.1f}  Y={tcp_init[1]:.1f}  Z={tcp_init[2]:.1f} mm\n")
    print(color(f"Camera blank sender started (port {args.cam_port})", GREEN))

    # ── 상태 변수 ─────────────────────────────────────────────────────────
    stop            = False
    connected       = False
    tracking        = False
    hand_init       = None
    robot_init_pos  = None
    robot_init_quat = None
    mode            = "UNKNOWN"
    last_mode       = "UNKNOWN"
    last_quest_t    = 0.0
    last_loop_t     = time.time()
    last_tcp_t      = 0.0
    wrist           = None

    def handle_signal(_sig, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    # ── 이동 로그 CSV ────────────────────────────────────────────────────
    log_path = Path(args.log)
    log_file = log_path.open("w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow([
        "time_s",
        "quest_dx_m", "quest_dy_m", "quest_dz_m",
        "robot_dx_m", "robot_dy_m", "robot_dz_m",
        "clamped_dx_m", "clamped_dy_m", "clamped_dz_m",
        "x_clamped", "y_clamped", "z_clamped",
        "tcp_x_mm", "tcp_y_mm", "tcp_z_mm",
        "tcp_rx_deg", "tcp_ry_deg", "tcp_rz_deg",
    ])
    log_start = time.time()

    print(color("servo 텔레오퍼레이션 준비", GREEN))
    print(f"host={args.host}  robot={args.robot_ip}")
    print(f"scale={args.scale}  speed_bar={args.speed}  hz={args.hz}  T1={t1:.3f}s")
    print(f"이동 로그: {log_path}")
    print()
    print("  1. Quest Stream 시작 → CONNECTED 표시 대기")
    print("  2. Index Pinch (초록) → 손을 원하는 시작 위치로")
    print("  3. Middle Pinch (파란) → 추적 시작, 손 따라 로봇 이동")
    print("  4. Index Pinch 복귀 → 추적 정지\n")

    # ── 메인 루프 ─────────────────────────────────────────────────────────
    wrist_last_valid = None   # 마지막 유효 손목 위치 (점프 감지 기준)
    tcp_prev_pos     = None   # 프레임 간 속도 제한용 (마지막 명령 위치)
    tracking_lost    = False  # 점프 감지로 트래킹 손실 상태
    lost_since       = 0.0    # 트래킹 손실 시작 시각
    try:
        while not stop:
            now = time.time()

            key_packet = None
            try:
                key_packet = key_sock.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                pass
            try:
                btn_pkt = btn_sock.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                btn_pkt = None
            try:
                tel_pkt = tel_sock.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                tel_pkt = None

            mode = decode_mode(tel_pkt, mode)

            if key_packet is not None:
                try:
                    keypoints    = parse_keypoints(key_packet)
                    wrist_new    = keypoints[WRIST_INDEX]
                    last_quest_t = now

                    # 점프 감지: 항상 마지막 유효 손목과 비교 (bad→bad 연속 통과 방지)
                    if wrist_last_valid is not None:
                        jump = np.linalg.norm(wrist_new - wrist_last_valid)
                        if jump > JUMP_GUARD_M:
                            if not tracking_lost:
                                tracking_lost = True
                                lost_since    = now
                                print(color(f"\n경고: 트래킹 손실 감지 ({jump*100:.0f}cm 점프) — 복구 대기", YELLOW))
                            elif now - lost_since > LOST_ACCEPT_S:
                                # 점프 상태 장기 지속 → Quest 좌표계가 바뀐 것 (재시작 등)
                                # 새 위치를 유효 기준으로 수용, 다음 패킷에서 재기준점 설정됨
                                wrist_last_valid = wrist_new.copy()
                                print(color("\n[수용] 새 좌표계 감지 — 새 손목 위치를 기준으로 수용", YELLOW))
                            continue

                    # 유효 패킷 — wrist 갱신
                    wrist            = wrist_new
                    wrist_last_valid = wrist_new

                    # 트래킹 손실에서 복구: 모드가 CONT면 현재 위치에서 자동 재개
                    if tracking_lost and mode == "CONT":
                        hand_init    = wrist.copy()
                        _, tcp_now   = robot.get_tcp_info(rc)
                        robot_init_pos, robot_init_quat = tcp_to_pos_quat(tcp_now)
                        tcp_prev_pos = robot_init_pos.copy()
                        tracking      = True
                        viz_state.tracking = True
                        tracking_lost = False
                        print(color("\n[복구] 트래킹 재연결 — 현재 위치에서 재기준점 설정", GREEN))
                    else:
                        tracking_lost = False

                    if not connected:
                        connected = True
                        print(color("\nCONNECTED: Quest 수신 시작", GREEN))
                        print(color("Middle Pinch로 추적을 시작하세요.", BLUE))
                except Exception as exc:
                    print(f"\n{color('파싱 오류: ' + str(exc), RED)}")

            # ── 모드 전환 ─────────────────────────────────────────────────
            if mode != last_mode:
                if mode == "CONT" and wrist is not None:
                    hand_init    = wrist.copy()
                    _, tcp_now   = robot.get_tcp_info(rc)
                    robot_init_pos, robot_init_quat = tcp_to_pos_quat(tcp_now)
                    tcp_prev_pos = robot_init_pos.copy()
                    viz_state.notice = None
                    tracking = True
                    viz_state.tracking = True
                    robot.disable_waiting_ack(rc)
                    print(color("\n[파란/ARM] 추적 시작", BLUE))
                    print(f"  손목 기준: X={hand_init[0]:.3f}  Y={hand_init[1]:.3f}  Z={hand_init[2]:.3f}")
                    print(f"  로봇 기준: X={robot_init_pos[0]*1000:.1f}  Y={robot_init_pos[1]*1000:.1f}  Z={robot_init_pos[2]*1000:.1f} mm")
                elif mode == "STOP":
                    tracking = False
                    viz_state.tracking    = False
                    viz_state.delta_robot = None
                    print(color("\n[초록/HAND] 추적 정지. Middle Pinch로 재시작", GREEN))
                last_mode = mode

            # ── 20Hz servo 루프 ───────────────────────────────────────────
            if now - last_loop_t < loop_period:
                time.sleep(0.001)
                continue
            last_loop_t = now

            # TCP를 2Hz로 폴링해서 viz에 전달 (로봇 쿼리 부담 최소화)
            if now - last_tcp_t > 0.5:
                try:
                    _, tcp_now = robot.get_tcp_info(rc)
                    viz_state.tcp_current = tcp_now.copy()
                    last_tcp_t = now
                except Exception:
                    pass

            if not tracking or wrist is None or hand_init is None:
                continue

            if now - last_quest_t > QUEST_TIMEOUT:
                print(color(f"\n경고: Quest 패킷 {QUEST_TIMEOUT}s 이상 없음 — 추적 일시 중단", YELLOW))
                tracking      = False
                viz_state.tracking = False
                tracking_lost = True   # 패킷 재개 시 자동 복구(재기준점) 경로로 진입
                lost_since    = now
                continue

            delta_quest = wrist - hand_init
            delta_robot = R_QUEST_TO_ROBOT @ delta_quest

            if np.linalg.norm(delta_robot) < DEAD_ZONE_M:
                continue

            target_pos_raw = robot_init_pos + delta_robot * args.scale

            # 1) 작업공간 클램핑 (절대 위치, base 기준)
            ws_target = np.array([
                np.clip(target_pos_raw[0], *WORKSPACE['x']),
                np.clip(target_pos_raw[1], *WORKSPACE['y']),
                np.clip(target_pos_raw[2], *WORKSPACE['z']),
            ])
            x_clamp = ws_target[0] != target_pos_raw[0]
            y_clamp = ws_target[1] != target_pos_raw[1]
            z_clamp = ws_target[2] != target_pos_raw[2]

            # 1-b) base 기둥 특이점 회피: 원통 안이면 표면으로 밀어내되,
            #      각도를 THETA 범위로 제한해 박스 한계도 함께 만족시킴
            r_xy = np.hypot(ws_target[0], ws_target[1])
            if 1e-6 < r_xy < R_MIN_XY:
                theta = np.clip(np.arctan2(ws_target[1], ws_target[0]), THETA_MIN, THETA_MAX)
                ws_target[0] = R_MIN_XY * np.cos(theta)
                ws_target[1] = R_MIN_XY * np.sin(theta)
                x_clamp = y_clamp = True

            # 2) 프레임당 속도 제한: 이전 명령에서 MAX_FRAME_M 이상 한 번에 이동 못 함
            if tcp_prev_pos is not None:
                frame_delta   = ws_target - tcp_prev_pos
                frame_clamped = np.clip(frame_delta, -MAX_FRAME_M, MAX_FRAME_M)
                target_pos    = tcp_prev_pos + frame_clamped
            else:
                target_pos = ws_target

            tcp_target   = pos_quat_to_tcp(target_pos, robot_init_quat)
            tcp_prev_pos = target_pos.copy()

            robot.move_servo_l(rc, tcp_target, t1, SERVO_T2, SERVO_GAIN, SERVO_ALPHA)

            # CSV 로그
            log_writer.writerow([
                f"{time.time() - log_start:.4f}",
                f"{delta_quest[0]:.6f}", f"{delta_quest[1]:.6f}", f"{delta_quest[2]:.6f}",
                f"{delta_robot[0]:.6f}", f"{delta_robot[1]:.6f}", f"{delta_robot[2]:.6f}",
                f"{(target_pos[0]-robot_init_pos[0]):.6f}", f"{(target_pos[1]-robot_init_pos[1]):.6f}", f"{(target_pos[2]-robot_init_pos[2]):.6f}",
                int(x_clamp), int(y_clamp), int(z_clamp),
                f"{tcp_target[0]:.2f}", f"{tcp_target[1]:.2f}", f"{tcp_target[2]:.2f}",
                f"{tcp_target[3]:.3f}", f"{tcp_target[4]:.3f}", f"{tcp_target[5]:.3f}",
            ])

            # viz 상태 갱신
            viz_state.delta_robot = delta_robot.copy()

            clamp_warn = ""
            if x_clamp or y_clamp or z_clamp:
                axes = "".join(a for a, c in zip("XYZ", [x_clamp, y_clamp, z_clamp]) if c)
                clamp_warn = color(f" [CLAMP:{axes}]", YELLOW)

            print(
                f"\r[추적] "
                f"Robot Δ X={delta_robot[0]*100:+5.1f}cm "
                f"Y={delta_robot[1]*100:+5.1f}cm "
                f"Z={delta_robot[2]*100:+5.1f}cm  │  "
                f"TCP X={tcp_target[0]:+7.1f} Y={tcp_target[1]:+7.1f} Z={tcp_target[2]:+7.1f} mm"
                f"{clamp_warn}  ",
                end="", flush=True,
            )

    finally:
        print("\n\n종료 중...")
        viz_state.running = False
        vt.join(timeout=2.0)
        try:
            robot.enable_waiting_ack(rc)
        except Exception:
            pass
        key_sock.close(0)
        btn_sock.close(0)
        tel_sock.close(0)
        ctx.term()
        log_file.close()
        print(f"이동 로그 저장: {log_path}")
        print("종료 완료")


if __name__ == "__main__":
    main()
