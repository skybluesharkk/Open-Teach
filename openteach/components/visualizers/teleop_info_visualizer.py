import io
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import cv2
import zmq
import rbpodo as rb

from openteach.components import Component
from openteach.utils.network import ZMQKeypointSubscriber, ZMQCompressedImageTransmitter


class TeleopInfoVisualizer(Component):
    """
    손목 이동량 + 로봇 TCP 정보를 이미지로 렌더링해서 Quest 화면에 전송.
    GraphStream.cs가 oculus_graph_port(15001)를 구독해서 표시함.
    """

    def __init__(self, host, transformed_keypoint_port, oculus_feedback_port,
                 robot_ip, display_plot=False):
        self.notify_component_start('teleop info visualizer')

        self._sub = ZMQKeypointSubscriber(host, transformed_keypoint_port,
                                          'transformed_hand_frame')
        self._transmitter = ZMQCompressedImageTransmitter(host, oculus_feedback_port)

        # 로봇 연결은 stream() 시작 시 지연 초기화 — 오퍼레이터 소켓 간섭 방지
        self._robot_ip = robot_ip
        self._robot = None
        self._rc = None

        self._wrist_init = None
        self._fig, self._ax = plt.subplots(figsize=(6, 4), dpi=100)

    def _get_wrist_delta(self):
        data = self._sub.recv_keypoints(flags=zmq.NOBLOCK)
        if data is None:
            return None
        frame = np.array(data).reshape(4, 3)
        wrist = frame[0]
        if self._wrist_init is None:
            self._wrist_init = wrist.copy()
        return wrist - self._wrist_init

    def _get_robot_tcp(self):
        if self._robot is None:
            return None
        try:
            _, tcp = self._robot.get_tcp_info(self._rc)
            return tcp
        except Exception:
            return None

    def _read_tap_status(self):
        try:
            raw = open('/tmp/tap_status.txt').read().strip()
            status, coords = raw.split('|')
            xyz = [float(v) for v in coords.split(',')]
            return status, xyz
        except Exception:
            return 'WAITING', [0, 0, 0]

    def _render(self, delta, tcp):
        ax = self._ax
        ax.clear()
        ax.set_facecolor('#1a1a2e')
        self._fig.patch.set_facecolor('#1a1a2e')
        ax.set_xlim(0, 1); ax.set_ylim(0, 1)
        ax.axis('off')

        WHITE  = '#ffffff'
        CYAN   = '#00d4ff'
        YELLOW = '#ffd700'
        GREEN  = '#00ff88'
        GRAY   = '#888888'
        RED    = '#ff4444'
        ORANGE = '#ff9900'

        ax.text(0.5, 0.97, 'RB Teleop Monitor',
                ha='center', va='top', color=WHITE,
                fontsize=14, fontweight='bold', transform=ax.transAxes)

        # Tap 상태 표시
        tap_status, tap_xyz = self._read_tap_status()
        status_color = {
            'WAITING': GRAY,
            'MOVING':  ORANGE,
            'DONE':    GREEN,
            'FAIL':    RED,
        }.get(tap_status, GRAY)
        status_label = {
            'WAITING': '대기중 (Index→이동→Middle)',
            'MOVING':  f'이동중 → {tap_xyz[0]:.0f}, {tap_xyz[1]:.0f}, {tap_xyz[2]:.0f} mm',
            'DONE':    f'완료   → {tap_xyz[0]:.0f}, {tap_xyz[1]:.0f}, {tap_xyz[2]:.0f} mm',
            'FAIL':    f'실패   → {tap_xyz[0]:.0f}, {tap_xyz[1]:.0f}, {tap_xyz[2]:.0f} mm',
        }.get(tap_status, '대기중')

        ax.text(0.5, 0.89, status_label,
                ha='center', va='top', color=status_color,
                fontsize=11, fontweight='bold', transform=ax.transAxes)

        ax.axhline(y=0.82, color=GRAY, linewidth=0.5)

        # 손목 델타
        ax.text(0.05, 0.76, '손목 이동 (cm)',
                ha='left', va='top', color=CYAN,
                fontsize=10, fontweight='bold', transform=ax.transAxes)

        if delta is not None:
            dx, dy, dz = delta * 100  # m → cm
            dom = int(np.argmax(np.abs([dx, dy, dz])))
            colors = [WHITE, WHITE, WHITE]
            colors[dom] = YELLOW

            ax.text(0.05, 0.73, f'X: {dx:+.1f}', ha='left', va='top',
                    color=colors[0], fontsize=12, fontfamily='monospace',
                    transform=ax.transAxes)
            ax.text(0.37, 0.73, f'Y: {dy:+.1f}', ha='left', va='top',
                    color=colors[1], fontsize=12, fontfamily='monospace',
                    transform=ax.transAxes)
            ax.text(0.69, 0.73, f'Z: {dz:+.1f}', ha='left', va='top',
                    color=colors[2], fontsize=12, fontfamily='monospace',
                    transform=ax.transAxes)

            # 주 이동 방향 바
            ax_labels = ['X', 'Y', 'Z']
            vals = [dx, dy, dz]
            bar_y = [0.60, 0.52, 0.44]
            for i, (lbl, val, by) in enumerate(zip(ax_labels, vals, bar_y)):
                norm = np.clip(val / 20, -1, 1)  # ±20cm 기준 정규화
                color = GREEN if abs(val) > 2 else GRAY
                bar_len = abs(norm) * 0.35
                bar_x = 0.5 if norm >= 0 else 0.5 - bar_len
                rect = mpatches.FancyBboxPatch(
                    (bar_x, by - 0.015), bar_len, 0.03,
                    boxstyle="round,pad=0.002", color=color, alpha=0.8,
                    transform=ax.transAxes
                )
                ax.add_patch(rect)
                ax.text(0.15, by, lbl, ha='center', va='center',
                        color=WHITE, fontsize=9, transform=ax.transAxes)
                ax.axvline(x=0.5, ymin=by-0.02, ymax=by+0.02, color=GRAY, linewidth=0.5)
        else:
            ax.text(0.5, 0.65, 'Quest 미연결',
                    ha='center', va='center', color=GRAY,
                    fontsize=11, transform=ax.transAxes)

        ax.axhline(y=0.38, color=GRAY, linewidth=0.5)

        # 로봇 TCP
        ax.text(0.05, 0.33, '로봇 TCP (mm)',
                ha='left', va='top', color=CYAN,
                fontsize=10, fontweight='bold', transform=ax.transAxes)

        if tcp is not None:
            ax.text(0.05, 0.24, f'X: {tcp[0]:+.1f}', ha='left', va='top',
                    color=WHITE, fontsize=12, fontfamily='monospace',
                    transform=ax.transAxes)
            ax.text(0.37, 0.24, f'Y: {tcp[1]:+.1f}', ha='left', va='top',
                    color=WHITE, fontsize=12, fontfamily='monospace',
                    transform=ax.transAxes)
            ax.text(0.69, 0.24, f'Z: {tcp[2]:+.1f}', ha='left', va='top',
                    color=WHITE, fontsize=12, fontfamily='monospace',
                    transform=ax.transAxes)
            dist = np.linalg.norm(tcp[:3])
            ax.text(0.5, 0.10, f'원점 거리: {dist:.1f} mm',
                    ha='center', va='top', color=YELLOW,
                    fontsize=10, transform=ax.transAxes)
        else:
            ax.text(0.5, 0.20, '로봇 미연결',
                    ha='center', va='center', color=GRAY,
                    fontsize=11, transform=ax.transAxes)

        buf = io.BytesIO()
        self._fig.savefig(buf, format='png', bbox_inches='tight',
                          facecolor=self._fig.get_facecolor())
        buf.seek(0)
        img_arr = np.frombuffer(buf.read(), dtype=np.uint8)
        img_bgr = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
        return img_bgr

    def stream(self):
        import time
        # 오퍼레이터가 먼저 연결 후 3초 뒤에 로봇 연결
        time.sleep(3)
        try:
            self._robot = rb.Cobot(self._robot_ip)
            self._rc = rb.ResponseCollector()
        except Exception as e:
            print(f'[TeleopInfoVisualizer] 로봇 연결 실패: {e}')

        while True:
            try:
                delta   = self._get_wrist_delta()
                tcp     = self._get_robot_tcp()
                img_bgr = self._render(delta, tcp)
                if img_bgr is not None:
                    self._transmitter.send_image(img_bgr)
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f'[TeleopInfoVisualizer] 오류: {e}')
                import time; import time; time.sleep(0.1)
