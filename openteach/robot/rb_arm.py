import numpy as np
import rbpodo as rb
from scipy.spatial.transform import Rotation

from .robot import RobotWrapper

# rbpodo TCP format: [X(mm), Y(mm), Z(mm), Rx(deg), Ry(deg), Rz(deg)]
# Open-Teach Cartesian format: [x(m), y(m), z(m), qx, qy, qz, qw]

_SERVO_T1 = 0.02   # look-ahead time (s)
_SERVO_T2 = 0.1    # smoothing time (s)
_SERVO_GAIN = 1.0
_SERVO_ALPHA = 1.0


def _tcp_to_homo(tcp):
    """[X(mm), Y(mm), Z(mm), Rx, Ry, Rz(deg)] → 4×4 homogeneous matrix (meters)"""
    t = tcp[:3] / 1000.0
    R = Rotation.from_euler('xyz', tcp[3:], degrees=True).as_matrix()
    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = t
    return H


def _cart_to_tcp(cartesian_coords):
    """[x(m), y(m), z(m), qx, qy, qz, qw] → [X(mm), Y(mm), Z(mm), Rx, Ry, Rz(deg)]"""
    t_mm = cartesian_coords[:3] * 1000.0
    euler_deg = Rotation.from_quat(cartesian_coords[3:]).as_euler('xyz', degrees=True)
    return np.concatenate([t_mm, euler_deg])


class RBArm(RobotWrapper):
    def __init__(self, robot_ip: str, speed_bar: float = 0.1, record: bool = False):
        self._rc = rb.ResponseCollector()
        self._robot = rb.Cobot(robot_ip)

        self._robot.set_operation_mode(self._rc, rb.OperationMode.Real)
        self._robot.set_speed_bar(self._rc, speed_bar)
        self._robot.set_collision_onoff(self._rc, True)
        self._robot.flush(self._rc)

        self._data_frequency = 60
        self._servo_mode = False  # arm_control 첫 호출 시 disable_waiting_ack로 전환

        # [0,0,0,0,0,0] 홈 자세는 IK 특이점 — 작업 자세로 이동
        self._move_to_working_pose()

    # ── 초기화 헬퍼 ──────────────────────────────────────────────────────

    def _move_to_working_pose(self):
        """텔레오퍼레이션 시작 전 작업 자세로 이동"""
        working_pose = np.array([350.51, 3.99, 80.47, -85.71, 90.44, -0.01])
        self._robot.move_j(self._rc, working_pose, 30, 60)
        if self._robot.wait_for_move_started(self._rc, 2.0).is_success():
            self._robot.wait_for_move_finished(self._rc)
        print('[RBArm] 작업 자세 이동 완료')

    # ── RobotWrapper 필수 프로퍼티 ──────────────────────────────────────

    @property
    def name(self):
        return 'rb_arm'

    @property
    def recorder_functions(self):
        return {
            'joint_states': self.get_joint_state,
            'cartesian_states': self.get_cartesian_position,
        }

    @property
    def data_frequency(self):
        return self._data_frequency

    # ── 상태 조회 ────────────────────────────────────────────────────────

    def get_joint_state(self):
        joints = []
        for sv in [
            rb.SystemVariable.SD_J0_ANG, rb.SystemVariable.SD_J1_ANG,
            rb.SystemVariable.SD_J2_ANG, rb.SystemVariable.SD_J3_ANG,
            rb.SystemVariable.SD_J4_ANG, rb.SystemVariable.SD_J5_ANG,
        ]:
            _, val = self._robot.get_system_variable(self._rc, sv)
            joints.append(val)
        return np.array(joints)

    def get_joint_position(self):
        try:
            return self.get_joint_state()
        except Exception:
            return None

    def get_cartesian_position(self):
        _, tcp = self._robot.get_tcp_info(self._rc)
        return tcp  # [X(mm), Y(mm), Z(mm), Rx, Ry, Rz(deg)]

    def get_pose(self):
        """FrankaArmOperator 호환: {'position': 4×4 homo matrix (meters)} 반환"""
        _, tcp = self._robot.get_tcp_info(self._rc)
        return {'position': _tcp_to_homo(tcp)}

    # ── 이동 명령 ────────────────────────────────────────────────────────

    def home(self):
        self._robot.enable_waiting_ack(self._rc)
        self._robot.move_j(self._rc, np.zeros(6), 30, 60)
        if self._robot.wait_for_move_started(self._rc, 0.5).is_success():
            self._robot.wait_for_move_finished(self._rc)
        self._robot.disable_waiting_ack(self._rc)

    def move(self, input_angles):
        """관절 공간 이동 (deg)"""
        self._robot.enable_waiting_ack(self._rc)
        self._robot.move_j(self._rc, np.asarray(input_angles, dtype=float), 30, 60)
        if self._robot.wait_for_move_started(self._rc, 0.5).is_success():
            self._robot.wait_for_move_finished(self._rc)
        self._robot.disable_waiting_ack(self._rc)

    def move_coords(self, cartesian_coords, duration=3):
        """카테시안 이동 [X(mm), Y(mm), Z(mm), Rx, Ry, Rz(deg)]"""
        self._robot.enable_waiting_ack(self._rc)
        self._robot.move_l(self._rc, np.asarray(cartesian_coords, dtype=float), 50, 100)
        if self._robot.wait_for_move_started(self._rc, 0.5).is_success():
            self._robot.wait_for_move_finished(self._rc)
        self._robot.disable_waiting_ack(self._rc)

    def arm_control(self, cartesian_coords):
        """
        실시간 텔레오퍼레이션 — RBArmOperator에서 60Hz로 호출됨.
        cartesian_coords: [x(m), y(m), z(m), qx, qy, qz, qw]
        """
        if not self._servo_mode:
            self._robot.disable_waiting_ack(self._rc)
            self._servo_mode = True

        tcp_target = _cart_to_tcp(np.asarray(cartesian_coords, dtype=float))
        self._robot.move_servo_l(
            self._rc, tcp_target,
            _SERVO_T1, _SERVO_T2, _SERVO_GAIN, _SERVO_ALPHA,
        )
