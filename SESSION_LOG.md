# RB 로봇 팔 텔레오퍼레이션 세션 로그

Meta Quest 3 + Open-Teach + rbpodo를 이용해 Rainbow Robotics RB5 로봇 팔 텔레오퍼레이션 구현 과정 전체 기록.

---

## 1. 프로젝트 개요

### 목표
- Meta Quest 3 VR 헤드셋으로 사람 손을 트래킹
- 손목 위치를 RB5 로봇 팔의 엔드이펙터로 매핑
- 실시간 텔레오퍼레이션 구현

### 사용 레포지토리
- `~/rbpodo`: Rainbow Robotics RB 시리즈 Python 클라이언트 라이브러리
- `~/Open-Teach`: NYU 제작 VR 텔레오퍼레이션 프레임워크 (fork)
- VS Code 워크스페이스: `Open-Teach/rb_teleop.code-workspace`

### 네트워크 구성 (최종)
```
WiFi 동글 (192.168.50.x)
  ├── PC (192.168.50.49)  ← 서버 (host_address)
  └── Meta Quest 3        ← VR 클라이언트

LAN 케이블 (enp14s0)
  └── 로봇 컨트롤박스 (10.0.2.100 → 10.0.2.7)
```

---

## 2. 의존성 분석 및 설치

### rbpodo 환경 (conda: `rbpodo`)
- Python 3.11.15
- numpy 2.4.6
- pybind11 3.0.4
- rbpodo 0.16.10

### Open-Teach 요구사항 (environment.yml)
- Python 3.10.13 → 3.11에서도 동작 확인
- numpy 1.26.2 → rbpodo가 `>=1.19` 요구하므로 2.4.6 유지 가능
- pytorch 1.12 → Python 3.11 미지원, 단 팔 텔레오퍼레이션에는 불필요

### 핵심 충돌 분석
| 패키지 | rbpodo | Open-Teach | 결론 |
|---|---|---|---|
| numpy | >=1.19 | ==1.26.2 | 현재 2.4.6 유지 (둘 다 호환) |
| pytorch 1.12 | 없음 | 시뮬레이션 전용 | 설치 생략 |
| pyrealsense2 | 없음 | 카메라용 | ImportError 방지 처리 |

### 설치 명령
```bash
conda run -n rbpodo pip install pyzmq hydra-core omegaconf scipy matplotlib h5py \
  opencv-python pandas pillow Flask gevent gunicorn tqdm ikpy shapely IPython blosc
conda run -n rbpodo pip install -e ~/Open-Teach
```

---

## 3. Open-Teach 아키텍처 이해

### 전체 파이프라인
```
Meta Quest (APK)
      │  ZMQ PUSH (포트 8087)
      ▼
OculusVRHandDetector        ← 프로세스 1
      │  ZMQ PUB (포트 8088, topic: 'right')
      ▼
TransformHandPositionCoords ← 프로세스 2
      │  ZMQ PUB (포트 8089, topic: 'transformed_hand_frame')
      ▼
[Robot]ArmOperator          ← 프로세스 3
      │  직접 함수 호출
      ▼
[Robot]Arm (RobotWrapper)   ← 로봇 컨트롤러
```

모든 컴포넌트는 `multiprocessing.Process`로 독립 실행, ZMQ로 통신.

### GestureDetector.cs (Unity APK 핵심)
```csharp
// 손 관절 위치를 Unity World Space로 전송
Vector3 bonePosition = bone.Transform.position;  // 절대 좌표 (m)
```
- `bone.Transform.position` = Unity **World Space** 절대 좌표 (left-handed, Y-up)
- Middle Pinch → `StreamAbsoluteData = true` → `"absolute:x,y,z|..."` 포맷으로 전송
- 포트 구성:
  - 8087: keypoint PUSH (Quest → Server)
  - 8095: resolution button (High/Low)
  - 8100: pause/continue 신호

### Quest 제스처 매핑 (SingleArmBot APK)
| 왼손 제스처 | 테두리 색 | 동작 |
|---|---|---|
| Index Pinch | 초록 | 손 모드 (STOP 신호) |
| Middle Pinch | 파란 | Arm 모드 (CONT 신호) |
| Ring Pinch | 빨간 | 일시정지 (STOP 신호) |
| Pinky Pinch | 검정 | 해상도 선택 |

### 키포인트 구조
- 총 24개 관절 (Unity World Space, 단위: m)
- index 0 = 손목 (wrist)
- index 6 = 검지 knuckle
- index 16 = 새끼 knuckle

### TransformHandPositionCoords 변환
```python
# 손 방향 프레임 계산
palm_normal    = normalize(cross(index_knuckle, pinky_knuckle))  # Z
palm_direction = normalize(index_knuckle + pinky_knuckle)        # Y
cross_product  = normalize(index_knuckle - pinky_knuckle)        # X

hand_dir_frame = [wrist_world_pos, cross_product, palm_normal, palm_direction]
# frame[0] = 손목 절대 위치 (Unity World Space)
```

---

## 4. RBArm 래퍼 구현

### 파일: `openteach/robot/rb_arm.py`

Open-Teach의 `RobotWrapper` ABC를 구현. rbpodo로 직접 TCP 통신 (ROS 불필요).

#### 좌표계 변환 함수
```python
def _tcp_to_homo(tcp):
    """[X(mm), Y(mm), Z(mm), Rx, Ry, Rz(deg)] → 4×4 동차 변환 행렬 (m)"""
    t = tcp[:3] / 1000.0
    R = Rotation.from_euler('xyz', tcp[3:], degrees=True).as_matrix()
    H = np.eye(4); H[:3, :3] = R; H[:3, 3] = t
    return H

def _cart_to_tcp(cartesian_coords):
    """[x(m), y(m), z(m), qx, qy, qz, qw] → [X(mm), Y(mm), Z(mm), Rx, Ry, Rz(deg)]"""
    t_mm = cartesian_coords[:3] * 1000.0
    euler_deg = Rotation.from_quat(cartesian_coords[3:]).as_euler('xyz', degrees=True)
    return np.concatenate([t_mm, euler_deg])
```

#### 핵심 메서드
- `get_pose()`: `get_tcp_info()` → 4×4 동차 변환 행렬 반환
- `arm_control(cartesian)`: 쿼터니언+미터 → 오일러+mm 변환 후 `move_servo_l()` 호출
- `get_joint_position()`: `get_system_variable(SD_J0_ANG~SD_J5_ANG)` 사용

#### 중요 발견사항
- `get_system_variable` API: out 파라미터 아닌 **튜플 반환** 방식
  ```python
  _, val = robot.get_system_variable(rc, rb.SystemVariable.SD_J0_ANG)
  ```
- `move_servo_l` 파라미터: `(rc, target[6], t1=0.02, t2=0.05, gain=1.0, alpha=1.0)`
- `disable_waiting_ack` 필요: servo 루프 전에 호출, 조회 명령 전에 `enable_waiting_ack`

#### 특이점(Singularity) 문제
홈 자세 `[0,0,0,0,0,0]`은 팔이 완전히 수직 신전 = IK 특이점:
```
에러: "armstratch"  (move_l에서)
에러: "Un-solvable Point"  (move_jl에서)
```
→ 해결: 초기화 시 작업 자세로 자동 이동
```python
def _move_to_working_pose(self):
    working_pose = np.array([350.51, 3.99, 80.47, -85.71, 90.44, -0.01])
    self._robot.move_j(self._rc, working_pose, 30, 60)
```

---

## 5. RBArmOperator 구현

### 파일: `openteach/components/operators/rb_arm.py`

FrankaArmOperator 로직을 RB 로봇용으로 재구현. `wrist_only` 모드 추가.

#### 파라미터
```python
def __init__(
    self,
    host, transformed_keypoints_port, robot_ip,
    use_filter=True,          # 지수 이동평균 필터
    arm_resolution_port=None,
    teleoperation_reset_port=None,
    dead_zone_mm=5.0,         # 최소 이동 임계값
    latency_frames=0,         # 명령 지연 프레임 수
    wrist_only=True,          # True: 손목 위치만 추적, 회전 무시
    control_hz=20,            # 오퍼레이터 루프 Hz
    move_scale=1.0,           # 손 이동량 → 로봇 이동량 비율
    tap_to_move=False,        # True: Middle Pinch 한 번에 한 번만 이동
):
```

#### wrist_only 모드 핵심 로직
```python
if self.wrist_only:
    wrist_init = self.hand_init_H[:3, 3]        # 초기 손목 (Unity World Space, m)
    wrist_now  = self.hand_moving_H[:3, 3]      # 현재 손목 (Unity World Space, m)
    delta_hand = wrist_now - wrist_init          # 이동량 (m)
    delta_clamped = np.clip(delta_hand, -0.1, 0.1)  # 최대 100mm 클램프
    robot_init_pos = self.robot_init_H[:3, 3]
    target_pos = robot_init_pos + delta_clamped * effective_scale
    final_pose = np.concatenate([target_pos, self._robot_init_orientation])
```

#### 좌표계 문제 (미해결)
```
Unity World Space (left-handed, Y-up, m)
        ↓  delta 계산
손목 이동량 (Unity 좌표계)
        ↓  그대로 더함 ← 좌표계 변환 없음!
Robot Base Frame (right-handed)
```
- Unity Y(위) → Robot Y로 매핑 (물리적으로 맞지 않음, Robot Z가 위)
- `R_unity2robot` 회전 행렬 필요하나 아직 미확정

#### tap_to_move 모드
```
Index Pinch (초록) → 손 이동
Middle Pinch (파란) → 그 순간 손목 위치로 move_l 실행 (1회)
이동 완료 후 → 기준점 갱신, 다음 tap 대기
```

핵심 수정: STOP→CONT 전환 시 `_reset_teleop` 호출 안 함 (delta=0 버그 방지)
```python
elif self.tap_to_move and arm_teleop_state == STOP and new_state == CONT:
    moving_hand_frame = self._get_hand_frame()  # reset 없이 현재 프레임만
```

---

## 6. 설정 파일

### `configs/robot/rb_arm.yaml`
```yaml
robot_name: rb_arm

detector:
  _target_: openteach.components.detector.oculus.OculusVRHandDetector
  host: ${host_address}
  oculus_port: ${oculus_reciever_port}
  keypoint_pub_port: ${keypoint_port}
  button_port: ${resolution_button_port}
  button_publish_port: ${resolution_button_publish_port}
  teleop_reset_port: ${teleop_reset_port}
  teleop_reset_publish_port: ${teleop_reset_publish_port}

transforms:
  - _target_: openteach.components.detector.keypoint_transform.TransformHandPositionCoords
    host: ${host_address}
    keypoint_port: ${keypoint_port}
    transformation_port: ${transformed_position_keypoint_port}
    moving_average_limit: 1

visualizers:
  - _target_: openteach.components.visualizers.teleop_info_visualizer.TeleopInfoVisualizer
    host: ${host_address}
    transformed_keypoint_port: ${transformed_position_keypoint_port}
    oculus_feedback_port: ${oculus_graph_port}
    robot_ip: "10.0.2.7"

operators:
  - _target_: openteach.components.operators.rb_arm.RBArmOperator
    host: ${host_address}
    transformed_keypoints_port: ${transformed_position_keypoint_port}
    robot_ip: "10.0.2.7"
    arm_resolution_port: ${resolution_button_publish_port}
    use_filter: True
    teleoperation_reset_port: ${teleop_reset_publish_port}
    dead_zone_mm: 5.0
    latency_frames: 0
    wrist_only: True
    control_hz: 20
    move_scale: 1.0
    tap_to_move: True

controllers:
  - _target_: openteach.robot.rb_arm.RBArm
    robot_ip: "10.0.2.7"
    speed_bar: 0.05

recorded_data:
  - - joint_states
    - cartesian_states
```

### `configs/network.yaml` (수정)
```yaml
host_address: '192.168.50.49'  # PC WiFi 동글 IP (Quest와 동일 네트워크)
```

---

## 7. 수정된 기존 파일

### `openteach/components/sensors/__init__.py`
pyrealsense2 등 선택적 의존성 미설치 시 ImportError 방지:
```python
try:
    from .realsense import RealsenseCamera
except ImportError:
    pass
try:
    from .fish_eye_cam import FishEyeCamera
except ImportError:
    pass
```

---

## 8. 새로 생성된 스크립트

### `mock_quest.py`
Meta Quest 없이 ZMQ keypoint 스트리밍 시뮬레이션.
```bash
python mock_quest.py              # 정적 손 자세
python mock_quest.py --move --host 192.168.50.49  # 손목 X축 진동
```

### `debug_pipeline.py`
전체 파이프라인을 단일 프로세스(threading)로 실행. 모든 출력 가시.
- Detector → Transform → Operator → RBArm 을 스레드로 묶어 실행
- teleop.py의 multiprocessing과 달리 stdout이 보임

### `monitor_tcp.py`
텔레오퍼레이션 중 로봇 TCP 실시간 모니터링 (별도 터미널).
```
X(mm)    Y(mm)    Z(mm)  | Rx(deg) Ry(deg) Rz(deg) | 원점거리(mm)
```

### `monitor_teleop.py`
손목 이동량 + 로봇 TCP를 동시에 실시간 표시.
```
손목 dX    손목 dY    손목 dZ  [주축]  TCP X  TCP Y  TCP Z  원점거리
```

---

## 9. TeleopInfoVisualizer

### 파일: `openteach/components/visualizers/teleop_info_visualizer.py`

matplotlib으로 대시보드 이미지를 생성해 ZMQ(포트 15001)로 Quest 화면에 전송.
기존 `GraphStream.cs`가 이 포트를 구독해서 화면에 표시.

#### Quest 화면 표시 내용
```
┌──────────────────────────────┐
│      RB Teleop Monitor       │
│  대기중 (Index→이동→Middle)  │ ← tap 상태 (색상 변화)
├──────────────────────────────┤
│ 손목 이동 (cm)                │
│ X: +5.2  Y: +0.1  Z: -0.3   │
│ X ████░░░  Y █░░  Z █░░      │ ← 방향 바
├──────────────────────────────┤
│ 로봇 TCP (mm)                 │
│ X: -282  Y: -111  Z: +769    │
│    원점 거리: 821.3 mm        │
└──────────────────────────────┘
```

#### 상태 색상
- 회색: 대기중 (WAITING)
- 주황: 이동중 (MOVING)
- 초록: 완료 (DONE)
- 빨강: 실패 (FAIL)

#### 주의: 한글 폰트 경고
```
UserWarning: Glyph *** missing from font(s) DejaVu Sans.
```
→ 동작에는 영향 없음. 한글 대신 영문으로 변경하거나 폰트 설치로 해결 가능.

---

## 10. 발생한 주요 문제와 해결

### 10.1 pyrealsense2 ImportError
- **원인**: `sensors/__init__.py`가 무조건 import
- **해결**: try/except로 감싸기

### 10.2 `return_real()` 미정의
- **원인**: Operator ABC에 있지만 FrankaArmOperator에는 없음
- **해결**: `RBArmOperator`에 추가
  ```python
  def return_real(self): return True
  ```

### 10.3 `get_system_variable` API 변경
- **구 API**: out 파라미터 `val = np.zeros(1); robot.get_system_variable(rc, sv, val)`
- **현 API**: 튜플 반환 `_, val = robot.get_system_variable(rc, sv)`

### 10.4 홈 자세 IK 특이점
- **원인**: `[0,0,0,0,0,0]` = 완전 신전 자세 = Cartesian 이동 불가
- **에러**: `"armstratch"`, `"Un-solvable Point"`
- **해결**: 초기화 시 작업 자세 `[350.51, 3.99, 80.47, -85.71, 90.44, -0.01]`로 이동

### 10.5 move_servo_l 작동 안 함 (초기)
- **원인**: 홈 자세 특이점
- **확인**: 작업 자세 이동 후 테스트하면 정상 동작

### 10.6 ZMQ 포트 충돌
- **원인**: 이전 teleop 프로세스가 포트 점유 후 좀비 상태
- **해결**:
  ```bash
  pkill -9 -f "teleop\|mock_quest\|debug_pipeline"
  for port in 8087 8088 8089 8093 8095 8100 8102 15001; do
    lsof -ti tcp:$port | xargs -r kill -9
  done
  ```

### 10.7 multiprocessing 로그 미출력
- **원인**: 자식 프로세스 stdout이 부모로 전달 안 됨
- **해결**: 파일 기록 방식 사용 (`/tmp/tap_status.txt`, `/tmp/teleop_axis_log.csv`)

### 10.8 matplotlib `axhline` transform 오류
- **오류**: `'transform' is not allowed as a keyword argument`
- **해결**: `transform=ax.transAxes` 인자 제거

### 10.9 tap_to_move delta=0 버그
- **원인**: STOP→CONT 전환 시 `_reset_teleop()` 호출 → `hand_init = 현재위치` → delta=0
- **해결**: tap_to_move 모드에서는 STOP→CONT 시 reset 없이 현재 프레임만 가져옴

### 10.10 move_scale=1.0 + 큰 Unity 좌표 = 범위 초과
- **원인**: Unity World Space 좌표가 실제 방 크기 (~수 m), 손이 조금만 이동해도 로봇이 수천 mm 목표
- **해결**: delta 클램프 추가 (max 100mm per tap)
  ```python
  delta_clamped = np.clip(delta_hand, -0.1, 0.1)
  ```

### 10.11 Quest APK 메뉴 사라짐
- **원인**: PlayerPrefs에 이전 IP 저장 → 자동 연결 → 메뉴 숨김
- **진단**: ZMQ PUSH Connect는 서버 없어도 성공 → `connectionEstablished=true`
- **해결**: ADB로 앱 데이터 초기화
  ```bash
  sudo apt install adb
  echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2833", MODE="0666", GROUP="plugdev"' | \
    sudo tee /etc/udev/rules.d/51-oculus.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger
  adb shell pm clear com.NYUGRAIL.KinovaBot
  ```

### 10.12 랜선 IP 설정 (네트워크 변경 후)
기존 두 랜선 → WiFi 동글 + 한 랜선 변경:
```bash
sudo ip addr add 10.0.2.100/24 dev enp14s0
```
영구 설정은 NetworkManager로:
```bash
sudo nmcli connection add type ethernet ifname enp14s0 con-name robot-lan ip4 10.0.2.100/24
sudo nmcli connection up robot-lan
```

---

## 11. 축 매핑 분석

### 로봇 축 테스트 결과
`move_l`로 각 방향 10mm 이동 테스트:
```
로봇 +X → 성공  ΔX=+10.0mm
로봇 -X → 성공  ΔX=-10.0mm
로봇 +Y → 성공  ΔY=+10.0mm
로봇 -Y → 성공  ΔY=-10.0mm
로봇 +Z → 성공  ΔZ=+10.0mm
로봇 -Z → 성공  ΔZ=-10.0mm
```
→ 로봇 베이스 좌표계 X/Y/Z 모두 독립적으로 동작 확인.

### Unity → Robot 축 매핑 (세션 로그 분석)
`/tmp/teleop_axis_log.csv` 데이터 분석:
```
hand_dX ↑ → robot_tX ↑  (동일 방향)
hand_dY ↑ → robot_tY ↑  (동일 방향)
hand_dZ ↓ → robot_tZ ↓  (동일 방향)
```
현재는 1:1 직접 매핑이나 물리적으로 맞지 않음:
- Unity Y = 실세계 위쪽 → Robot Y로 가지만 → **Robot Z가 위쪽이어야 직관적**
- `R_unity2robot` 캘리브레이션 행렬 미완성 상태

### rbpodo TCP 좌표계
- `get_tcp_info()` 반환: `[X, Y, Z, Rx, Ry, Rz]` (mm, deg)
- `move_l()`, `move_servo_l()`: **로봇 베이스 좌표계** 기준
- Euler 컨벤션: `Rotation.from_euler('xyz', ...)` 사용 (XYZ 순서)

---

## 12. 성능 및 주파수 설정

### 주파수 설정
| 항목 | 값 | 비고 |
|---|---|---|
| VR_FREQ | 60Hz | Quest 키포인트 전송 |
| control_hz | 20Hz | 오퍼레이터 루프 (yaml 설정) |
| move_servo_l t1 | 0.02s | look-ahead (≥1/60 = 0.017s) |
| move_servo_l t2 | 0.05s | 스무딩 |
| C++ 예제 servo | 200Hz | rbpodo 권장 주기 |

### 성능 최적화
- dead_zone에서 `get_pose()` 네트워크 호출 제거 → `_last_sent_pos` 캐싱으로 대체
- Visualizer 로봇 연결 지연 초기화 (3초 후) → Operator 소켓 간섭 방지

---

## 13. 실행 가이드

### 전체 파이프라인 실행
```bash
# 터미널 1: 서버
cd ~/Open-Teach && conda activate rbpodo
python teleop.py robot=rb_arm

# 터미널 2: TCP 모니터 (선택)
python monitor_tcp.py
# 또는
python monitor_teleop.py
```

### Quest 없이 테스트 (mock)
```bash
# 터미널 1: 서버
python teleop.py robot=rb_arm

# 터미널 2: 가상 손 스트리밍
python mock_quest.py --move --host 192.168.50.49
```

### Quest 연결 시
1. SingleArmBot APK 실행 (패키지명: `com.NYUGRAIL.KinovaBot`)
2. Menu → Change IP → `192.168.50.49`
3. Stream 클릭 → 테두리 초록색 확인
4. **Index Pinch** → 손 이동 → **Middle Pinch** (tap_to_move 모드)

### 서버 재시작 시
- 자동으로 작업 자세 `[350.51, 3.99, 80.47, -85.71, 90.44, -0.01]`로 이동
- Quest 앱은 IP 재입력 없이 Stream 버튼만 누르면 재연결

---

## 14. 파일 구조 (추가/변경된 파일)

```
Open-Teach/
├── configs/
│   ├── network.yaml              ← host_address 수정
│   └── robot/
│       └── rb_arm.yaml           ← 새로 생성
├── openteach/
│   ├── components/
│   │   ├── sensors/
│   │   │   └── __init__.py       ← try/except 추가
│   │   ├── operators/
│   │   │   └── rb_arm.py         ← 새로 생성
│   │   └── visualizers/
│   │       └── teleop_info_visualizer.py  ← 새로 생성
│   └── robot/
│       └── rb_arm.py             ← 새로 생성
├── mock_quest.py                 ← 새로 생성
├── debug_pipeline.py             ← 새로 생성
├── monitor_tcp.py                ← 새로 생성
├── monitor_teleop.py             ← 새로 생성
└── rb_teleop.code-workspace      ← 새로 생성

rbpodo/
├── examples/
│   └── basic_check.py            ← 새로 생성
└── RB_GUIDE.md                   ← 새로 생성
```

---

## 15. 미완성 / 다음 단계

### 미완성 항목
1. **R_unity2robot 축 매핑**: Unity 좌표계 → Robot Base 좌표계 변환 행렬 미확정
   - 물리적으로 손을 위로 올리면 로봇이 Y 방향으로 가지만 Z 방향이 되어야 함
   - 캘리브레이션 세션 필요 (각 방향으로 tap하여 로그 수집 후 행렬 계산)

2. **Workspace 한계 설정**: 로봇 작업 공간 범위(mm) 클램프 미설정
   ```yaml
   # rb_arm.yaml controllers에 추가 예정
   # workspace_limits:
   #   x: [-400, 400]
   #   y: [-600, 100]
   #   z: [200, 1000]
   ```

3. **tap_to_move 속도 조정**: move_l speed 파라미터 최적화
   - 현재 `move_l(rc, target, 150, 300)` 사용 중

4. **한글 폰트 경고 해결**: DejaVu Sans 폰트에 한글 없음 → 영문 전환 또는 나눔고딕 설치

5. **Closed-loop IK 피드백**: 현재 open-loop → 실제 TCP 피드백 받아 오차 보정 미구현

### 참고 프레임워크
- **Open-TeleVision**: wrist pose + SE(3) filtering + closed-loop IK 구현체
- 추후 참고하여 안정성 개선 예정

---

## 16. 커밋 히스토리 (Open-Teach fork)

```
9e3a2dc docs: 브랜치 전략 및 커밋 컨벤션 추가
85e8434 docs: RB 텔레오퍼레이션 세팅 가이드 및 워크스페이스 파일 추가
3404da9 chore: 개발 및 테스트 유틸리티 추가
691b00a chore: RB 로봇 텔레오퍼레이션 설정 파일 추가
623a520 feat: RB 로봇 텔레오퍼레이션 오퍼레이터 구현
5f28df2 feat: Rainbow Robotics RB 시리즈 로봇 래퍼 구현
8972b05 fix: 선택적 의존성 없을 때 sensors import 오류 수정
```

## 17. 커밋 히스토리 (rbpodo)

```
a92ee23 docs: 브랜치 전략 및 커밋 컨벤션 추가
0d17a13 docs: RB 텔레오퍼레이션 워크스페이스 세팅 안내 추가
b610216 docs: rbpodo 한국어 사용 가이드 추가
3a8d3d1 chore: 연결 및 기본 동작 확인 예제 추가
```