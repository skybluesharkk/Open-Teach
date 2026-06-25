# RB 로봇 텔레오퍼레이션 세팅 가이드

> 이 포크는 **Meta Quest 3**로 **Rainbow Robotics RB 시리즈** 로봇 팔을 텔레오퍼레이션하기 위한 구현입니다.
> Open-Teach 파이프라인에 rbpodo 기반 RB 로봇 래퍼를 추가했습니다.

## 브랜치 전략

- 각자 **본인 이름으로 브랜치**를 생성해서 작업하세요: `git checkout -b <이름>`
- 기능 완성 후 `main`으로 PR을 올려 리뷰 후 merge합니다
- 작업 전 항상 `git pull origin main`으로 최신 상태를 유지하세요

## 커밋 컨벤션

```
feat:  새로운 기능 추가
fix:   버그 수정
chore: 설정, 의존성 등 기타 변경
docs:  문서 수정
style: 포맷 수정 (기능 변경 없음)
refactor: 코드 구조 개선
test:  테스트 추가 및 수정
```

## 워크스페이스 구조

두 레포를 **같은 디렉토리 아래** 클론해야 합니다.

```bash
mkdir rb_teleop && cd rb_teleop
git clone https://github.com/<your-account>/Open-Teach.git
git clone https://github.com/<your-account>/rbpodo.git
```

VS Code에서 `Open-Teach/rb_teleop.code-workspace`를 열면 두 레포가 하나의 워크스페이스로 구성됩니다.

## 환경 세팅

```bash
# conda 환경 생성
conda create -n rbpodo python=3.11
conda activate rbpodo

# rbpodo 설치 (소스 빌드)
pip install -e rbpodo/

# Open-Teach 의존성 설치
pip install pyzmq hydra-core omegaconf scipy matplotlib h5py opencv-python \
            pandas pillow Flask gevent gunicorn tqdm ikpy shapely IPython blosc
pip install -e Open-Teach/
```

## 설정

**1. 네트워크 설정** — `Open-Teach/configs/network.yaml`

```yaml
host_address: '<PC IP>'   # ip addr show 로 확인 (Quest와 같은 WiFi 인터페이스)
```

**2. 로봇 IP 설정** — `Open-Teach/configs/robot/rb_arm.yaml`

```yaml
robot_ip: "<로봇 컨트롤박스 IP>"   # 기본값: 10.0.2.7 (랜선 직접 연결 시)
```

**3. 작업 자세 설정** — `Open-Teach/openteach/robot/rb_arm.py`

```python
def _move_to_working_pose(self):
    working_pose = np.array([J0, J1, J2, J3, J4, J5])  # 로봇에 맞는 관절 각도(deg)
```
홈 자세 `[0,0,0,0,0,0]`은 IK 특이점이므로 텔레오퍼레이션 가능한 작업 자세로 변경 필요합니다.

## 실행

```bash
# Meta Quest에 VR/APK/SingleArmBot.apk 설치 후
# Quest WiFi를 PC와 같은 공유기에 연결

cd Open-Teach
conda activate rbpodo
python teleop.py robot=rb_arm
```

서버 시작 후 Quest에서 앱 실행 → Menu → Change IP → Stream.
왼손 **Middle Pinch**(파란 테두리)로 Arm 모드 활성화.

## Quest 없이 테스트

```bash
# 터미널 1: 서버
python teleop.py robot=rb_arm

# 터미널 2: Mock Quest (가상 손 움직임 전송)
python mock_quest.py --move --host <PC IP>
```

---

# OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation

##### Authors: Aadhithya Iyer ,Zhuoran Peng, Yinlong Dai, Irmak Guzey, Siddhant Haldar, Soumith Chintala, Lerrel Pinto 

[Paper](https://arxiv.org/abs/2403.07870) [Website](https://open-teach.github.io/)

This is the official implementation of the Open Teach including unity scripts for the VR application, teleoperation pipeline and demonstration collection pipeline.

Open Teach consists of two parts. 

- [x] Teleoperation using Meta Quest 3 and data collection over a range of robot morphologies and simulation environments.

- [x] Policy training for various dexterous manipulation tasks across different robots and simulations.

### VR Code and User Interface

Read VR specific information, User Interface and APK files [here](/docs/vr.md)

### Server Code Installation 

Install the conda environment from the yaml file in the codebase

**Allegro Sim**

`conda env create -f env_isaac.yml`

**Others**

`conda env create -f environment.yml`

This will install all the dependencies required for the server code.  

After installing all the prerequisites, you can install this pipeline as a package with pip:

`pip install -e . `

You can test if it had installed correctly by running ` import openteach` from the python shell.

### Robot Controller Installation Specific Information

1. For Simulation specific information, follow the instructions [here](/docs/simulation.md).

2. For Robot controller installation, follow the instructions [here](https://github.com/NYU-robot-learning/OpenTeach-Controllers)

### For starting the camera sensors

For starting the camera sensors and streaming them inside the screen in the oculus refer [here](/docs/sensors.md)

### Running the Teleoperation and Data Collection

For information on running the teleoperation and data collection refer [here](/docs/teleop_data_collect.md).


### Policy Learning 

For open-source code of the policies we trained on the robots refer [here](/docs/policy_learning.md) 

### Policy Learning API

For using the API we use for policy learning, use [this](https://github.com/NYU-robot-learning/Open-Teach-API)

### Call for contributions

For adding your own robot and simulation refer [here](/docs/add_your_own_robot.md)

### Citation
If you use this repo in your research, please consider citing the paper as follows:
```
@misc{iyer2024open,
      title={OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation}, 
      author={Aadhithya Iyer and Zhuoran Peng and Yinlong Dai and Irmak Guzey and Siddhant Haldar and Soumith Chintala and Lerrel Pinto},
      year={2024},
      eprint={2403.07870},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}



