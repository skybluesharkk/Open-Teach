# Quest APK 윈도우 빌드 가이드 (TactileOverlay 포함)

리눅스(커널 6.x 최신)에서는 Unity 2021.3의 빌드 백엔드(bee_backend)가
스크립트 컴파일 단계에서 데드락되는 이슈가 있어 에디터가 무한 멈춤.
→ **윈도우에서 빌드한다.**

## 1. 준비 (윈도우 PC)

1. **Unity Hub 설치** — https://unity.com/download
2. Hub → Installs → **Unity 2021.3.x LTS** 설치 (예: 2021.3.45f1)
   - 모듈 선택에서 **Android Build Support** 체크
     (하위 항목 Android SDK & NDK Tools, OpenJDK 포함 전부)
3. 이 레포 clone:
   ```
   git clone <레포 주소>
   ```

## 2. 프로젝트 열기

1. Unity Hub → Projects → **Add** → `Open-Teach/VR/Franka-Bot-Unity` 폴더 선택
2. 에디터 버전 2021.3.x로 열기 — 첫 임포트 5~15분 소요
3. "Project settings were changed..." 안내가 뜨면 **Continue/확인** (같은 2021.3
   계열 마이너 업그레이드 안내라 무해)

## 3. TactileOverlay 씬 연결 (1회)

택타일 시각화 스크립트(`Assets/Scripts/TactileOverlay.cs`)는 이미 레포에
포함돼 있고, 씬에 컴포넌트로 붙이기만 하면 된다.

1. `Assets/Scenes/SampleScene` 열기
2. Hierarchy에서 **GestureDetector가 붙어 있는 오브젝트** 선택
   (검색창에 GestureDetector 입력하면 찾기 쉬움)
3. Inspector → **Add Component** → `TactileOverlay` 검색해 추가
4. TactileOverlay의 `Right Hand Skeleton` 필드에
   GestureDetector의 `Right Hand Skeleton`과 **같은 오브젝트**를 드래그
   - 비워둬도 런타임 자동 탐색이 동작하지만, 명시 할당이 확실함
5. 씬 저장 (Ctrl+S)

## 4. APK 빌드

1. File → Build Settings → Platform에서 **Android** 선택
   - Android가 활성화 안 돼 있으면 **Switch Platform** 클릭 (재임포트 수 분)
2. **Build** 클릭 → 파일명 예: `SingleArmBot_tactile.apk`

## 5. Quest 설치

Quest를 USB 연결 (개발자 모드 필요 — 기존 APK 설치해봤다면 이미 활성).

```
adb install -r SingleArmBot_tactile.apk
```

윈도우에 adb가 없으면 SideQuest로 설치해도 된다.

## 6. 동작 확인 (PC 리눅스 쪽)

```bash
conda activate rbpodo
python tactile_viz_dummy.py --host 192.168.50.49   # PC WiFi IP
```

- Quest에서 앱 실행 → 손 트래킹되면 손끝 위에 시각화 표시
- 터미널에서 `1`/`2`/`3` + 엔터로 모드 전환:
  - **1 = F1** 히트맵 (손끝 구체, 파랑→빨강)
  - **2 = F2A** 손끝당 힘 벡터 화살표 1개
  - **3 = F2B** 손끝당 4×6 벡터장 (deformation field)
- 브라우저 미리보기: `tactile_viz_preview.html` (레포 루트)

## 포트 구성

| 포트 | 용도 |
|---|---|
| 8087 | 핸드 키포인트 (Quest→PC) |
| 8095 / 8100 | 해상도 / 일시정지 신호 |
| 10505 | 카메라 패널 이미지 (PC→Quest) |
| 15001 | 그래프/상태 패널 (PC→Quest) |
| **15002** | **택타일 시각화 데이터 (PC→Quest, 신규)** |

포트는 `Assets/Resources/Configurations/Network.json`의 `tactilePortNum`으로 변경 가능.

## 문제 해결

- **컴파일 에러가 뜬다** → Console 창의 빨간 에러 확인. NetMQ 관련이면
  `Assets/NuGet` 패키지 임포트가 끝날 때까지 대기 후 재시도
- **손끝 화살표 방향이 이상하다** → `TactileOverlay.cs`의 `TipLocalToWorld()`
  함수에서 축 매핑만 조정 (본의 -Y를 손끝 법선으로 근사 중)
- **시각화가 안 뜬다** → PC 방화벽에서 15002 포트 허용 확인,
  Network.json의 IP가 PC WiFi IP와 일치하는지 확인
