# 윈도우 이관 핸드오프 — XHand 택타일 시각화 개발

리눅스에서 진행하던 Quest 택타일 시각화 개발을 윈도우로 옮겨 이어가기 위한 문서.
새 Claude 세션을 시작하면 이 문서를 먼저 읽게 할 것.

## 1. 프로젝트 컨텍스트 (한 단락 요약)

Meta Quest 3 핸드트래킹 위에 XHand 로봇핸드의 택타일 센서값을 AR로 시각화한다.
PC의 더미 퍼블리셔(`tactile_viz_dummy.py`)가 ZMQ(포트 15002)로 패킷을 쏘고,
Quest 앱(Franka-Bot-Unity의 `TactileOverlay.cs`)이 받아서 세 가지 모드로 렌더:
- **F1**: 손 메시 자체가 물드는 히트맵 (손 머티리얼을 `TactileHandHeat.shader`로 교체,
  손끝 5개 히트포인트 거리 기반 jet 컬러맵)
- **F2A**: 손끝당 힘 벡터 화살표 1개 (손가락 등쪽 법선 방향)
- **F2B**: 손끝당 5×8 벡터장 — 곡면에 감긴 관통 대응점에서 평행 다발로 솟음
모드는 패킷 prefix(F1:/F2A:/F2B:)가 결정 — **모드 전환·데이터 변경은 재빌드 불필요**.
렌더 방식/좌표 변경은 C#/셰이더 수정이라 **재빌드 필요**.

로봇 텔레옵(servo_teleop.py, RB5-850)은 리눅스 PC 전용(유선 10.0.2.x) — 이관 대상 아님.

## 2. 현재 상태 (2026-07-21 기준)

마지막 커밋: `f26b488` (엄지 롤 +55° 부호 반전)

**직전 빌드에서 확인된 것:**
- 14초 후 앱 멈춤 → 해결됨 (PUSH 송신 큐 블로킹, `bc4c071`)
- F1 손 메시 변색 동작함 (렌더러 선택 수정 후)
- F2B 화살표·곡면 래핑·평행 다발 동작함

**이번 빌드(f26b488 포함)에서 검증할 것:**
1. F2B: 손바닥을 보여주며 시작해도 화살표가 반전되지 않는지 (`2ae03d7`)
2. F1: 색이 손바닥 전체가 아니라 손끝 부위에만 물드는지 (반경 16→11mm)
3. F1: 잠깐 되다 끊겨도 자동 복구되는지 (`[Tactile] 손 머티리얼이 외부에서 교체됨` 로그 확인)
4. **엄지 롤 방향** (+55°) — 아래 직렬화 함정 주의

## 3. ⚠ 엄지 롤(thumbRollDeg) 직렬화 함정

`thumbRollDeg` 코드 기본값을 -55 → **+55**로 바꿨는데, 윈도우에서 이전에
**씬을 저장한 적이 있으면 -55가 씬에 박제**되어 코드 기본값이 무시된다.

재빌드 후에도 엄지 방향이 반대면:
1. Unity에서 SampleScene 열기 → TactileOverlay 붙은 오브젝트 선택
2. Inspector → **Thumb Roll Deg** 값이 -55면 **55로 직접 수정**
3. 씬 저장 후 빌드

각도 크기(55°)도 실물 보고 미세 조정 대상. 관련 파라미터:
- `thumbRollDeg`: 엄지 등쪽 롤 보정 (부호 = 회전 방향)
- `flipHandNormal`: 모든 손가락 법선이 통째로 반대면 체크
- `handHeatRadius`(0.011), `fingerRadius`(0.008): F1 확산/곡면 반경

## 4. 윈도우 환경 세팅

### 필수 도구
1. **git** — 레포 clone (이미 있음)
2. **Unity 2021.3.x + Android Build Support** (이미 있음)
3. **adb** — [Android platform-tools](https://developer.android.com/tools/releases/platform-tools)
   다운로드 → 압축 풀고 PATH 추가. 명령어는 리눅스와 100% 동일
4. **Python 3.9+** — 퍼블리셔용. rbpodo 환경 전체는 불필요, 이것만:
   ```
   pip install pyzmq numpy
   ```

### 네트워크 (듀얼부팅 — 같은 PC라 IP 통일 가능)

리눅스는 WiFi(ASUS_38)에 **192.168.50.49 수동 고정**이다. 윈도우는 기본 DHCP라
부팅 시 다른 IP를 받을 수 있음 → **윈도우도 같은 49로 수동 고정하면**
OS를 오가도 Quest 앱 IP 재입력이 영영 불필요해진다 (한 번에 한 OS만 부팅하므로 충돌 없음).

1. 먼저 `ipconfig` 확인 — 이미 192.168.50.49면 아무것도 안 해도 됨
2. 아니면: 설정 → 네트워크 및 인터넷 → Wi-Fi → ASUS_38 속성 → **IP 할당: 수동** → IPv4 켬
   - IP: `192.168.50.49` / 서브넷 접두사 길이: `24`
   - 게이트웨이·DNS: `ipconfig`의 "기본 게이트웨이" 값 (보통 `192.168.50.1`)
3. 이후 퍼블리셔는 항상 `python tactile_viz_dummy.py --host 192.168.50.49`
4. 방화벽이 물으면 Python 허용 (또는 15002/tcp 인바운드 허용)

### 테스트 세션 시작 루틴
```
adb devices                                                  # device 확인
adb shell am broadcast -a com.oculus.vrpowermanager.prox_close   # 절전 끔 (재부팅마다!)
python tactile_viz_dummy.py --host <윈도우IP>                # 10초 자동 모드 순환
adb logcat -s Unity                                          # [Tactile] 로그 관찰
```

### 빌드·설치
- Unity Build → APK → `adb install -r xxx.apk`
- 상세: [VR/WINDOWS_BUILD.md](VR/WINDOWS_BUILD.md)

## 5. 핵심 파일 지도

| 파일 | 역할 |
|---|---|
| `tactile_viz_dummy.py` | 더미 퍼블리셔 (모드 순환 `--cycle`, 강도 패턴, 그리드 크기) |
| `VR/.../Scripts/TactileOverlay.cs` | Quest 시각화 전부 (수신·파싱·3모드 렌더) |
| `VR/.../Resources/TactileHandHeat.shader` | F1 손 메시 히트 셰이더 |
| `VR/.../Resources/TactileHeat.shader` | F1 셸 폴백용 정점색 셰이더 |
| `tactile_viz_preview.html` | 브라우저 미리보기 (디자인 논의용) |
| `VR/TROUBLESHOOTING.md` | **멈춤/버그 진단 기록 — 문제 생기면 여기부터** |
| `VR/WINDOWS_BUILD.md` | 빌드 절차 |

## 6. 자주 쓰는 진단 명령

```
adb logcat -s Unity | findstr Tactile        # 상태 전환/오류 (윈도우는 findstr)
adb logcat -d -b crash                       # 크래시 버퍼
adb shell pidof com.Xigbee.FrankaBot         # 앱 생존 확인
adb uninstall com.Xigbee.FrankaBot           # 서명 충돌 시 제거 후 설치
```

## 7. 리눅스에 남는 것 (로봇 작업 시 복귀)

- servo_teleop.py (RB5 로봇 텔레옵) — 로봇 유선망(10.0.2.x) 필요
- rbpodo 환경, 로봇 IP 10.0.2.7, PC 유선 10.0.2.100
- 리눅스 WiFi를 192.168.50.49 고정으로 쓰던 설정 — 윈도우 작업 중에는
  Quest 앱 IP가 윈도우를 향하므로, 로봇+택타일 동시 작업하게 되면 IP 정리 필요

## 8. 다음 작업 후보 (우선순위 논의된 것)

1. 이번 빌드 4개 항목 검증 (§2)
2. 실제 XHand 센서 연동 — 더미의 `finger_grid_scalar`/`finger_grid`/`finger_vector`만 실측으로 교체
3. teleop + 택타일 동시 구동 통합 테스트 (리눅스)
4. 손 상태 추정기(가려짐 대응 모션 모델), 스프링-댐퍼 프록시, XHand 리타게팅
