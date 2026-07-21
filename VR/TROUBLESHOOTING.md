# Quest 앱(Franka-Bot-Unity) 트러블슈팅 기록

택타일 시각화(TactileOverlay) 개발 중 발견·해결한 문제들의 기록.
증상만 보면 원인을 오판하기 쉬웠던 사례가 많아, 진단 과정과 함께 남긴다.

진단 도구: `adb logcat` (전체), `adb logcat -s Unity` (앱 로그만)

---

## 1. 원본 Open-Teach 코드의 매 프레임 예외 3종 (d403e3d)

**증상**: 앱 버벅임/간헐 멈춤. logcat에 초당 200+ 예외 로그 폭주.

**원인** (셋 다 72Hz로 매 프레임 발생):

| 위치 | 예외 | 이유 |
|---|---|---|
| `GestureDetector.SendHandData` | `NotSupportedException: PushSocket doesn't support receiving` | PushSocket(송신 전용)에 `ReceiveFrameBytes()` 호출. 원래부터 100% 실패하는 죽은 코드 — 전송은 예외 전에 끝나서 텔레옵이 "우연히" 동작해 왔음 |
| `GraphStream.Update` | `ArgumentOutOfRangeException` | 포트 15001에 퍼블리셔가 없으면 빈 리스트에 `[Count-1]` 인덱싱 |
| `CameraOneStreamer.Update` | `ArgumentOutOfRangeException` | 포트 10505 동일 |

**왜 이제야 드러났나**: 예전엔 servo_teleop.py가 15001/10505에 이미지를 계속
보내서 리스트가 비지 않았음. 택타일 퍼블리셔(15002)만 켜고 테스트하면서
두 포트가 비자 매 프레임 터지기 시작.

**수정**: 죽은 수신 호출 제거, 빈 리스트 가드 추가, 매 프레임 Debug.Log 스팸 제거.
**기존 텔레옵 동작에는 영향 없음** (전송 로직 무변경, 이미지 오면 동일하게 표시).

## 2. 수신 스레드 영구 사망 → 시각화 박제 (4427689)

**증상**: 몇 초 뒤 택타일 시각화가 마지막 상태로 얼어붙음. 앱 엔진은 정상
(VrApi 로그 72/72 FPS, App 0.5ms — "앱이 멈췄다"고 느끼지만 실제론 데이터만 멈춤).

**원인**: `ReceiveLoop`의 `catch (Exception) { break; }` — 일시적 예외 한 번에
스레드가 **조용히, 로그 없이, 영구히** 종료. `latestPacket`이 마지막 값으로 박제.

**수정**:
- `TryReceiveFrameString(200ms)` 타임아웃 기반으로 변경, 일시 예외는 재시도
- 수신 하트비트: 1초+ 새 패킷 없으면 박제 대신 시각화 숨김
- 트래킹/수신 상태 전환 로그 추가 (`[Tactile] ...`) — `adb logcat -s Unity`로 확인

## 3. 오브젝트 생성 스파이크 (51e38c6)

**증상**: F2B(벡터장) 첫 진입 순간 수백 ms 멈춤.

**원인**: 오브젝트 120개(프리미티브 240개)를 한 프레임에 몰아 생성 +
`Shader.Find("Standard")`를 매 머티리얼마다 호출(유니티 대표 성능 함정).

**수정**: Shader.Find 1회 캐싱, 오브젝트를 프레임당 `buildPerFrame`(10)개씩
점진 생성. F1은 이후 히트 셸 방식(손끝당 1메시)으로 재설계되어 해당 없음.

## 4. 미착용 감지 자동 절전 (코드 아님)

**증상**: 헤드셋을 손에 들고 테스트하면 몇 초 뒤 앱이 pause되며 멈춘 것처럼 보임.

**원인**: logcat에 `VrPowerManagerService: HEADSET_UNMOUNTED → WAITING_FOR_SLEEP`.
근접 센서가 얼굴을 못 느끼면 Quest가 절전 진입.

**주의: 이 설정은 헤드셋 재부팅 시 풀린다** — "어제 껐는데 또 멈춘다"의 원인.
테스트 세션 시작 때마다 재적용할 것. 헤드셋을 살짝 들거나 이마에 걸치기만 해도
근접 센서가 미착용으로 판단할 수 있음.

**해결** (개발 중, 재부팅 전까지 유효):
```bash
adb shell am broadcast -a com.oculus.vrpowermanager.prox_close      # 절전 끔
adb shell am broadcast -a com.oculus.vrpowermanager.automation_disable  # 원복
```

## 5. 리눅스에서 Unity 에디터 행 (환경 문제)

**증상**: Unity 2021.3 에디터가 스크립트 컴파일(bee_backend)에서 무한 멈춤.
CPU 0%, 로그 정지. 캐시 삭제해도 동일 지점 재발.

**원인**: Unity 2021 세대 bee_backend와 최신 리눅스 커널(6.x)의 호환성 문제.

**해결**: 윈도우에서 빌드 (상세 절차: [WINDOWS_BUILD.md](WINDOWS_BUILD.md)).

## 6. 기타 운영 메모

- **PC IP 변동**: DHCP 재할당으로 IP가 바뀌면 (49→50 등) 퍼블리셔 bind 실패
  (`Cannot assign requested address`) + Quest 앱의 저장된 IP 불일치.
  공유기에서 MAC 고정(DHCP 예약) 권장. WiFi 연결명: `ASUS_38`.
- **adb unauthorized**: 헤드셋 착용 상태에서 USB 디버깅 허용 팝업 승인 필요.
  안 뜨면 `rm ~/.android/adbkey*` 후 `adb kill-server && adb start-server`.
- **모드 전환은 빌드 불필요**: 모드는 PC 패킷 prefix(F1:/F2A:/F2B:)가 결정.
  `tactile_viz_dummy.py --cycle 10` 으로 10초 자동 순환 (0=수동).
- **APK 서명 충돌**: 원저자 APK 위에 자체 빌드 설치 시
  `INSTALL_FAILED_UPDATE_INCOMPATIBLE` → `adb uninstall <패키지>` 후 설치.
