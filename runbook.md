# 실행/튜닝/연동 가이드

## ESP32 펌웨어 업로드

`esp32_firmware.ino`를 Arduino IDE로 그대로 열어서 업로드.

1. Arduino IDE에서 `esp32_firmware.ino` 열기
2. 파일 안 `WIFI_SSID`, `WIFI_PASSWORD`, `SERVER_URL` 수정
3. `도구 → 보드` → ESP32 Dev Module, `도구 → 포트` → ESP32 COM 포트 선택
4. 업로드
5. 시리얼 모니터(115200)에서 `연결됨` → `POST -> 200` 확인

## 튜닝 값 위치

| 값 | 파일 | 위치 |
|---|---|---|
| `GAS_ALERT_THRESHOLD` | server.py | server.py:35 |
| `TEMP_TREND_ALERT_PER_MIN` | server.py | server.py:42 |
| `NARROW_DIST_THRESHOLD_CM` | server.py | server.py:37 |
| `dog.sound(0,1,80)` 트랙번호 | server.py | server.py:226 |
| `FLAME_ACTIVE_LOW` | esp32_firmware.ino (server.py 아님) | esp32_firmware.ino:33 |
| `DOG_COM_PORT` | server.py | server.py:28 |
| `CAMERA_SOURCE` | server.py | server.py:31 |

## 실측 방법

- **FLAME_ACTIVE_LOW**: 라이터 대고 시리얼 모니터에서 `flame:1` 나오는지 확인. 계속 0이면 `false`로 바꿔 재업로드.
- **GAS_ALERT_THRESHOLD**: 대시보드 "가스 농도" 평소값 확인 → 가스 뿌려서 오르는 값 확인 → 중간값으로 설정.
- **TEMP_TREND_ALERT_PER_MIN**: 온도센서에 손난로/라이터 대고 대시보드 온도 추세(°C/분) 관찰 → 그보다 살짝 낮게 설정.
- **NARROW_DIST_THRESHOLD_CM**: 실제 우드락 통로 폭 자로 재서 그 값으로.
- **dog.sound 트랙번호**: `tools/deflib.py` 또는 로보독 매뉴얼에서 원하는 트랙 번호 확인 후 교체.

## 연동 구조

```
[ESP32] --Wi-Fi POST--> /sensor_update  ┐
[로보독 동글] --USB(시리얼)-->            │
[PC 웹캠/폰 IP웹캠] --cv2.VideoCapture--> ├─> server.py
                                          │
[브라우저 대시보드] <--/status, /video_feed┘
```

| 연동 | 담당 코드 |
|---|---|
| ESP32 → 서버 | esp32_firmware.ino `http.POST()` → server.py `sensor_update()` (server.py:302) |
| 서버 → 로보독 | server.py `connect_dog()`(150), `_handle_fire_transition()`(203) → `dog.move()`/`dog.sound()`. 실제 시리얼 통신은 tools/robodog.py |
| 카메라 → 서버 | server.py `camera_worker()`(236), `cv2.VideoCapture(CAMERA_SOURCE)` |
| 서버 → 대시보드 | server.py `/`(347) → templates/dashboard.html, 1초마다 `/status`+`/video_feed` 폴링 |
| 카메라 → OCR → 대시보드 | server.py `ocr_worker()`(284) → `state["plate_number"]` → `/status` |

서버(`server.py`) 하나가 전부 중계. ESP32/로보독/카메라/대시보드는 서로 직접 안 붙고 다 서버를 거침.
