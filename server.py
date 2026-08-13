"""로보독 화재 대응 통합 서버.

ESP32(Wi-Fi)에서 불꽃/가스/온습도/초음파 센서값을 POST로 받고,
카메라 프레임을 캡처하며, 화재/가스 감지 시 로보독을 정지시키고
경보 반응을 실행하는 중앙 Flask 서버.

앱인벤터 대시보드는 /status, /snapshot.jpg 를 폴링해서 사용한다.
"""

import atexit
import logging
import os
import threading
import time
from datetime import datetime

import cv2
from flask import Flask, Response, jsonify, request

from tools import RoboDog, STOP

# ---------------------------------------------------------------------------
# 설정값 (실제 하드웨어에 맞게 조정)
# ---------------------------------------------------------------------------

DOG_COM_PORT = "COM4"      # 로보독 무선 동글이 잡는 COM 포트
DOG_PATROL_SPEED = 30      # 평상시 순찰 이동 속도

CAMERA_SOURCE = 0          # 개발용 PC 웹캠.
                           # 폰 IP웹캠 연동 시 "http://<폰IP>:포트/video" 로 교체 (작업 8, 추후 진행)

FLAME_FIRE_VALUE = 1       # ESP32가 화재 감지 시 보내는 값(0/1로 정규화해서 보내도록 펌웨어에서 처리)
GAS_ALERT_THRESHOLD = 500  # 가스 센서 경보 임계값 - 실측 후 보정 필요

SNAPSHOT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "snapshots")
os.makedirs(SNAPSHOT_DIR, exist_ok=True)

MOCK_SENSORS = os.environ.get("MOCK_SENSORS", "0") == "1"  # ESP32 없이 테스트할 때만 "1"로 실행

logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(message)s")
log = logging.getLogger("robodog-server")

app = Flask(__name__)

# ---------------------------------------------------------------------------
# 전역 상태 (Firebase 대신 쓰는 파이썬 프로세스 내 "DB")
# ---------------------------------------------------------------------------

_state_lock = threading.Lock()
state = {
    "flame": 0,
    "gas": 0,
    "temp": None,
    "dist_cm_left": None,
    "dist_cm_right": None,
    "fire_alert": False,
    "alert_reason": None,
    "narrow_passage": False,   # 작업 6(통로 폭 판단)에서 계산 예정, 현재는 항상 False
    "dog_status": "unknown",   # "patrol" | "stopped"
    "photo_path": None,
    "notify_119_sent": False,
    "last_sensor_update": None,
}


def update_state(**kwargs):
    """센서값을 반영하고 화재/가스 경보를 판정한다."""
    with _state_lock:
        state.update(kwargs)
        state["last_sensor_update"] = datetime.now().isoformat(timespec="seconds")

        is_flame = state["flame"] == FLAME_FIRE_VALUE
        is_gas = (state["gas"] or 0) >= GAS_ALERT_THRESHOLD
        state["fire_alert"] = is_flame or is_gas
        state["alert_reason"] = "flame" if is_flame else ("gas" if is_gas else None)

        snapshot = dict(state)

    _handle_fire_transition(snapshot)


def get_state():
    with _state_lock:
        return dict(state)


# ---------------------------------------------------------------------------
# 로보독 연결 및 화재 반응 로직
# ---------------------------------------------------------------------------

dog = RoboDog()
_dog_connected = False


def connect_dog():
    global _dog_connected
    _dog_connected = dog.Open(DOG_COM_PORT)
    if _dog_connected:
        dog.move(DOG_PATROL_SPEED)
        with _state_lock:
            state["dog_status"] = "patrol"
    else:
        log.warning(
            "로보독 연결 실패 (%s) - 서버는 계속 실행되지만 로보독 제어는 동작하지 않습니다.",
            DOG_COM_PORT,
        )


@atexit.register
def _shutdown_dog():
    if _dog_connected:
        try:
            dog.move(STOP)
            dog.Close()
        except Exception:
            log.exception("로보독 종료 중 오류")


def _handle_fire_transition(snapshot):
    """fire_alert 상태 변화에 반응해서 로보독을 정지/경보시키거나 순찰을 재개한다."""
    if not _dog_connected:
        return

    if snapshot["fire_alert"] and snapshot["dog_status"] != "stopped":
        dog.move(STOP)
        try:
            dog.sound(0, 1, 80)  # TODO: 실제 경보음 트랙 번호로 교체
        except Exception:
            log.exception("경보음 재생 실패")

        photo_path = _save_snapshot()
        log.warning("[119 신고 시뮬레이션] 화재/가스 감지 (%s)", snapshot["alert_reason"])

        with _state_lock:
            state["dog_status"] = "stopped"
            state["photo_path"] = photo_path
            state["notify_119_sent"] = True

    elif not snapshot["fire_alert"] and snapshot["dog_status"] == "stopped":
        dog.move(DOG_PATROL_SPEED)
        with _state_lock:
            state["dog_status"] = "patrol"
            state["notify_119_sent"] = False


# ---------------------------------------------------------------------------
# 카메라
# ---------------------------------------------------------------------------

_frame_lock = threading.Lock()
_latest_frame = None


def camera_worker():
    global _latest_frame
    cap = cv2.VideoCapture(CAMERA_SOURCE)
    if not cap.isOpened():
        log.warning("카메라(%s)를 열 수 없습니다.", CAMERA_SOURCE)
        return
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.5)
            continue
        with _frame_lock:
            _latest_frame = frame
        time.sleep(0.03)


def _get_frame_copy():
    with _frame_lock:
        return None if _latest_frame is None else _latest_frame.copy()


def _save_snapshot():
    frame = _get_frame_copy()
    if frame is None:
        return None
    filename = f"fire_{datetime.now():%Y%m%d_%H%M%S}.jpg"
    path = os.path.join(SNAPSHOT_DIR, filename)
    cv2.imwrite(path, frame)
    return path


def _mjpeg_generator():
    while True:
        frame = _get_frame_copy()
        if frame is None:
            time.sleep(0.1)
            continue
        ok, buf = cv2.imencode(".jpg", frame)
        if not ok:
            continue
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n")
        time.sleep(0.05)


# ---------------------------------------------------------------------------
# 개발용 센서 목업 (ESP32 없이 테스트할 때만)
# ---------------------------------------------------------------------------

def mock_sensor_worker():
    import random

    log.info("MOCK_SENSORS=1 - ESP32 없이 임의 센서값으로 동작합니다.")
    while True:
        update_state(
            flame=0,
            gas=random.randint(100, 200),
            temp=round(random.uniform(22, 26), 1),
            dist_cm_left=random.randint(50, 400),
            dist_cm_right=random.randint(50, 400),
        )
        time.sleep(1)


# ---------------------------------------------------------------------------
# 라우트
# ---------------------------------------------------------------------------

@app.route("/sensor_update", methods=["POST"])
def sensor_update():
    """ESP32가 1초마다 보내는 센서값을 받는다.

    예: {"flame": 0, "gas": 320, "temp": 24.5, "dist_cm_left": 350, "dist_cm_right": 210}
    """
    data = request.get_json(force=True, silent=True) or {}
    try:
        update_state(
            flame=int(data.get("flame", 0)),
            gas=int(data.get("gas", 0)),
            temp=float(data["temp"]) if "temp" in data else state.get("temp"),
            dist_cm_left=float(data["dist_cm_left"]) if "dist_cm_left" in data else state.get("dist_cm_left"),
            dist_cm_right=float(data["dist_cm_right"]) if "dist_cm_right" in data else state.get("dist_cm_right"),
        )
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "invalid payload"}), 400
    return jsonify({"ok": True})


@app.route("/status")
def status():
    return jsonify(get_state())


@app.route("/snapshot.jpg")
def snapshot():
    frame = _get_frame_copy()
    if frame is None:
        return ("no frame", 503)
    ok, buf = cv2.imencode(".jpg", frame)
    if not ok:
        return ("encode error", 500)
    return Response(buf.tobytes(), mimetype="image/jpeg")


@app.route("/video_feed")
def video_feed():
    return Response(_mjpeg_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    connect_dog()
    threading.Thread(target=camera_worker, daemon=True).start()
    if MOCK_SENSORS:
        threading.Thread(target=mock_sensor_worker, daemon=True).start()
    app.run(host="0.0.0.0", port=5000)
