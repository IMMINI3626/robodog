"""로보독 화재 대응 통합 서버.

ESP32(Wi-Fi)에서 불꽃/가스/온습도/초음파 센서값을 POST로 받고,
카메라 프레임을 캡처하며, 화재/가스 감지 시 로보독을 정지시키고
경보 반응을 실행하는 중앙 Flask 서버.

앱인벤터 대시보드는 /status, /snapshot.jpg 를 폴링해서 사용한다.
"""

import atexit
import logging
import os
import re
import threading
import time
from collections import deque
from datetime import datetime

import cv2
from flask import Flask, Response, jsonify, render_template, request, send_file

from tools import RoboDog, STOP

# ---------------------------------------------------------------------------
# 설정값 (실제 하드웨어에 맞게 조정)
# ---------------------------------------------------------------------------

DOG_COM_PORT = "COM4"      # 로보독 무선 동글이 잡는 COM 포트
DOG_PATROL_SPEED = 30      # 평상시 순찰 이동 속도

CAMERA_SOURCE = "http://admin:admin@192.168.0.19:8081/"  # 아이폰 IP Camera 앱 스트림. 폰 IP 바뀌면 여기 수정

FLAME_FIRE_VALUE = 1       # ESP32가 화재 감지 시 보내는 값(0/1로 정규화해서 보내도록 펌웨어에서 처리)
GAS_ALERT_THRESHOLD = 500  # 가스 센서 경보 임계값 - 실측 후 보정 필요

NARROW_DIST_THRESHOLD_CM = 30   # 우드락 미니어처 기준 임시값 - 실제 통로 폭 재고 조정
NARROW_HOLD_SECONDS = 1.0       # 이 시간 이상 지속돼야 확정 (순간 노이즈 방지)

TEMP_TREND_WINDOW_SECONDS = 30      # 온도 추세 계산에 쓰는 시간 창 (짧은 시연에 맞춰 축소)
TEMP_TREND_MIN_SPAN_SECONDS = 10    # 이만큼 시간이 쌓이기 전엔 추세값 신뢰 안 함 (DHT11 정수 단위 튐 방지)
TEMP_TREND_ALERT_PER_MIN = 2.0      # 분당 이 값(°C) 이상 상승하면 사전 경보 - 실측 후 보정 필요

# 화재위험 종합지수: 가스 + 건조지수(습도 기반)만 반영. 전기설비 센서가 없어서 제외 - 나중에 센서 추가되면 가중치 재조정
RISK_GAS_WEIGHT = 0.5
RISK_DRYNESS_WEIGHT = 0.5
RISK_HISTORY_MAX_SAMPLES = 300  # 1초 간격 기준 약 5분치

SNAPSHOT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "snapshots")
os.makedirs(SNAPSHOT_DIR, exist_ok=True)

MOCK_SENSORS = os.environ.get("MOCK_SENSORS", "0") == "1"  # ESP32 없이 테스트할 때만 "1"로 실행

ENABLE_OCR = os.environ.get("ENABLE_OCR", "1") == "1"  # EasyOCR 로딩이 무거워서 끄고 싶을 때 "0"
OCR_INTERVAL_SECONDS = 1.0
OCR_SCALE = 0.5  # OCR 처리용으로 프레임을 이 비율로 축소 (속도 개선, 너무 작으면 인식률 떨어짐)
PLATE_PATTERN = re.compile(r"\d{2,3}[가-힣]\d{4}")

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
    "humidity": None,
    "dist_cm_left": None,
    "dist_cm_right": None,
    "fire_alert": False,
    "alert_reason": None,
    "narrow_passage": False,
    "temp_trend_c_per_min": None,
    "temp_trend_alert": False,
    "temp_alert_119_sent": False,
    "fire_risk_index": None,
    "fire_risk_gas_score": None,
    "fire_risk_dryness_score": None,
    "dog_status": "unknown",   # "patrol" | "stopped"
    "photo_path": None,
    "notify_119_sent": False,
    "plate_number": None,
    "blocking_vehicle_plate": None,
    "blocking_vehicle_detected_at": None,
    "last_sensor_update": None,
}


_narrow_since = None                # 통로가 좁아지기 시작한 monotonic 시각
_temp_samples = deque()             # (monotonic 시각, 온도) 최근 TEMP_TREND_WINDOW_SECONDS 구간
_prev_temp_trend_alert = False      # 온도추세 경보의 직전 상태 (False->True 전환 감지용)
_risk_history = deque(maxlen=RISK_HISTORY_MAX_SAMPLES)  # (시각, fire_risk_index)


def _update_narrow_passage(dist_left, dist_right):
    global _narrow_since
    candidates = [d for d in (dist_left, dist_right) if d is not None]
    is_narrow_now = bool(candidates) and min(candidates) < NARROW_DIST_THRESHOLD_CM
    now = time.monotonic()
    if not is_narrow_now:
        _narrow_since = None
        return False
    if _narrow_since is None:
        _narrow_since = now
    return (now - _narrow_since) >= NARROW_HOLD_SECONDS


def _update_temp_trend(temp):
    if temp is None:
        return None
    now = time.monotonic()
    _temp_samples.append((now, temp))
    while _temp_samples and now - _temp_samples[0][0] > TEMP_TREND_WINDOW_SECONDS:
        _temp_samples.popleft()
    if len(_temp_samples) < 2:
        return None
    oldest_time, oldest_temp = _temp_samples[0]
    span = now - oldest_time
    if span < TEMP_TREND_MIN_SPAN_SECONDS:
        return None
    return (temp - oldest_temp) / (span / 60.0)


def _compute_fire_risk_index(gas, humidity):
    """가스 + 건조지수(습도 기반) 기준 0~100 화재위험 종합지수. 전기설비 항목은 센서 없어서 제외."""
    gas_score = min(100.0, max(0.0, (gas or 0) / GAS_ALERT_THRESHOLD * 100))
    dryness_score = 0.0 if humidity is None else max(0.0, 100.0 - humidity)
    index = gas_score * RISK_GAS_WEIGHT + dryness_score * RISK_DRYNESS_WEIGHT
    return round(index, 1), round(gas_score, 1), round(dryness_score, 1)


def update_state(**kwargs):
    """센서값을 반영하고 화재/가스/통로폭/온도추세/위험지수를 판정한다."""
    global _prev_temp_trend_alert

    with _state_lock:
        state.update(kwargs)
        now_iso = datetime.now().isoformat(timespec="seconds")
        state["last_sensor_update"] = now_iso

        is_flame = state["flame"] == FLAME_FIRE_VALUE
        is_gas = (state["gas"] or 0) >= GAS_ALERT_THRESHOLD
        state["fire_alert"] = is_flame or is_gas
        state["alert_reason"] = "flame" if is_flame else ("gas" if is_gas else None)

        state["narrow_passage"] = _update_narrow_passage(state["dist_cm_left"], state["dist_cm_right"])

        # 통로가 좁아진 시점에 마지막으로 인식된 번호판을 "통행방해 차량"으로 특정
        if state["narrow_passage"] and state["plate_number"]:
            state["blocking_vehicle_plate"] = state["plate_number"]
            state["blocking_vehicle_detected_at"] = now_iso

        trend = _update_temp_trend(state["temp"])
        state["temp_trend_c_per_min"] = round(trend, 2) if trend is not None else None
        state["temp_trend_alert"] = trend is not None and trend >= TEMP_TREND_ALERT_PER_MIN

        # 온도 급상승이 새로 감지된 순간에만 사전경보(119 시뮬레이션) 발송
        if state["temp_trend_alert"] and not _prev_temp_trend_alert:
            log.warning("[119 사전경보 시뮬레이션] 온도 급상승 감지 (%.2f °C/분)", trend)
            state["temp_alert_119_sent"] = True
        elif not state["temp_trend_alert"]:
            state["temp_alert_119_sent"] = False
        _prev_temp_trend_alert = state["temp_trend_alert"]

        risk_index, gas_score, dryness_score = _compute_fire_risk_index(state["gas"], state["humidity"])
        state["fire_risk_index"] = risk_index
        state["fire_risk_gas_score"] = gas_score
        state["fire_risk_dryness_score"] = dryness_score
        _risk_history.append({"time": now_iso, "value": risk_index})

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
    cap = None
    while True:
        if cap is None:
            cap = cv2.VideoCapture(CAMERA_SOURCE)
            if not cap.isOpened():
                log.warning("카메라(%s)를 열 수 없습니다. 5초 후 재시도.", CAMERA_SOURCE)
                cap.release()
                cap = None
                time.sleep(5)
                continue
            log.info("카메라 연결됨: %s", CAMERA_SOURCE)

        ok, frame = cap.read()
        if not ok:
            log.warning("카메라 프레임 읽기 실패. 재연결 시도.")
            cap.release()
            cap = None
            time.sleep(2)
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
# 번호판 OCR (EasyOCR)
# ---------------------------------------------------------------------------

def ocr_worker():
    import easyocr

    log.info("EasyOCR 모델 로딩 중 (최초 1회, 다소 걸림)...")
    reader = easyocr.Reader(["ko", "en"], gpu=False)
    log.info("EasyOCR 준비 완료")

    while True:
        frame = _get_frame_copy()
        if frame is not None:
            try:
                small = cv2.resize(frame, (0, 0), fx=OCR_SCALE, fy=OCR_SCALE)
                results = reader.readtext(small)
                if results:
                    log.info("OCR 인식: %s", [(text, round(conf, 2)) for _, text, conf in results])

                found = None
                for _, text, _conf in results:
                    match = PLATE_PATTERN.search(text.replace(" ", ""))
                    if match:
                        found = match.group()
                        break

                # 번호판이 여러 텍스트 조각으로 나뉘어 인식된 경우 (예: "12가" / "3456") 합쳐서 재시도
                if found is None and results:
                    combined = "".join(text for _, text, _conf in results).replace(" ", "")
                    match = PLATE_PATTERN.search(combined)
                    if match:
                        found = match.group()

                if found:
                    with _state_lock:
                        state["plate_number"] = found
            except Exception:
                log.exception("OCR 처리 실패")
        time.sleep(OCR_INTERVAL_SECONDS)


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
            humidity=round(random.uniform(35, 55), 1),
            dist_cm_left=random.randint(50, 400),
            dist_cm_right=random.randint(50, 400),
        )
        time.sleep(1)


# ---------------------------------------------------------------------------
# 라우트
# ---------------------------------------------------------------------------

@app.route("/")
def dashboard():
    return render_template("dashboard.html")


@app.route("/sensor_update", methods=["POST"])
def sensor_update():
    """ESP32가 1초마다 보내는 센서값을 받는다.

    예: {"flame": 0, "gas": 320, "temp": 24.5, "humidity": 45.0, "dist_cm_left": 350, "dist_cm_right": 210}
    """
    data = request.get_json(force=True, silent=True) or {}
    try:
        update_state(
            flame=int(data.get("flame", 0)),
            gas=int(data.get("gas", 0)),
            temp=float(data["temp"]) if "temp" in data else state.get("temp"),
            humidity=float(data["humidity"]) if "humidity" in data else state.get("humidity"),
            dist_cm_left=float(data["dist_cm_left"]) if "dist_cm_left" in data else state.get("dist_cm_left"),
            dist_cm_right=float(data["dist_cm_right"]) if "dist_cm_right" in data else state.get("dist_cm_right"),
        )
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "invalid payload"}), 400
    return jsonify({"ok": True})


@app.route("/status")
def status():
    return jsonify(get_state())


@app.route("/risk_history")
def risk_history():
    return jsonify(list(_risk_history))


@app.route("/snapshot.jpg")
def snapshot():
    frame = _get_frame_copy()
    if frame is None:
        return ("no frame", 503)
    ok, buf = cv2.imencode(".jpg", frame)
    if not ok:
        return ("encode error", 500)
    return Response(buf.tobytes(), mimetype="image/jpeg")


@app.route("/fire_photo.jpg")
def fire_photo():
    path = get_state().get("photo_path")
    if not path or not os.path.exists(path):
        return ("no photo", 404)
    return send_file(path, mimetype="image/jpeg")


@app.route("/clear_fire_photo", methods=["POST"])
def clear_fire_photo():
    """화재 진압 완료 처리: 사진 삭제 + 센서값 정상화 + 로보독 순찰 재개."""
    with _state_lock:
        path = state.get("photo_path")
        state["photo_path"] = None
    if path and os.path.exists(path):
        try:
            os.remove(path)
        except OSError:
            log.exception("사진 삭제 실패: %s", path)

    update_state(flame=0, gas=0)
    return jsonify({"ok": True})


@app.route("/video_feed")
def video_feed():
    return Response(_mjpeg_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    connect_dog()
    threading.Thread(target=camera_worker, daemon=True).start()
    if ENABLE_OCR:
        threading.Thread(target=ocr_worker, daemon=True).start()
    if MOCK_SENSORS:
        threading.Thread(target=mock_sensor_worker, daemon=True).start()
    app.run(host="0.0.0.0", port=5000)
