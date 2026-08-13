/*
 * ESP32 화재 대응 센서 펌웨어
 * 가스/불꽃/온도/습도/초음파(좌우) 값을 1초마다 PC Flask 서버로 POST
 *
 * 필요 라이브러리 (Arduino IDE 라이브러리 매니저에서 설치):
 *   - DHT sensor library (Adafruit)
 *   - Adafruit Unified Sensor (DHT 라이브러리 의존성)
 *
 * 핀 배정은 wiring.md 참고
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>

// ---- Wi-Fi / 서버 설정 (환경에 맞게 수정) ----
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* SERVER_URL = "http://192.168.0.20:5000/sensor_update";  // PC IP 다르면 수정 (cmd에서 ipconfig)

// ---- 핀 배정 (wiring.md) ----
#define FLAME_PIN 27
#define GAS_PIN   34
#define DHT_PIN   14
#define DHT_TYPE  DHT11

#define TRIG_LEFT  5
#define ECHO_LEFT  18
#define TRIG_RIGHT 19
#define ECHO_RIGHT 23

// 불꽃 센서 극성: 라이터를 대봤을 때 Serial 출력으로 실측 후 필요하면 false로 변경
#define FLAME_ACTIVE_LOW true

DHT dht(DHT_PIN, DHT_TYPE);

void setup() {
  Serial.begin(115200);

  pinMode(FLAME_PIN, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  dht.begin();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Wi-Fi 연결 중");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("연결됨, IP: ");
  Serial.println(WiFi.localIP());
}

float readDistanceCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);  // 타임아웃 30ms (약 5m)
  if (duration == 0) return -1;                    // 측정 실패
  return duration * 0.0343 / 2.0;
}

void loop() {
  int flameRaw = digitalRead(FLAME_PIN);
  int flame = FLAME_ACTIVE_LOW ? (flameRaw == LOW ? 1 : 0) : (flameRaw == HIGH ? 1 : 0);

  int gasRaw = analogRead(GAS_PIN);   // ESP32 ADC: 0~4095
  int gas = gasRaw / 4;               // server.py 임계값(0~1023 기준)에 맞춰 스케일 변환

  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();
  float distLeft = readDistanceCm(TRIG_LEFT, ECHO_LEFT);
  float distRight = readDistanceCm(TRIG_RIGHT, ECHO_RIGHT);

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(SERVER_URL);
    http.addHeader("Content-Type", "application/json");

    char payload[224];
    int len = snprintf(payload, sizeof(payload),
      "{\"flame\":%d,\"gas\":%d", flame, gas);

    if (!isnan(temp)) {
      len += snprintf(payload + len, sizeof(payload) - len, ",\"temp\":%.1f", temp);
    }
    if (!isnan(humidity)) {
      len += snprintf(payload + len, sizeof(payload) - len, ",\"humidity\":%.1f", humidity);
    }
    if (distLeft >= 0) {
      len += snprintf(payload + len, sizeof(payload) - len, ",\"dist_cm_left\":%.1f", distLeft);
    }
    if (distRight >= 0) {
      len += snprintf(payload + len, sizeof(payload) - len, ",\"dist_cm_right\":%.1f", distRight);
    }
    snprintf(payload + len, sizeof(payload) - len, "}");

    int status = http.POST(payload);
    Serial.print("POST -> ");
    Serial.print(status);
    Serial.print(" ");
    Serial.println(payload);

    http.end();
  } else {
    Serial.println("Wi-Fi 끊김, 재연결 시도");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  delay(1000);
}
