#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <HCSR04.h>

// ---------- Telegram ----------
const char* BOT_TOKEN = "7654914811:AAGgov2e3QRW2Z92u6IDJ5PzU3LWxxFYmHE";
const String CHAT_ID  = "8077582282";   // your user ID

// ---------- USC Secure Wireless (WPA2-Enterprise: PEAP/MSCHAPv2) ----------
const char* SSID_USC     = "USC Secure Wireless";
const char* EAP_IDENTITY = "mushibuk@usc.edu";   // outer identity
const char* EAP_USERNAME = "mushibuk";           // inner identity
const char* EAP_PASSWORD = "GradMarmar1367!!";   // password

// ---------- Hardware pins ----------
const int PIN_TRIG    = 13;  // HC-SR04 TRIG
const int PIN_ECHO    = 34;  // HC-SR04 ECHO (⚠ use divider if sensor @5V)
const int PIN_BUZZER  = 4;   // Piezo: GPIO4 -> 220Ω -> piezo -> GND
const int PIN_BUTTON  = 33;  // Button -> GND (2-wire), INPUT_PULLUP (pressed=LOW)

// ---------- Ultrasonic ----------
UltraSonicDistanceSensor distanceSensor(PIN_TRIG, PIN_ECHO);

// ---------- Behavior tuning ----------
const float  ALERT_CM        = 60.0f;            // trigger threshold
const uint8_t AVG_SAMPLES    = 3;                // smoothing
const unsigned long ALERT_T1 = 10000UL;          // 10s first alert
const unsigned long ALERT_T2 = 20000UL;          // 20s second alert (additional 10s)

// ---------- Buzzer (LEDC PWM for passive piezo) ----------
const int BUZZ_CH   = 0;
const int BUZZ_FREQ = 4000;    // 3–5 kHz good for piezo
const int BUZZ_RES  = 10;      // 10-bit
int BUZZ_DUTY       = 700;     // volume 0..1023

inline void buzzerOn()  { ledcWrite(BUZZ_CH, BUZZ_DUTY); }
inline void buzzerOff() { ledcWrite(BUZZ_CH, 0); }

// ---------- Button (debounced, pressed = LOW) ----------
const unsigned long DEBOUNCE_MS = 25;
bool btnStable = false, btnLastRaw = true;
unsigned long btnLastChangeMs = 0;
bool buttonPressed() {
  bool raw = (digitalRead(PIN_BUTTON) == LOW);
  if (raw != btnLastRaw) { btnLastRaw = raw; btnLastChangeMs = millis(); }
  if (millis() - btnLastChangeMs > DEBOUNCE_MS) btnStable = raw;
  return btnStable;
}

// ---------- State (latched alarm + staged Telegram) ----------
bool alarmLatched          = false;
bool firstAlertSent        = false;
bool secondAlertSent       = false;
unsigned long latchedAtMs  = 0;

// ---------- Helpers ----------
String urlEncode(const String &msg) {
  String enc; char buf[4];
  for (size_t i = 0; i < msg.length(); i++) {
    uint8_t c = (uint8_t)msg[i];
    if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) enc += (char)c;
    else if (c == ' ') enc += '+';
    else { sprintf(buf, "%%%02X", c); enc += buf; }
  }
  return enc;
}

bool sendTelegram(const String &msg) {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  String url = String("https://api.telegram.org/bot") + BOT_TOKEN +
               "/sendMessage?chat_id=" + CHAT_ID +
               "&text=" + urlEncode(msg);
  http.begin(url);
  http.setConnectTimeout(6000);
  http.setTimeout(6000);
  int status = http.GET();
  http.end();
  Serial.printf("Telegram status: %d\n", status);
  return status == 200;
}

void wifiConnectUSC() {
  WiFi.mode(WIFI_STA);
  // Arduino-ESP32 enterprise overload (PEAP/MSCHAPv2)
  WiFi.begin(SSID_USC, WPA2_AUTH_PEAP, EAP_IDENTITY, EAP_USERNAME, EAP_PASSWORD);

  Serial.print("Connecting to USC Secure Wireless");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 25000) {
    Serial.print('.');
    delay(500);
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Wi-Fi OK: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("Wi-Fi not connected (Enterprise cert/policy may block IoT).");
  }
}

float readDistanceAvgCm() {
  float sum = 0.0f; uint8_t good = 0;
  for (uint8_t i = 0; i < AVG_SAMPLES; i++) {
    double d = distanceSensor.measureDistanceCm(); // cm (<0 on error)
    if (d > 0.0 && d < 400.0) { sum += (float)d; good++; }
    delay(10);
  }
  return good ? (sum / good) : NAN;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  ledcSetup(BUZZ_CH, BUZZ_FREQ, BUZZ_RES);
  ledcAttachPin(PIN_BUZZER, BUZZ_CH);
  buzzerOff();

  Serial.println("Latched proximity alarm + staged Telegram alerts");
  wifiConnectUSC();
}

void loop() {
  // 1) Button clears/silences and fully resets alerts
  if (buttonPressed()) {
    alarmLatched   = false;
    firstAlertSent = false;
    secondAlertSent= false;
    buzzerOff();
  }

  // 2) Always measuring: entering alert zone latches alarm
  float d = readDistanceAvgCm();
  if (!isnan(d)) {
    if (!alarmLatched && d <= ALERT_CM) {
      alarmLatched    = true;
      firstAlertSent  = false;
      secondAlertSent = false;
      latchedAtMs     = millis();
    }
  }

  // 3) Buzzer follows latched state (stays ON until button press)
  if (alarmLatched) buzzerOn(); else buzzerOff();

  // 4) Staged messages while still latched
  if (alarmLatched) {
    unsigned long elapsed = millis() - latchedAtMs;

    // First message at T+10s
    if (!firstAlertSent && elapsed >= ALERT_T1) {
      bool ok = sendTelegram("Maia may be in trouble. Please check in on them to make sure their safe!");
      if (!ok && WiFi.status() != WL_CONNECTED) {
        wifiConnectUSC(); // quick retry
        sendTelegram("Maia may be in trouble. Please check in on them to make sure their safe!");
      }
      firstAlertSent = true;
    }

    // Second message at T+20s
    if (!secondAlertSent && elapsed >= ALERT_T2) {
      bool ok = sendTelegram("Haven't heard from them? Contact help: DPS Emergency Line 213-740-4321");
      if (!ok && WiFi.status() != WL_CONNECTED) {
        wifiConnectUSC(); // quick retry
        sendTelegram("Haven't heard from them? Contact help: DPS Emergency Line 213-740-4321");
      }
      secondAlertSent = true;
    }
  }

  delay(50);
}
