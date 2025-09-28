#include <Arduino.h>
#include <HCSR04.h>

// -------- Pins --------
const int PIN_TRIG    = 13;  // HC-SR04 TRIG
const int PIN_ECHO    = 34;  // HC-SR04 ECHO (⚠ level-shift to 3.3V if sensor @5V)
const int PIN_BUZZER  = 4;   // Piezo: GPIO4 -> 220Ω -> piezo -> GND
const int PIN_BUTTON  = 33;  // Button to GND (2-wire), INPUT_PULLUP (pressed=LOW)

// -------- Sensor --------
UltraSonicDistanceSensor distanceSensor(PIN_TRIG, PIN_ECHO);

// -------- Tuning --------
const float ALERT_CM      = 60.0f;  // trigger threshold
const uint8_t AVG_SAMPLES = 3;      // distance smoothing

// -------- Buzzer (LEDC PWM for passive piezo) --------
const int BUZZ_CH   = 0;
const int BUZZ_FREQ = 4000;   // 3–5 kHz is good for piezos
const int BUZZ_RES  = 10;     // 10-bit
int BUZZ_DUTY       = 700;    // 0..1023

inline void buzzerOn()  { ledcWrite(BUZZ_CH, BUZZ_DUTY); }
inline void buzzerOff() { ledcWrite(BUZZ_CH, 0); }

// -------- Button (debounced, pressed = LOW) --------
const unsigned long DEBOUNCE_MS = 25;
bool btnStable = false, btnLastRaw = true;
unsigned long btnLastChangeMs = 0;

bool buttonPressed() {
  bool raw = (digitalRead(PIN_BUTTON) == LOW);
  if (raw != btnLastRaw) { btnLastRaw = raw; btnLastChangeMs = millis(); }
  if (millis() - btnLastChangeMs > DEBOUNCE_MS) btnStable = raw;
  return btnStable;
}

// -------- State (latched alarm) --------
bool alarmLatched = false;

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
  pinMode(PIN_BUTTON, INPUT_PULLUP);  // two-wire button on D33 <-> GND

  ledcSetup(BUZZ_CH, BUZZ_FREQ, BUZZ_RES);
  ledcAttachPin(PIN_BUZZER, BUZZ_CH);
  buzzerOff();

  Serial.println("Latched proximity alarm ready");
}

void loop() {
  // Button clears/silences the alarm and rearms the system
  if (buttonPressed()) {
    alarmLatched = false;
    buzzerOff();
  }

  // Take a distance reading (always measuring)
  float d = readDistanceAvgCm();
  if (!isnan(d)) {
    // If armed (not latched) and we are inside the alert zone, latch the alarm
    if (!alarmLatched && d <= ALERT_CM) {
      alarmLatched = true;
    }
  }

  // Output based on latched state (distance no longer matters once latched)
  if (alarmLatched) {
    buzzerOn();
  } else {
    buzzerOff();
  }

  // Optional debug:
  // Serial.printf("d=%.1f  latched=%d  btn=%d\n", isnan(d)?-1:d, alarmLatched?1:0, buttonPressed()?1:0);

  delay(50);
}
