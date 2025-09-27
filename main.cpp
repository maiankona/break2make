#define TRIG_PIN 5
#define ECHO_PIN 18
#define BUZZER_PIN 19

// Parameters
const int DIST_THRESHOLD = 150;   // cm
const unsigned long ALERT_DELAY = 15000; // 15 seconds

// State
unsigned long objectDetectedStart = 0;
bool objectInRange = false;
bool buzzerOn = false;

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout ~5m
  long distance = duration * 0.034 / 2; // cm
  return distance;
}

void loop() {
  long distance = readDistanceCM();
  Serial.println("Distance: " + String(distance) + " cm");

  if (distance > 0 && distance < DIST_THRESHOLD) {
    // Object in range
    if (!objectInRange) {
      objectDetectedStart = millis();
      objectInRange = true;
    } else {
      if (!buzzerOn && (millis() - objectDetectedStart >= ALERT_DELAY)) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerOn = true;
        Serial.println("⚠️ ALERT: Object too close for too long!");
      }
    }
  } else {
    // No object in range → reset
    objectInRange = false;
    buzzerOn = false;
    digitalWrite(BUZZER_PIN, LOW);
  }

  delay(200);
}
