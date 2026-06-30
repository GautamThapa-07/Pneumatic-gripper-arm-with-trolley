#define BLYNK_TEMPLATE_ID   "TMPL3CYeHEy_D"
#define BLYNK_TEMPLATE_NAME "arm bend"
#define BLYNK_AUTH_TOKEN    "l6ISKaCyie6aw0hMES5LZNUmOBvVehvS"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>

char ssid[] = "150";
char pass[] = "12345678"; 

// =====================================================
//   SERVO SETUP
// =====================================================

Servo servo1, servo2, servo3, servo4;

const int SERVO_PIN_1 = 13;
const int SERVO_PIN_2 = 14;   // gripper servo — needs counter-torque
const int SERVO_PIN_3 = 27;
const int SERVO_PIN_4 = 26;

const int DEAD_LOW   = 462;
const int DEAD_HIGH  = 561;
const int STOP_PULSE = 1500;  // stop for servo 1, 3, 4

// ── CHANGED: idle hold pulse for servo 2 (pin 14) ──────────
// 1450 = light clockwise torque to counter gripper weight.
// Tune this value: lower = more clockwise force (e.g. 1400, 1350)
//                  higher = less force (e.g. 1480, 1490)
const int HOLD_PULSE_S2 = 1335;
const int HOLD_PULSE_S3 = 1440;
// ────────────────────────────────────────────────────────────

// Servo 2 gets its own pulse mapper so centre returns HOLD_PULSE_S2
int joystickToPulse(int joyVal) {
  if (joyVal > DEAD_LOW && joyVal < DEAD_HIGH) {
    return STOP_PULSE;          // stop for servo 1, 3, 4
  }
  return map(joyVal, 0, 1023, 1000, 2000);
}

// ── CHANGED: separate mapper for servo 2 ───────────────────
// When joystick is released (centre), returns HOLD_PULSE_S2
// instead of STOP_PULSE so the gripper holds position.
int joystickToPulseS2(int joyVal) {
  if (joyVal > DEAD_LOW && joyVal < DEAD_HIGH) {
    return HOLD_PULSE_S2;       // hold clockwise instead of stopping
  }
  return map(joyVal, 0, 1023, 1000, 2000);
}
int joystickToPulseS3(int joyVal) {
  if (joyVal > DEAD_LOW && joyVal < DEAD_HIGH) {
    return HOLD_PULSE_S3;       // hold clockwise instead of stopping
  }
  return map(joyVal, 0, 1023, 1000, 2000);
}
// ────────────────────────────────────────────────────────────

BLYNK_WRITE(V0) { servo1.writeMicroseconds(joystickToPulse(param.asInt())); }

// ── CHANGED: V1 uses joystickToPulseS2 instead of joystickToPulse ──
BLYNK_WRITE(V1) { servo2.writeMicroseconds(joystickToPulseS2(param.asInt())); }
// ────────────────────────────────────────────────────────────

BLYNK_WRITE(V2) { servo3.writeMicroseconds(joystickToPulseS3(param.asInt())); }
BLYNK_WRITE(V3) { servo4.writeMicroseconds(joystickToPulse(param.asInt())); }

// =====================================================
//   DC MOTOR SETUP  (L298N x2)
//   D-Pad: V4=Forward V5=Backward V6=Left V7=Right
//
//   L298N #1 — Left motors
//     IN1→GPIO25  IN2→GPIO33  ENA→GPIO32
//   L298N #2 — Right motors
//     IN3→GPIO19  IN4→GPIO18  ENB→GPIO4
// =====================================================

const int LEFT_IN1  = 25;
const int LEFT_IN2  = 33;
const int LEFT_ENA  = 32;

const int RIGHT_IN3 = 19;
const int RIGHT_IN4 = 18;
const int RIGHT_ENB = 4;

const int MOTOR_SPEED = 255;   // 0–255, tune if too fast/slow

bool btnForward  = false;
bool btnBackward = false;
bool btnLeft     = false;
bool btnRight    = false;

void setLeftMotors(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
    ledcWrite(LEFT_ENA, speed);
  } else if (speed < 0) {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
    ledcWrite(LEFT_ENA, -speed);
  } else {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    ledcWrite(LEFT_ENA, 0);
  }
}

void setRightMotors(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(RIGHT_IN3, HIGH);
    digitalWrite(RIGHT_IN4, LOW);
    ledcWrite(RIGHT_ENB, speed);
  } else if (speed < 0) {
    digitalWrite(RIGHT_IN3, LOW);
    digitalWrite(RIGHT_IN4, HIGH);
    ledcWrite(RIGHT_ENB, -speed);
  } else {
    digitalWrite(RIGHT_IN3, LOW);
    digitalWrite(RIGHT_IN4, LOW);
    ledcWrite(RIGHT_ENB, 0);
  }
}

void stopMotors() {
  setLeftMotors(0);
  setRightMotors(0);
}

void applyDrive() {
  if (btnForward && !btnLeft && !btnRight) {
    setLeftMotors(MOTOR_SPEED);
    setRightMotors(MOTOR_SPEED);
  } else if (btnBackward && !btnLeft && !btnRight) {
    setLeftMotors(-MOTOR_SPEED);
    setRightMotors(-MOTOR_SPEED);
  } else if (btnLeft && !btnForward && !btnBackward) {
    setLeftMotors(-MOTOR_SPEED);
    setRightMotors(MOTOR_SPEED);
  } else if (btnRight && !btnForward && !btnBackward) {
    setLeftMotors(MOTOR_SPEED);
    setRightMotors(-MOTOR_SPEED);
  } else if (btnForward && btnLeft) {
    setLeftMotors(140);
    setRightMotors(MOTOR_SPEED);
  } else if (btnForward && btnRight) {
    setLeftMotors(MOTOR_SPEED);
    setRightMotors(140);
  } else if (btnBackward && btnLeft) {
    setLeftMotors(-140);
    setRightMotors(-MOTOR_SPEED);
  } else if (btnBackward && btnRight) {
    setLeftMotors(-MOTOR_SPEED);
    setRightMotors(-140);
  } else {
    stopMotors();
  }
}

BLYNK_WRITE(V4) { btnForward  = param.asInt(); applyDrive(); }
BLYNK_WRITE(V5) { btnBackward = param.asInt(); applyDrive(); }
BLYNK_WRITE(V6) { btnLeft     = param.asInt(); applyDrive(); }
BLYNK_WRITE(V7) { btnRight    = param.asInt(); applyDrive(); }

// =====================================================
//   SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  // Attach all servos
  servo1.attach(SERVO_PIN_1, 1000, 2000);
  servo2.attach(SERVO_PIN_2, 1000, 2000);
  servo3.attach(SERVO_PIN_3, 1000, 2000);
  servo4.attach(SERVO_PIN_4, 1000, 2000);

  // Stop servos 1, 3, 4 at neutral
  servo1.writeMicroseconds(STOP_PULSE);
  servo4.writeMicroseconds(STOP_PULSE);
  servo3.writeMicroseconds(STOP_PULSE);

  // ── CHANGED: servo 2 boots with hold torque, not stop ──
  servo2.writeMicroseconds(HOLD_PULSE_S2);
  // ────────────────────────────────────────────────────────

  // Motor pins
  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);

  // Motor PWM (ESP32 core v3.x)
  ledcAttach(LEFT_ENA,  1000, 8);
  ledcAttach(RIGHT_ENB, 1000, 8);

  stopMotors();

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("Connected to Blynk!");
}

// =====================================================
//   LOOP
// =====================================================

void loop() {
  Blynk.run();
}