#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================= MOTOR PINS =================
// Left motor
#define ENCA_LEFT  2
#define PWM_LEFT   6
#define IN1_LEFT   7
#define IN2_LEFT   8

// Right motor
#define ENCA_RIGHT 3
#define PWM_RIGHT  11
#define IN1_RIGHT  12
#define IN2_RIGHT  10

// ================= OLED =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ================= ENCODERS =================
volatile long leftTicks = 0;
volatile long rightTicks = 0;

void ISR_left()  { leftTicks++; }
void ISR_right() { rightTicks++; }

// ================= DISTANCE =================
const float MM_PER_TICK = 0.966;     // calibrated
const float TARGET_DISTANCE_MM = 1000.0;

// ================= SPEED PI CONTROL =================
const unsigned long PID_INTERVAL_MS = 50;  // 20 Hz

// Speed targets (ticks/sec)
const float CRUISE_SPEED = 120.0;
const float SLOW_SPEED   = 60.0;

const float SLOW_ZONE_MM = 60.0;

// PI gains (safe starter values)
const float KP = 0.8;
const float KI = 0.25;

// PWM limits
const int PWM_MIN = 80;
const int PWM_MAX = 180;

// ================= PI STATE =================
float targetSpeedL = 0, targetSpeedR = 0;
float measSpeedL = 0, measSpeedR = 0;
float intL = 0, intR = 0;
int pwmL = 0, pwmR = 0;

long lastTicksL = 0, lastTicksR = 0;
unsigned long lastPIDTime = 0;

// ================= HELPERS =================
void setMotor(int pwmPin, int in1, int in2, int pwm) {
  pwm = constrain(pwm, 0, 255);

  // ✅ FIX: Inverted direction to match your working car-3 code
  // Forward = IN1 LOW, IN2 HIGH
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  analogWrite(pwmPin, pwm);
}

void brakeMotors() {
  // short-brake (ok)
  digitalWrite(IN1_LEFT, HIGH);
  digitalWrite(IN2_LEFT, HIGH);
  digitalWrite(IN1_RIGHT, HIGH);
  digitalWrite(IN2_RIGHT, HIGH);
  analogWrite(PWM_LEFT, 0);
  analogWrite(PWM_RIGHT, 0);
}

void coastMotors() {
  // coast (ok)
  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, LOW);
  digitalWrite(IN1_RIGHT, LOW);
  digitalWrite(IN2_RIGHT, LOW);
  analogWrite(PWM_LEFT, 0);
  analogWrite(PWM_RIGHT, 0);
}

// ================= SETUP =================
void setup() {
  Serial.begin(9600);

  pinMode(IN1_LEFT, OUTPUT);
  pinMode(IN2_LEFT, OUTPUT);
  pinMode(PWM_LEFT, OUTPUT);

  pinMode(IN1_RIGHT, OUTPUT);
  pinMode(IN2_RIGHT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);

  pinMode(ENCA_LEFT, INPUT_PULLUP);
  pinMode(ENCA_RIGHT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA_LEFT), ISR_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_RIGHT), ISR_right, RISING);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  leftTicks = rightTicks = 0;
  lastPIDTime = millis();
}

// ================= LOOP =================
void loop() {
  unsigned long now = millis();
  if (now - lastPIDTime < PID_INTERVAL_MS) return;

  float dt = (now - lastPIDTime) / 1000.0;
  lastPIDTime = now;

  long lTicks, rTicks;
  noInterrupts();
  lTicks = leftTicks;
  rTicks = rightTicks;
  interrupts();

  long dL = lTicks - lastTicksL;
  long dR = rTicks - lastTicksR;
  lastTicksL = lTicks;
  lastTicksR = rTicks;

  measSpeedL = dL / dt;
  measSpeedR = dR / dt;

  long avgTicks = (lTicks + rTicks) / 2;
  float distanceMM = avgTicks * MM_PER_TICK;
  float remainingMM = TARGET_DISTANCE_MM - distanceMM;

  if (remainingMM <= 0) {
    brakeMotors();
    delay(150);
    coastMotors();
    while (1);
  }

  // Speed selection
  float targetSpeed = (remainingMM < SLOW_ZONE_MM) ? SLOW_SPEED : CRUISE_SPEED;
  targetSpeedL = targetSpeedR = targetSpeed;

  // ================= PI CONTROLLER =================
  float errL = targetSpeedL - measSpeedL;
  float errR = targetSpeedR - measSpeedR;

  intL += errL * dt;
  intR += errR * dt;

  pwmL += (int)(KP * errL + KI * intL);
  pwmR += (int)(KP * errR + KI * intR);

  pwmL = constrain(pwmL, PWM_MIN, PWM_MAX);
  pwmR = constrain(pwmR, PWM_MIN, PWM_MAX);

  setMotor(PWM_LEFT, IN1_LEFT, IN2_LEFT, pwmL);
  setMotor(PWM_RIGHT, IN1_RIGHT, IN2_RIGHT, pwmR);

  // ================= OLED DEBUG =================
  display.clearDisplay();

  display.setCursor(0,0);
  display.println("Speed PI 1m");

  display.setCursor(0,10);
  display.print("Dist:");
  display.print(distanceMM,1);
  display.print(" mm");

  display.setCursor(0,20);
  display.print("Spd T:");
  display.print(targetSpeed,0);

  display.setCursor(0,30);
  display.print("L/R:");
  display.print(measSpeedL,0);
  display.print("/");
  display.print(measSpeedR,0);

  display.setCursor(0,40);
  display.print("PWM:");
  display.print(pwmL);
  display.print("/");
  display.print(pwmR);

  display.display();
}
