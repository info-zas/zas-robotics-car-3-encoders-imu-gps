#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

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

// ================= ROBOT CONSTANTS =================
const float MM_PER_TICK = 0.966;     // calibrated
const float WHEELBASE_MM = 170.0;    // given
const float TARGET_DISTANCE_MM = 1000.0;

// ================= SPEED PI =================
const unsigned long PID_INTERVAL_MS = 50;

// Speeds in ticks/sec
const float CRUISE_SPEED = 320.0;
const float SLOW_SPEED   = 160.0;
const float SLOW_ZONE_MM = 60.0;

// PI gains
const float KP = 0.8;
const float KI = 0.25;

// Speed synchronisation gain
const float K_SYNC = 0.35; // trial & error

// PWM limits
const int PWM_MIN = 80;
const int PWM_MAX = 180;

// ================= STATE =================
// Speed control
float measSpeedL = 0, measSpeedR = 0;
float targetSpeedL = 0, targetSpeedR = 0;
float intL = 0, intR = 0;
int pwmL = 100, pwmR = 100;

// Encoder bookkeeping
long lastTicksL = 0, lastTicksR = 0;
unsigned long lastPIDTime = 0;

// ================= ODOMETRY STATE =================
float x_mm = 0.0;
float y_mm = 0.0;
float theta = 0.0;   // radians

// ================= HELPERS =================
void setMotor(int pwmPin, int in1, int in2, int pwm) {
  pwm = constrain(pwm, 0, 255);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(pwmPin, pwm);
}

void brakeMotors() {
  digitalWrite(IN1_LEFT, HIGH);
  digitalWrite(IN2_LEFT, HIGH);
  digitalWrite(IN1_RIGHT, HIGH);
  digitalWrite(IN2_RIGHT, HIGH);
  analogWrite(PWM_LEFT, 0);
  analogWrite(PWM_RIGHT, 0);
}

void coastMotors() {
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

  long dL_ticks = lTicks - lastTicksL;
  long dR_ticks = rTicks - lastTicksR;
  lastTicksL = lTicks;
  lastTicksR = rTicks;

  // ================= SPEED MEASUREMENT =================
  measSpeedL = dL_ticks / dt;
  measSpeedR = dR_ticks / dt;

  // ================= ODOMETRY =================
  float dSL = dL_ticks * MM_PER_TICK;
  float dSR = dR_ticks * MM_PER_TICK;

  float dS  = (dSL + dSR) * 0.5;
  float dTh = (dSR - dSL) / WHEELBASE_MM;

  x_mm += dS * cos(theta + dTh * 0.5);
  y_mm += dS * sin(theta + dTh * 0.5);
  theta += dTh;

  // ================= DISTANCE =================
  float distMM = (lTicks + rTicks) * 0.5 * MM_PER_TICK;
  float remainingMM = TARGET_DISTANCE_MM - distMM;

  if (remainingMM <= 0) {
    brakeMotors();
    delay(150);
    coastMotors();
    while (1);
  }

  // ================= TARGET SPEED =================
  float baseSpeed = (remainingMM < SLOW_ZONE_MM) ? SLOW_SPEED : CRUISE_SPEED;

  // ================= SPEED SYNC =================
  float speedDiff = measSpeedL - measSpeedR;
  targetSpeedL = baseSpeed - K_SYNC * speedDiff;
  targetSpeedR = baseSpeed + K_SYNC * speedDiff;

  // ================= PI CONTROL =================
  float errL = targetSpeedL - measSpeedL;
  float errR = targetSpeedR - measSpeedR;

  intL += errL * dt;
  intR += errR * dt;

  pwmL += KP * errL + KI * intL;
  pwmR += KP * errR + KI * intR;

  pwmL = constrain(pwmL, PWM_MIN, PWM_MAX);
  pwmR = constrain(pwmR, PWM_MIN, PWM_MAX);

  setMotor(PWM_LEFT, IN1_LEFT, IN2_LEFT, pwmL);
  setMotor(PWM_RIGHT, IN1_RIGHT, IN2_RIGHT, pwmR);

  // ================= OLED =================
  display.clearDisplay();

  display.setCursor(0,0);
  display.println("Odometry (x,y,th)");

  display.setCursor(0,10);
  display.print("X:");
  display.print(x_mm/1000.0, 3);
  display.print(" Y:");
  display.print(y_mm/1000.0, 3);

  display.setCursor(0,20);
  display.print("Th:");
  display.print(theta * 180.0 / PI, 2);
  display.println(" deg");

  display.setCursor(0,30);
  display.print("Spd:");
  display.print(measSpeedL,0);
  display.print("/");
  display.print(measSpeedR,0);

  display.setCursor(0,40);
  display.print("PWM:");
  display.print(pwmL);
  display.print("/");
  display.println(pwmR);

  display.setCursor(0,50);
  display.print("Dist:");
  display.print(distMM,1);

  display.display();
}
