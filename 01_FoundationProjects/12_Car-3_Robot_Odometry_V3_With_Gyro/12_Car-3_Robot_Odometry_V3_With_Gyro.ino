#include <Wire.h>
#include <U8x8lib.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ================= OLED =================
U8X8_SSD1306_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE);

// ================= MPU6050 =================
Adafruit_MPU6050 mpu;
float biasZ = 0;

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

// ================= ENCODERS =================
volatile long leftTicks = 0;
volatile long rightTicks = 0;
void ISR_left()  { leftTicks++; }
void ISR_right() { rightTicks++; }

// ================= ROBOT CONSTANTS =================
const float MM_PER_TICK  = 0.966;
const float WHEELBASE_MM = 170.0;
const float TARGET_MM    = 1000.0;

// ================= CONTROL =================
const unsigned long DT_MS = 50;

const float CRUISE_SPEED = 320.0;
const float SLOW_SPEED   = 160.0;
const float SLOW_ZONE_MM = 60.0;

const float KP = 0.8;
const float KI = 0.25;
const float K_SYNC = 0.35;

const int PWM_MIN = 80;
const int PWM_MAX = 180;

// ================= FUSION =================
const float ALPHA = 0.85;     // safer
const float GYRO_DEADBAND = 0.02; // rad/s

// ================= STATE =================
float measSpeedL=0, measSpeedR=0;
float targetSpeedL=0, targetSpeedR=0;
float intL=0, intR=0;
int pwmL=100, pwmR=100;

long lastTicksL=0, lastTicksR=0;
unsigned long lastTime=0;

// Odometry
float x_mm=0, y_mm=0, theta=0;

// ================= MOTOR HELPERS =================
void setMotorFwd(int pwmPin, int in1, int in2, int pwm) {
  pwm = constrain(pwm, 0, 255);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(pwmPin, pwm);
}

void brakeMotors() {
  digitalWrite(IN1_LEFT, HIGH);  digitalWrite(IN2_LEFT, HIGH);
  digitalWrite(IN1_RIGHT, HIGH); digitalWrite(IN2_RIGHT, HIGH);
  analogWrite(PWM_LEFT, 0);
  analogWrite(PWM_RIGHT, 0);
}

void coastMotors() {
  digitalWrite(IN1_LEFT, LOW);  digitalWrite(IN2_LEFT, LOW);
  digitalWrite(IN1_RIGHT, LOW); digitalWrite(IN2_RIGHT, LOW);
  analogWrite(PWM_LEFT, 0);
  analogWrite(PWM_RIGHT, 0);
}

// ================= SETUP =================
void setup() {
  Wire.begin();

  oled.begin();
  oled.setPowerSave(0);
  oled.setFont(u8x8_font_chroma48medium8_r);
  oled.drawString(0,0,"Gyro Odom Z+");

  pinMode(IN1_LEFT, OUTPUT); pinMode(IN2_LEFT, OUTPUT); pinMode(PWM_LEFT, OUTPUT);
  pinMode(IN1_RIGHT, OUTPUT); pinMode(IN2_RIGHT, OUTPUT); pinMode(PWM_RIGHT, OUTPUT);

  pinMode(ENCA_LEFT, INPUT_PULLUP);
  pinMode(ENCA_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_LEFT), ISR_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_RIGHT), ISR_right, RISING);

  if (!mpu.begin()) {
    oled.drawString(0,2,"MPU FAIL");
    while(1);
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // Gyro bias calibration (still)
  const int N=200;
  float s=0;
  for(int i=0;i<N;i++){
    sensors_event_t a,g,t;
    mpu.getEvent(&a,&g,&t);
    s += g.gyro.z;
    delay(5);
  }
  biasZ = s/N;

  lastTime = millis();
}

// ================= LOOP =================
void loop() {
  unsigned long now = millis();
  if (now - lastTime < DT_MS) return;
  float dt = (now - lastTime)/1000.0;
  lastTime = now;

  long lTicks, rTicks;
  noInterrupts();
  lTicks = leftTicks;
  rTicks = rightTicks;
  interrupts();

  long dL = lTicks - lastTicksL;
  long dR = rTicks - lastTicksR;
  lastTicksL = lTicks;
  lastTicksR = rTicks;

  measSpeedL = dL/dt;
  measSpeedR = dR/dt;

  float dSL = dL * MM_PER_TICK;
  float dSR = dR * MM_PER_TICK;
  float dS  = 0.5*(dSL+dSR);
  float dTh_enc = (dSR-dSL)/WHEELBASE_MM;

  sensors_event_t a,g,t;
  mpu.getEvent(&a,&g,&t);
  float omegaZ = g.gyro.z - biasZ;
  if (fabs(omegaZ) < GYRO_DEADBAND) omegaZ = 0;
  float dTh_gyro = omegaZ * dt;

  float dTh = ALPHA*dTh_gyro + (1-ALPHA)*dTh_enc;

  x_mm += dS * cos(theta + 0.5*dTh);
  y_mm += dS * sin(theta + 0.5*dTh);
  theta += dTh;

  float distMM = 0.5*(lTicks+rTicks)*MM_PER_TICK;
  float remain = TARGET_MM - distMM;

  if (remain <= 0) {
    brakeMotors();
    delay(150);
    coastMotors();
    while(1);
  }

  float base = (remain < SLOW_ZONE_MM) ? SLOW_SPEED : CRUISE_SPEED;

  float diff = measSpeedL - measSpeedR;
  targetSpeedL = base - K_SYNC*diff;
  targetSpeedR = base + K_SYNC*diff;

  intL += (targetSpeedL - measSpeedL)*dt;
  intR += (targetSpeedR - measSpeedR)*dt;

  pwmL += KP*(targetSpeedL - measSpeedL) + KI*intL;
  pwmR += KP*(targetSpeedR - measSpeedR) + KI*intR;

  pwmL = constrain(pwmL, PWM_MIN, PWM_MAX);
  pwmR = constrain(pwmR, PWM_MIN, PWM_MAX);

  setMotorFwd(PWM_LEFT, IN1_LEFT, IN2_LEFT, pwmL);
  setMotorFwd(PWM_RIGHT, IN1_RIGHT, IN2_RIGHT, pwmR);

  // OLED (text-safe)
  // ================= OLED (COMPACT DEBUG) =================
char buf[16];

oled.clearLine(0);
oled.clearLine(1);
oled.clearLine(2);
oled.clearLine(3);

// Line 0: X Y (meters)
dtostrf(x_mm/1000.0, 5, 3, buf);
oled.drawString(0, 0, "X:");
oled.drawString(2, 0, buf);

dtostrf(y_mm/1000.0, 5, 3, buf);
oled.drawString(9, 0, "Y:");
oled.drawString(11, 0, buf);

// Line 1: Theta (deg)
dtostrf(theta * 180.0 / M_PI, 6, 2, buf);
oled.drawString(0, 1, "Th:");
oled.drawString(4, 1, buf);

// Line 2: Speed L/R
itoa((int)measSpeedL, buf, 10);
oled.drawString(0, 2, "Sp:");
oled.drawString(3, 2, buf);

itoa((int)measSpeedR, buf, 10);
oled.drawString(8, 2, "/");
oled.drawString(9, 2, buf);

// Line 3: PWM L/R
itoa(pwmL, buf, 10);
oled.drawString(0, 3, "Pw:");
oled.drawString(3, 3, buf);

itoa(pwmR, buf, 10);
oled.drawString(8, 3, "/");
oled.drawString(9, 3, buf);

}
