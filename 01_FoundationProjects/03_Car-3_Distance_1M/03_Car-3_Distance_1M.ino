#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================== MOTOR PINS ==================
// Left motor
#define ENCA_LEFT 2
#define PWM_LEFT  6
#define IN1_LEFT  7
#define IN2_LEFT  8

// Right motor
#define ENCA_RIGHT 3
#define PWM_RIGHT  11
#define IN1_RIGHT  12
#define IN2_RIGHT  10

// ================== OLED ==================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ================== ENCODERS ==================
volatile long leftTicks = 0;
volatile long rightTicks = 0;

void ISR_left()  { leftTicks++; }
void ISR_right() { rightTicks++; }

// ================== ROBOT CONSTANTS ==================
const float WHEEL_DIAMETER_MM = 65.0;
const float WHEEL_CIRC_MM = 3.1416 * WHEEL_DIAMETER_MM;

const float GEAR_RATIO = 35;
const int   ENCODER_PPR = 11;

const float TICKS_PER_WHEEL_REV = ENCODER_PPR * GEAR_RATIO;
const float MM_PER_TICK = WHEEL_CIRC_MM / TICKS_PER_WHEEL_REV;

const float TARGET_DISTANCE_MM = 1000.0;
const long  TARGET_TICKS = TARGET_DISTANCE_MM / MM_PER_TICK;

// ================== CONTROL ==================
int basePWM = 110;
float kStraight = 0.8;

// ================== HELPERS ==================
void setMotor(int pwmPin, int in1, int in2, int pwm) {
  pwm = constrain(pwm, 0, 255);

  // ✅ FIX: Inverted direction to match your working car-3 code
  // Forward = IN1 LOW, IN2 HIGH
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  analogWrite(pwmPin, pwm);
}

void brakeMotors() {
  // TB6612 "short brake" style (ok)
  digitalWrite(IN1_LEFT, HIGH);
  digitalWrite(IN2_LEFT, HIGH);
  digitalWrite(IN1_RIGHT, HIGH);
  digitalWrite(IN2_RIGHT, HIGH);
  analogWrite(PWM_LEFT, 0);
  analogWrite(PWM_RIGHT, 0);
}

// ================== SETUP ==================
void setup() {
  Serial.begin(9600);

  pinMode(IN1_LEFT, OUTPUT);  pinMode(IN2_LEFT, OUTPUT);  pinMode(PWM_LEFT, OUTPUT);
  pinMode(IN1_RIGHT, OUTPUT); pinMode(IN2_RIGHT, OUTPUT); pinMode(PWM_RIGHT, OUTPUT);

  pinMode(ENCA_LEFT, INPUT_PULLUP);
  pinMode(ENCA_RIGHT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA_LEFT), ISR_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_RIGHT), ISR_right, RISING);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

// ================== LOOP ==================
void loop() {
  long lTicks, rTicks;
  noInterrupts();
  lTicks = leftTicks;
  rTicks = rightTicks;
  interrupts();

  long avgTicks = (lTicks + rTicks) / 2;
  float distanceMM = avgTicks * MM_PER_TICK;

  // Straight correction (simple proportional)
  int correction = (lTicks - rTicks) * kStraight;
  int pwmL = constrain(basePWM - correction, 90, 180);
  int pwmR = constrain(basePWM + correction, 90, 180);

  if (avgTicks < TARGET_TICKS) {
    setMotor(PWM_LEFT, IN1_LEFT, IN2_LEFT, pwmL);
    setMotor(PWM_RIGHT, IN1_RIGHT, IN2_RIGHT, pwmR);
  } else {
    brakeMotors();
    while (1); // stop permanently
  }

  // ================== OLED ==================
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("1m Encoder Drive");

  display.setCursor(0,10);
  display.print("Target ticks:");
  display.println(TARGET_TICKS);

  display.setCursor(0,20);
  display.print("L/R:");
  display.print(lTicks);
  display.print("/");
  display.println(rTicks);

  display.setCursor(0,30);
  display.print("Avg:");
  display.println(avgTicks);

  display.setCursor(0,40);
  display.print("Dist mm:");
  display.println(distanceMM,1);

  display.setCursor(0,50);
  display.print("PWM L/R:");
  display.print(pwmL);
  display.print("/");
  display.println(pwmR);

  display.display();
}
