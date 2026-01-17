#include <Wire.h>
#include <U8x8lib.h>
#include <math.h>

// ================= OLED =================
U8X8_SSD1306_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE);

// ================= MOTOR PINS =================
#define ENCA_LEFT  2
#define PWM_LEFT   6
#define IN1_LEFT   7
#define IN2_LEFT   8

#define ENCA_RIGHT 3
#define PWM_RIGHT  11
#define IN1_RIGHT  12
#define IN2_RIGHT  10

// ================= ROBOT PARAMS =================
const float MM_PER_TICK = 0.966;     // mm per encoder tick (calibrated)
const float WHEEL_BASE  = 0.17;      // meters (distance between wheels)

// ================= ENCODERS =================
volatile long ticksL = 0;
volatile long ticksR = 0;
void ISR_L() { ticksL++; }
void ISR_R() { ticksR++; }

// ================= ODOMETRY =================
float x = 0, y = 0, theta = 0;       // meters, meters, radians

// =====================================================
// SINE PATH (MAKE IT VERY NOTICEABLE)
// y(x) = A * sin(2*pi*x / lambda)
// =====================================================
const float AMP     = 0.55;          // meters (big sideways swing)
const float LAMBDA  = 1.20;          // meters (shorter => more waves)
const float K       = 2.0f * M_PI / LAMBDA;

// Travel distance (choose >= 2*LAMBDA for visible waves)
const float TOTAL_X = 3.6;           // meters (~3 full waves)

// =====================================================
// PURE PURSUIT STYLE SETTINGS (makes curve look great)
// =====================================================
const float LOOKAHEAD = 0.35;        // meters (0.25–0.45 typical)
const float K_STEER   = 1.8;         // steering strength (rad -> turn)
const int   BASE_PWM  = 105;         // increase speed for better visibility

// PWM limits
const int PWM_MIN = 70;
const int PWM_MAX = 170;

// ================= MOTOR =================
void setPWM(int left, int right) {
  left  = constrain(left, 0, 255);
  right = constrain(right, 0, 255);

  // ✅ Forward direction for YOUR wiring:
  // IN1 = LOW, IN2 = HIGH
  digitalWrite(IN1_LEFT,  LOW);
  digitalWrite(IN2_LEFT,  HIGH);
  digitalWrite(IN1_RIGHT, LOW);
  digitalWrite(IN2_RIGHT, HIGH);

  analogWrite(PWM_LEFT, left);
  analogWrite(PWM_RIGHT, right);
}

// ================= ODOMETRY UPDATE =================
void updateOdometry() {
  static long lastL = 0, lastR = 0;

  // atomic read
  noInterrupts();
  long curL = ticksL;
  long curR = ticksR;
  interrupts();

  long dL = curL - lastL;
  long dR = curR - lastR;
  lastL = curL;
  lastR = curR;

  float dl = dL * MM_PER_TICK / 1000.0;  // meters
  float dr = dR * MM_PER_TICK / 1000.0;  // meters

  float dc     = (dl + dr) / 2.0;
  float dtheta = (dr - dl) / WHEEL_BASE;

  x     += dc * cos(theta + dtheta / 2.0);
  y     += dc * sin(theta + dtheta / 2.0);
  theta += dtheta;

  // keep theta readable
  while (theta > M_PI)  theta -= 2 * M_PI;
  while (theta < -M_PI) theta += 2 * M_PI;
}

// ================= PATH FUNCTION =================
float yRef(float xq) {
  return AMP * sin(K * xq);
}

// ================= OLED DEBUG =================
void showOLED(int pwmL, int pwmR, float yref, float alphaDeg) {
  char buf[16];

  oled.clearLine(0);
  oled.clearLine(1);
  oled.clearLine(2);
  oled.clearLine(3);
  oled.clearLine(4);
  oled.clearLine(5);

  oled.drawString(0,0,"SINE PUREPUR");

  oled.drawString(0,1,"x:");
  dtostrf(x,5,2,buf);
  oled.drawString(3,1,buf);

  oled.drawString(0,2,"y:");
  dtostrf(y,5,2,buf);
  oled.drawString(3,2,buf);

  oled.drawString(9,2,"r:");
  dtostrf(yref,5,2,buf);
  oled.drawString(12,2,buf);

  oled.drawString(0,3,"th:");
  dtostrf(theta * 180.0 / M_PI,5,1,buf);
  oled.drawString(4,3,buf);

  oled.drawString(0,4,"a:");
  dtostrf(alphaDeg,5,1,buf);
  oled.drawString(3,4,buf);

  oled.drawString(0,5,"PWM:");
  itoa(pwmL, buf, 10);
  oled.drawString(4,5,buf);
  oled.drawString(8,5,"/");
  itoa(pwmR, buf, 10);
  oled.drawString(10,5,buf);
}

// ================= SINE MOTION (PURE PURSUIT) =================
void driveSinePurePursuit() {
  // reset
  noInterrupts();
  ticksL = 0;
  ticksR = 0;
  interrupts();
  x = 0; y = 0; theta = 0;

  while (x < TOTAL_X) {
    updateOdometry();

    // Lookahead point on path
    float xL = x + LOOKAHEAD;
    float yL = yRef(xL);

    // Vector from robot -> lookahead point
    float dx = xL - x;
    float dy = yL - y;

    // Desired heading to that point
    float theta_target = atan2(dy, dx);

    // Heading error alpha (wrap)
    float alpha = theta_target - theta;
    while (alpha > M_PI)  alpha -= 2*M_PI;
    while (alpha < -M_PI) alpha += 2*M_PI;

    // Convert heading error into differential PWM
    // Bigger alpha -> more left/right split
    int diff = (int)(K_STEER * 90.0 * alpha);  // 90 is scaling for “feel”

    int pwmL = BASE_PWM - diff;
    int pwmR = BASE_PWM + diff;

    pwmL = constrain(pwmL, PWM_MIN, PWM_MAX);
    pwmR = constrain(pwmR, PWM_MIN, PWM_MAX);

    setPWM(pwmL, pwmR);

    float y_now_ref = yRef(x);
    showOLED(pwmL, pwmR, y_now_ref, alpha * 180.0 / M_PI);

    // Serial log for plotting
    Serial.print(x,3); Serial.print(",");
    Serial.print(y,3); Serial.print(",");
    Serial.print(y_now_ref,3); Serial.print(",");
    Serial.println(alpha,3);

    delay(35);
  }

  setPWM(0, 0);
  oled.drawString(0,7,"DONE");
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(IN1_LEFT, OUTPUT);
  pinMode(IN2_LEFT, OUTPUT);
  pinMode(PWM_LEFT, OUTPUT);
  pinMode(IN1_RIGHT, OUTPUT);
  pinMode(IN2_RIGHT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);

  pinMode(ENCA_LEFT, INPUT_PULLUP);
  pinMode(ENCA_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_LEFT), ISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_RIGHT), ISR_R, RISING);

  oled.begin();
  oled.setPowerSave(0);
  oled.setFont(u8x8_font_chroma48medium8_r);
  oled.drawString(0,0,"READY");

  delay(1500);
  driveSinePurePursuit();
}

void loop() {}
