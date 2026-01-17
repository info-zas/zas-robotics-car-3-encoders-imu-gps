#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// === Right Motor Pins ===
#define IN1_R 12
#define IN2_R 10
#define PWM_R 11
#define ENCA_R 3
#define ENCB_R 5

// === Left Motor Pins ===
#define IN1_L 7
#define IN2_L 8
#define PWM_L 6
#define ENCA_L 2
#define ENCB_L 4

// === Encoder Counts ===
volatile long encoderCountR = 0;
volatile long encoderCountL = 0;

// === Encoder ISRs with Direction Detection ===
void readEncoderR() {
  // If ENCB differs, direction flips
  if (digitalRead(ENCB_R) == LOW) encoderCountR--;
  else encoderCountR++;
}

void readEncoderL() {
  if (digitalRead(ENCB_L) == LOW) encoderCountL++;
  else encoderCountL--;
}

void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(IN1_R, OUTPUT); pinMode(IN2_R, OUTPUT); pinMode(PWM_R, OUTPUT);
  pinMode(IN1_L, OUTPUT); pinMode(IN2_L, OUTPUT); pinMode(PWM_L, OUTPUT);

  // Encoder pins (use PULLUP for stability if encoder supports it)
  pinMode(ENCA_R, INPUT_PULLUP);
  pinMode(ENCB_R, INPUT_PULLUP);
  pinMode(ENCA_L, INPUT_PULLUP);
  pinMode(ENCB_L, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA_R), readEncoderR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_L), readEncoderL, RISING);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (true);
  }

  // Start both motors forward (test)
  digitalWrite(IN1_R, HIGH); digitalWrite(IN2_R, LOW);
  analogWrite(PWM_R, 180);

  digitalWrite(IN1_L, HIGH); digitalWrite(IN2_L, LOW);
  analogWrite(PWM_L, 180);
}

void drawEncoderOLED(long rCnt, long lCnt) {
  display.clearDisplay();

  // Header
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("ENCODER COUNTS");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // Right count (big)
  display.setTextSize(1);
  display.setCursor(0, 16);
  display.print("Right");

  display.setTextSize(2);
  display.setCursor(60, 14);
  display.println(rCnt);

  // Left count (big)
  display.setTextSize(1);
  display.setCursor(0, 44);
  display.print("Left");

  display.setTextSize(2);
  display.setCursor(60, 42);
  display.println(lCnt);

  display.display();
}

void loop() {
  // Atomic snapshot of counts (avoid partial read while ISR updates)
  long r, l;
  noInterrupts();
  r = encoderCountR;
  l = encoderCountL;
  interrupts();

  // Serial log
  Serial.print("Right: "); Serial.print(r);
  Serial.print(" | Left: "); Serial.println(l);

  // OLED
  drawEncoderOLED(r, l);

  delay(200);
}
