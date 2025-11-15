#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <TinyGPSPlus.h>
#include <MPU6050_light.h>
#include <QMC5883LCompass.h>

// ==== Motor Pins ====
#define ENCA_L 3
#define ENCB_L 5
#define PWM_L  6
#define IN2_L  7
#define IN1_L  8
#define ENCA_R 2
#define ENCB_R 4
#define PWM_R  11
#define IN2_R  10
#define IN1_R  12

// ==== Rotary Encoder ====
#define ROTARY_CLK A1
#define ROTARY_DT  A2
#define ROTARY_SW  A0

// ==== Display & Sensors ====
SSD1306AsciiWire oled;
TinyGPSPlus gps;
MPU6050 mpu(Wire);
QMC5883LCompass comp;

// ==== GPS variables ====
double lat = 0.0;
double lon = 0.0;


// ==== Variables ====
volatile long encL = 0, encR = 0;
const long goal = 5000;
int lastCLK;
long rot = 0;
unsigned long tMotor = 0, tDisplay = 0;

// ==== Encoder ISRs ====
void encL_ISR() { bool a = digitalRead(ENCA_L), b = digitalRead(ENCB_L); if (a == b) encL++; else encL--; }
void encR_ISR() { bool a = digitalRead(ENCA_R), b = digitalRead(ENCB_R); if (a == b) encR++; else encR--; }

// ==== Motor Control ====
void motorCtrl() {
  // ---- LEFT MOTOR ----
  if (encL >= goal) {
    analogWrite(PWM_L, 0);
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, LOW);   // actively brake
  } else {
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
    analogWrite(PWM_L, 150);
  }

  // ---- RIGHT MOTOR ----
  if (encR >= goal) {
    analogWrite(PWM_R, 0);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, LOW);   // actively brake
  } else {
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
    analogWrite(PWM_R, 150);
  }

  // Clamp counts
  if (encL > goal) encL = goal;
  if (encR > goal) encR = goal;
}


// ==== Rotary ====
void rotUpdate() {
  int c = digitalRead(ROTARY_CLK);
  if (c != lastCLK) {
    if (digitalRead(ROTARY_DT) != c) rot++; else rot--;
  }
  lastCLK = c;
}

// ==== Setup ====
void setup() {
  Serial.begin(9600);
  //Serial.begin(19200);

  Wire.begin();

  // OLED
  oled.begin(&Adafruit128x64, 0x3C);
  oled.setFont(System5x7);
  oled.clear();
  oled.println("Init...");

  // Sensors
  mpu.begin();
  mpu.calcOffsets();
  comp.init();

  // Rotary
  pinMode(ROTARY_CLK, INPUT_PULLUP);
  pinMode(ROTARY_DT, INPUT_PULLUP);
  pinMode(ROTARY_SW, INPUT_PULLUP);
  lastCLK = digitalRead(ROTARY_CLK);

  pinMode(ENCA_R, INPUT_PULLUP);
  pinMode(ENCB_R, INPUT_PULLUP);

  pinMode(ENCA_R, INPUT_PULLUP);
  pinMode(ENCB_R, INPUT_PULLUP);
  // Motors
  pinMode(IN1_L, OUTPUT); pinMode(IN2_L, OUTPUT); pinMode(PWM_L, OUTPUT);
  pinMode(IN1_R, OUTPUT); pinMode(IN2_R, OUTPUT); pinMode(PWM_R, OUTPUT);
  pinMode(ENCA_L, INPUT_PULLUP); pinMode(ENCB_L, INPUT_PULLUP);
  pinMode(ENCA_R, INPUT_PULLUP); pinMode(ENCB_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA_L), encL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_R), encR_ISR, CHANGE);

  oled.println("Setup OK");
  delay(300);
  oled.clear();
}

// ==== Loop ====
void loop() {
  // === GPS Parse ===
  //while (Serial.available()) gps.encode(Serial.read());

 // --- always feed TinyGPS++ ---
while (Serial.available()) {
  gps.encode(Serial.read());
}

// --- use parsed values every 200 ms ---
static unsigned long lastGPS = 0;
if (millis() - lastGPS > 200) {
  lastGPS = millis();

  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
  }
}


/* static long lastL = 0;
if (encL != lastL) {
  Serial.println(encL);
  lastL = encL;
}*/

  // === Sensors ===
  mpu.update();
  comp.read();

  // === Rotary Encoder ===
  rotUpdate();

  // === Motor Control (every 100 ms) ===
  if (millis() - tMotor > 100) {
    tMotor = millis();
    motorCtrl();
  }

  // === OLED Update (every 400 ms) ===
  /*if (millis() - tDisplay > 400) {
    tDisplay = millis();
    oled.clear();
    oled.print("LAT:"); oled.println(gps.location.isValid() ? gps.location.lat() : 0, 2);
    oled.print("LON:"); oled.println(gps.location.isValid() ? gps.location.lng() : 0, 2);
    oled.print("R:"); oled.print((int)mpu.getAngleX());
    oled.print(" P:"); oled.println((int)mpu.getAngleY());
    oled.print("B:"); oled.println(comp.getAzimuth());
    oled.print("Rot:"); oled.println(rot);
    oled.print("Enc:"); oled.print(encL); oled.print("/"); oled.println(encR);
  }*/
  if (millis() - tDisplay > 400) {
  tDisplay = millis();

  oled.setCursor(0,0);              // no clear()
  oled.print("LAT:"); oled.println(gps.location.isValid() ? gps.location.lat() : 0,2);

  oled.print("LON:"); oled.println(gps.location.isValid() ? gps.location.lng() : 0,2);

  oled.print("R:"); oled.print((int)mpu.getAngleX());
  oled.print(" P:"); oled.println((int)mpu.getAngleY());

  oled.print("B:"); oled.println(comp.getAzimuth());

  oled.print("Rot:"); oled.println(rot);

  oled.print("Enc:"); oled.print(encL);
  oled.print("/"); oled.println(encR);

  // pad the remaining lines with spaces so old text is erased
  oled.println("                ");
}
}

