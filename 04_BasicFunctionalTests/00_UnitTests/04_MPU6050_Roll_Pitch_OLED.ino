#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const int MPU = 0x68; // I2C address of MPU6050

int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float roll, pitch;

void setup() {
  Wire.begin();

  // Wake up MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for (;;); // Don't continue, loop forever if OLED not found
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

void loop() {
  // Read raw values from MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Starting register for accelerometer
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  // Convert raw to float for calculation
float fAcX = (float)AcX;
float fAcY = (float)AcY;
float fAcZ = (float)AcZ;

const float epsilon = 0.0001;  // prevent divide-by-zero

roll  = atan2(fAcY, fAcZ) * 180.0 / PI;
pitch = atan2(-fAcX, sqrt(fAcY * fAcY + fAcZ * fAcZ + epsilon)) * 180.0 / PI;



  // Display on OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("AcX: "); display.println(AcX);
  display.print("AcY: "); display.println(AcY);
  display.print("AcZ: "); display.println(AcZ);
  display.print("Roll: "); display.println(roll, 1);
  display.print("Pitch: "); display.println(pitch, 1);
  display.display();

  delay(300);
}
