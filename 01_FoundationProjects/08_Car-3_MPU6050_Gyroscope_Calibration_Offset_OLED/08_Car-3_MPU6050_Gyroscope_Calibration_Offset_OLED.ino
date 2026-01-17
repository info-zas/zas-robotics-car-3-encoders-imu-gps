#include <Wire.h>
#include <U8x8lib.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ================= OLED (U8X8) =================
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);

// ================= MPU6050 =================
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  delay(500);

  // OLED init
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.clear();
  u8x8.drawString(0, 0, "MPU6050 CAL");

  // MPU init
  if (!mpu.begin()) {
    Serial.println("MPU6050 NOT FOUND!");
    u8x8.drawString(0, 2, "MPU NOT FOUND");
    while (1);
  }

  // Optional: set stable settings
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Keep robot STILL...");
  u8x8.drawString(0, 2, "Keep STILL...");
  delay(2000);

  // ================= Calibration =================
  const int N = 1000;
  double sumZ = 0;

  for (int i = 1; i <= N; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    sumZ += g.gyro.z;  // rad/s

    // small delay for stable sampling
    delay(2);

    // progress on OLED (optional)
    if (i % 100 == 0) {
      u8x8.clearLine(4);
      u8x8.setCursor(0, 4);
      u8x8.print("Samples:");
      u8x8.print(i);
    }
  }

  double biasZ_rad = sumZ / N;
  double biasZ_dps = biasZ_rad * 180.0 / PI;

  // ================= Print Results =================
  Serial.println("========== GYRO CAL DONE ==========");
  Serial.print("Samples: ");
  Serial.println(N);

  Serial.print("GyroZ bias (rad/s): ");
  Serial.println(biasZ_rad, 6);

  Serial.print("GyroZ bias (deg/s): ");
  Serial.println(biasZ_dps, 6);

  // ================= OLED Results =================
  u8x8.clear();
  u8x8.drawString(0, 0, "CAL DONE");

  u8x8.setCursor(0, 2);
  u8x8.print("Z rad/s:");
  u8x8.setCursor(0, 3);
  u8x8.print(biasZ_rad, 4);

  u8x8.setCursor(0, 5);
  u8x8.print("Z deg/s:");
  u8x8.setCursor(0, 6);
  u8x8.print(biasZ_dps, 3);

  // Stop here so value stays visible
  while (1);
}

void loop() {
  // not used
}
