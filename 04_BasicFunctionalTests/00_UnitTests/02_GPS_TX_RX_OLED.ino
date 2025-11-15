#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ------------------- OLED SETTINGS -------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1    // Reset pin (not used)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ------------------- GPS SETTINGS -------------------
TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);   // GPS connected to Arduino RX (D0) and TX (D1)
  delay(1000);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Common I2C address: 0x3C
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't continue
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("GPS Module Test"));
  display.display();
  delay(1500);
}

void loop() {
  // Read GPS data from Serial
  while (Serial.available() > 0) {
    gps.encode(Serial.read());

    if (gps.location.isUpdated()) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);

      // Latitude
      display.print(F("Lat: "));
      if (gps.location.isValid())
        display.println(gps.location.lat(), 6);
      else
        display.println(F("Invalid"));

      // Longitude
      display.print(F("Lon: "));
      if (gps.location.isValid())
        display.println(gps.location.lng(), 6);
      else
        display.println(F("Invalid"));

      // Date
      display.print(F("Date: "));
      if (gps.date.isValid()) {
        display.print(gps.date.day());
        display.print("/");
        display.print(gps.date.month());
        display.print("/");
        display.println(gps.date.year());
      } else {
        display.println(F("Invalid"));
      }

      // Time (UTC)
      display.print(F("Time: "));
      if (gps.time.isValid()) {
        if (gps.time.hour() < 10) display.print("0");
        display.print(gps.time.hour()); display.print(":");
        if (gps.time.minute() < 10) display.print("0");
        display.print(gps.time.minute()); display.print(":");
        if (gps.time.second() < 10) display.print("0");
        display.println(gps.time.second());
      } else {
        display.println(F("Invalid"));
      }

      display.display(); // Show all text on OLED

      // Optional: Print to Serial as well
      Serial.print("Lat: "); Serial.print(gps.location.lat(), 6);
      Serial.print(", Lon: "); Serial.print(gps.location.lng(), 6);
      Serial.print(", Date: ");
      if (gps.date.isValid()) {
        Serial.print(gps.date.day()); Serial.print("/");
        Serial.print(gps.date.month()); Serial.print("/");
        Serial.print(gps.date.year());
      } else Serial.print("Invalid");
      Serial.print(", Time: ");
      if (gps.time.isValid()) {
        Serial.print(gps.time.hour()); Serial.print(":");
        Serial.print(gps.time.minute()); Serial.print(":");
        Serial.print(gps.time.second());
      } else Serial.print("Invalid");
      Serial.println();
    }
  }
}
