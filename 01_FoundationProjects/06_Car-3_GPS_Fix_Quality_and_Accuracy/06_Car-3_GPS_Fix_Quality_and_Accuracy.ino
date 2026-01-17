#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);
  delay(1000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for (;;);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(18, 25);
  display.println(F("GPS FIX TEST"));
  display.display();
  delay(1200);
}

void loop() {
  while (Serial.available() > 0) {
    gps.encode(Serial.read());
  }

  // refresh every 500ms (even if location not updated)
  static unsigned long last = 0;
  if (millis() - last < 500) return;
  last = millis();

  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(30, 0);
  display.println(F("GPS STATUS"));
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // Fix
  display.setCursor(0, 14);
  display.print(F("Fix : "));
  display.println(gps.location.isValid() ? F("YES") : F("NO"));

  // Age (freshness)
  display.setCursor(0, 24);
  display.print(F("Age : "));
  if (gps.location.isValid()) {
    unsigned long age = gps.location.age();
    if (age == 0xFFFFFFFFUL) display.println(F("--"));
    else {
      display.print(age);
      display.println(F(" ms"));
    }
  } else {
    display.println(F("--"));
  }

  // Lat
  display.setCursor(0, 38);
  display.print(F("Lat: "));
  if (gps.location.isValid()) display.println(gps.location.lat(), 6);
  else display.println(F("--"));

  // Lon
  display.setCursor(0, 48);
  display.print(F("Lon: "));
  if (gps.location.isValid()) display.println(gps.location.lng(), 6);
  else display.println(F("--"));

  display.display();
}
