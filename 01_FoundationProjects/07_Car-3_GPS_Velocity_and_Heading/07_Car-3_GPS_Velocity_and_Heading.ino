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
  Serial.begin(9600);   // GPS on D0/D1
  delay(1000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for (;;);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(8, 25);
  display.println(F("GPS SPEED & DIR"));
  display.display();
  delay(1200);
}

void loop() {
  while (Serial.available() > 0) {
    gps.encode(Serial.read());
  }

  static unsigned long last = 0;
  if (millis() - last < 500) return;
  last = millis();

  display.clearDisplay();

  // Title
  display.setCursor(30, 0);
  display.println(F("GPS STATUS"));
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // Fix + satellites (small, top line)
  display.setCursor(0, 14);
  display.print(F("Fix: "));
  display.print(gps.location.isValid() ? F("YES") : F("NO"));
  display.print(F("  S: "));
  if (gps.satellites.isValid()) display.println(gps.satellites.value());
  else display.println(F("--"));

  // Speed
  display.setCursor(0, 28);
  display.print(F("Speed: "));
  if (gps.speed.isValid()) {
    display.print(gps.speed.kmph(), 1);
    display.println(F(" km/h"));
  } else {
    display.println(F("--"));
  }

  // Direction / course
  display.setCursor(0, 40);
  display.print(F("Dir  : "));
  if (gps.course.isValid()) {
    display.print(gps.course.deg(), 0);
    display.println(F(" deg"));
  } else {
    display.println(F("--"));
  }

  // Move flag (simple threshold)
  display.setCursor(0, 52);
  display.print(F("Move : "));
  if (gps.speed.isValid() && gps.speed.kmph() > 1.0) display.println(F("YES"));
  else display.println(F("NO"));

  display.display();
}
