#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println("Cleany ESP32-S3 controller ready");
  Serial.println("Run hardware checks with pio test.");
}

void loop() {
  delay(1000);
}
