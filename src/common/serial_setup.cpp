#include "serial_setup.h"

void serialSetup(const char* app_name, uint32_t wait_ms) {
  Serial.begin(115200);

  // Serial is a bool-convertible object; on the STM32 VCP this lets a
  // terminal attach before we print. Timeout so a headless boot still runs.
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < wait_ms) {
    delay(10);
  }

  Serial.println();
  Serial.println(F("========================================"));
  Serial.print(F("APP   : ")); Serial.println(app_name);
  Serial.print(F("BUILD : ")); Serial.print(F(__DATE__));
  Serial.print(F(" "));        Serial.println(F(__TIME__));
  Serial.print(F("MCU   : ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));
  Serial.println(F("========================================"));

  pinMode(LED_BUILTIN, OUTPUT);
}

void fatal(const char* msg, uint8_t blinks) {
  Serial.print(F("FATAL: ")); Serial.println(msg);
  Serial.flush();
  pinMode(LED_BUILTIN, OUTPUT);
  for (;;) {
    for (uint8_t i = 0; i < blinks; i++) {
      digitalWrite(LED_BUILTIN, HIGH); delay(150);
      digitalWrite(LED_BUILTIN, LOW);  delay(150);
    }
    delay(1000);
  }
}