#include <Arduino.h>

namespace {
constexpr uint8_t TRIG_PIN = 10;
constexpr uint8_t ECHO_PIN = 11;
constexpr uint32_t ECHO_TIMEOUT_US = 30000;
constexpr uint32_t INTERVAL_MS = 100;
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(115200);
  delay(500);
  Serial.println("Teste ultrassonico iniciado");
  Serial.println("TRIG: GPIO10 | ECHO: GPIO11");
}

void loop() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  const uint32_t duracaoUs =
      pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);

  if (duracaoUs == 0) {
    Serial.println("Sem eco");
  } else {
    const float distanciaCm = duracaoUs * 0.0343f / 2.0f;
    Serial.print("Distancia: ");
    Serial.print(distanciaCm, 1);
    Serial.println(" cm");
  }

  delay(INTERVAL_MS);
}
