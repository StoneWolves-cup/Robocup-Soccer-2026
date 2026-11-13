#include <Arduino.h>
#include <ConfigPinos.h>

namespace {
constexpr uint8_t PINO_TRIG = ConfigPinos::Teste::Ultrassonicos::TRIG;
constexpr uint8_t PINO_ECHO = ConfigPinos::Teste::Ultrassonicos::ECHO1;
constexpr uint32_t TIMEOUT_ECO_US = 25000;
constexpr float DISTANCIA_MIN_CM = 2.0f;
constexpr float DISTANCIA_MAX_CM = 430.0f;
}

void setup() {
  Serial.begin(115200);
  const uint32_t inicioEsperaSerial = millis();
  while (!Serial && (uint32_t)(millis() - inicioEsperaSerial) < 5000) {
    delay(10);
  }

  pinMode(PINO_TRIG, OUTPUT);
  digitalWrite(PINO_TRIG, LOW);
  pinMode(PINO_ECHO, INPUT);

  Serial.println("Teste de um sensor ultrassonico HC-SR04");
  Serial.print("TRIG: GPIO ");
  Serial.println(PINO_TRIG);
  Serial.print("ECHO: GPIO ");
  Serial.println(PINO_ECHO);
}

void loop() {
  digitalWrite(PINO_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PINO_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PINO_TRIG, LOW);

  const uint32_t duracaoEcoUs = pulseIn(PINO_ECHO, HIGH, TIMEOUT_ECO_US);
  if (duracaoEcoUs == 0) {
    Serial.println("Sem eco: verifique sensor, alimentacao e conexoes.");
  } else {
    const float distanciaCm = duracaoEcoUs * 0.0343f / 2.0f;
    Serial.print("Distancia: ");
    Serial.print(distanciaCm, 1);
    Serial.print(" cm");

    if (distanciaCm < DISTANCIA_MIN_CM || distanciaCm > DISTANCIA_MAX_CM) {
      Serial.print(" (fora da faixa valida de 2 a 430 cm)");
    }
    Serial.println();
  }

  delay(250);
}
