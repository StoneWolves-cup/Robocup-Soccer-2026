#include <Arduino.h>
#include <Ultrassonicos.h>

Ultrassonicos ultrassonicos;

void setup() {
  Serial.begin(115200);
  ultrassonicos.begin();

  Serial.println("Teste de 4 sensores HC-SR04 com TRIG compartilhado");
  Serial.print("TRIG: GPIO ");
  Serial.println(Ultrassonicos::PINO_TRIG);
  for (uint8_t i = 0; i < Ultrassonicos::QUANTIDADE; ++i) {
    Serial.print("ECHO U");
    Serial.print(i + 1);
    Serial.print(": GPIO ");
    Serial.println(Ultrassonicos::PINOS_ECHO[i]);
  }
}

void loop() {
  float distanciasCm[Ultrassonicos::QUANTIDADE] = {};
  bool leiturasValidas[Ultrassonicos::QUANTIDADE] = {};

  ultrassonicos.ler(distanciasCm, leiturasValidas);

  for (uint8_t i = 0; i < Ultrassonicos::QUANTIDADE; ++i) {
    Serial.print("U");
    Serial.print(i + 1);
    Serial.print(" (ECHO GPIO ");
    Serial.print(Ultrassonicos::PINOS_ECHO[i]);
    Serial.print("): ");
    if (leiturasValidas[i]) {
      Serial.print(distanciasCm[i], 1);
      Serial.println(" cm");
    } else {
      Serial.println("sem eco / leitura invalida");
    }
  }

  Serial.println("------------------------------");
  delay(200);
}
