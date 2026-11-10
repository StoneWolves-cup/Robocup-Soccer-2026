#include <Arduino.h>

#define RX_PIN 18
#define TX_PIN 17

void setup() {
  Serial.begin(115200);

  // Inicializa UART1
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.println("ESP32-S3 Receptor iniciado!");
}

void loop() {
  if (Serial1.available()) {
    String mensagem = Serial1.readStringUntil('\n');

    Serial.print("Recebido: ");
    Serial.println(mensagem);
  }
}