#include <Arduino.h>

#define RX_PIN 18
#define TX_PIN 17

void setup() {
  Serial.begin(115200);

  // Inicializa UART1
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.println("ESP32-S3 Transmissor iniciado!");
}

void loop() {
  Serial1.println("Ola, ESP32-S3 B!");

  Serial.println("Mensagem enviada!");

  delay(1000);
}