#include <Arduino.h>

namespace {
constexpr uint32_t USB_SERIAL_BAUD = 460800;
constexpr uint32_t UART_BAUD = 115200;
constexpr int UART_RX_PIN = 18;
constexpr int UART_TX_PIN = 17;
}

void setup() {
  Serial.begin(USB_SERIAL_BAUD);

  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  Serial.println("ESP32-S3 Transmissor iniciado!");
}

void loop() {
  Serial1.println("Ola, ESP32-S3 B!");

  Serial.println("Mensagem enviada!");

  delay(1000);
}