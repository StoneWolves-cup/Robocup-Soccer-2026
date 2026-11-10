#include <Arduino.h>

#define RX_PIN 18
#define TX_PIN 17

namespace {
constexpr uint32_t UART_TIMEOUT_MS = 2500;
uint32_t ultimoRecebimentoMs = 0;
bool comunicacaoAtiva = false;
}

void setup() {
  neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);

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

    ultimoRecebimentoMs = millis();
    if (!comunicacaoAtiva) {
      neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);
      comunicacaoAtiva = true;
    }
  }

  if (comunicacaoAtiva &&
      (uint32_t)(millis() - ultimoRecebimentoMs) > UART_TIMEOUT_MS) {
    neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);
    comunicacaoAtiva = false;
  }
}
