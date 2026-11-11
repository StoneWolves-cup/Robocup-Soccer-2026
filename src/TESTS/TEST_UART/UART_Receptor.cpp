#include <Arduino.h>

namespace {
constexpr uint32_t USB_SERIAL_BAUD = 460800;
constexpr uint32_t UART_BAUD = 115200;
constexpr uint32_t UART_TIMEOUT_MS = 2500;
constexpr int UART_RX_PIN = 18;
constexpr int UART_TX_PIN = 17;
constexpr size_t TAMANHO_BUFFER_MENSAGEM = 64;

char bufferMensagem[TAMANHO_BUFFER_MENSAGEM] = {};
size_t tamanhoMensagem = 0;
uint32_t ultimoRecebimentoMs = 0;
uint32_t inicioAguardandoMs = 0;
bool comunicacaoAtiva = false;
bool falhaInformada = false;
bool descartandoMensagemLonga = false;

void processarUART() {
  while (Serial1.available() > 0) {
    const int byteRecebido = Serial1.read();
    if (byteRecebido < 0) return;

    if (descartandoMensagemLonga) {
      if (byteRecebido == '\n') {
        descartandoMensagemLonga = false;
        Serial.println("Erro: mensagem UART excedeu o buffer e foi descartada.");
      }
      continue;
    }

    if (byteRecebido == '\n') {
      if (tamanhoMensagem > 0 && bufferMensagem[tamanhoMensagem - 1] == '\r') {
        tamanhoMensagem--;
      }
      bufferMensagem[tamanhoMensagem] = '\0';

      if (tamanhoMensagem > 0) {
        Serial.print("Recebido: ");
        Serial.println(bufferMensagem);
        ultimoRecebimentoMs = millis();

        if (!comunicacaoAtiva) {
          neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);
          Serial.println(falhaInformada
              ? "COMUNICACAO UART1 RESTABELECIDA."
              : "COMUNICACAO UART1 ATIVA.");
          comunicacaoAtiva = true;
          falhaInformada = false;
        }
      }

      tamanhoMensagem = 0;
      continue;
    }

    if (tamanhoMensagem < sizeof(bufferMensagem) - 1) {
      bufferMensagem[tamanhoMensagem++] = static_cast<char>(byteRecebido);
    } else {
      tamanhoMensagem = 0;
      descartandoMensagemLonga = true;
    }
  }
}
}

void setup() {
  neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);

  Serial.begin(USB_SERIAL_BAUD);

  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  inicioAguardandoMs = millis();
  Serial.println("ESP32-S3 Receptor iniciado!");
  Serial.println("Aguardando mensagens UART1 a 115200 baud.");
}

void loop() {
  processarUART();

  const uint32_t agoraMs = millis();
  const uint32_t referenciaTimeoutMs =
      comunicacaoAtiva ? ultimoRecebimentoMs : inicioAguardandoMs;
  if (!falhaInformada &&
      (uint32_t)(agoraMs - referenciaTimeoutMs) > UART_TIMEOUT_MS) {
    neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);
    comunicacaoAtiva = false;
    Serial.println(
        "FALHA DE COMUNICACAO UART1: nenhuma linha completa recebida em 2,5 s.");
    falhaInformada = true;
  }
}
