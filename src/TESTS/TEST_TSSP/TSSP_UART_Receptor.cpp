#include <Arduino.h>
#include <ConfigPinos.h>
#include <IRProtocol.h>
#include <Motores.h>
#include <Omni.h>
#include <math.h>
#include <string.h>

namespace {
constexpr uint32_t UART_BAUD = 460800;
constexpr int UART_RX_PIN = ConfigPinos::Geral::UART1::RX;
constexpr int UART_TX_PIN = ConfigPinos::Geral::UART1::TX;
constexpr uint32_t TIMEOUT_COMUNICACAO_MS = 100;
constexpr uint32_t SERIAL_MS = 100;
constexpr float POTENCIA_MAXIMA = 0.30f;

uint8_t bufferUART[sizeof(IRActivityPacket)] = {};
size_t bytesNoBuffer = 0;
uint32_t ultimaRecepcaoValidaMs = 0;
uint32_t ultimaSerial = 0;
bool pacoteValidoRecebido = false;
bool motoresSeguindoBola = false;

Motores motores;
Omni omni;

bool pacoteValido(const IRActivityPacket &pacote) {
  return pacote.magic == IR_PACKET_MAGIC &&
         pacote.version == IR_PACKET_VERSION &&
         pacote.checksum == irPacketChecksum(pacote) &&
         pacote.ballDetected <= 1 &&
         pacote.confidencePercent <= 100 &&
         pacote.directionTenthsDegrees >= -1800 &&
         pacote.directionTenthsDegrees <= 1800;
}

void pararMotores() {
  motores.parar();
  motoresSeguindoBola = false;
}

void seguirBola(const IRActivityPacket &pacote) {
  if (pacote.ballDetected == 0) {
    pararMotores();
    return;
  }

  const float anguloRad =
      (pacote.directionTenthsDegrees / 10.0f) * DEG_TO_RAD;
  const float potencia =
      POTENCIA_MAXIMA * (pacote.confidencePercent / 100.0f);
  const ComandosMotores comando =
      omni.calcular(sinf(anguloRad), cosf(anguloRad), 0.0f);

  motores.definir(comando.m1 * potencia,
                  comando.m2 * potencia,
                  comando.m3 * potencia,
                  comando.m4 * potencia);
  motoresSeguindoBola = potencia > 0.0f;
}

void processarUART() {
  while (Serial1.available() > 0) {
    if (bytesNoBuffer == sizeof(bufferUART)) {
      memmove(bufferUART, bufferUART + 1, --bytesNoBuffer);
    }
    bufferUART[bytesNoBuffer++] = static_cast<uint8_t>(Serial1.read());

    while (bytesNoBuffer >= sizeof(uint16_t)) {
      uint16_t magic = 0;
      memcpy(&magic, bufferUART, sizeof(magic));
      if (magic != IR_PACKET_MAGIC) {
        memmove(bufferUART, bufferUART + 1, --bytesNoBuffer);
        continue;
      }
      if (bytesNoBuffer < sizeof(IRActivityPacket)) break;

      IRActivityPacket pacote{};
      memcpy(&pacote, bufferUART, sizeof(pacote));
      if (!pacoteValido(pacote)) {
        memmove(bufferUART, bufferUART + 1, --bytesNoBuffer);
        continue;
      }

      ultimaRecepcaoValidaMs = millis();
      pacoteValidoRecebido = true;
      seguirBola(pacote);
      bytesNoBuffer -= sizeof(pacote);
      memmove(bufferUART, bufferUART + sizeof(pacote), bytesNoBuffer);
    }
  }
}

void imprimirDiagnostico() {
  const bool comunicacaoAtiva = pacoteValidoRecebido &&
      (uint32_t)(millis() - ultimaRecepcaoValidaMs) <=
          TIMEOUT_COMUNICACAO_MS;
  Serial.print("UART: ");
  Serial.print(comunicacaoAtiva ? "ATIVA" : "SEM QUADROS");
  Serial.print(" | Movimento: ");
  Serial.println(motoresSeguindoBola ? "SEGUINDO BOLA" : "PARADO");
}
} // namespace

void setup() {
  Serial.begin(115200);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  motores.begin();
  pararMotores();

  Serial.println("Receptor de seguimento TSSP iniciado; sem outros comportamentos.");
  Serial.println(
      "UART1: RX GPIO18 <- TX GPIO17 do emissor; conecte o GND comum.");
}

void loop() {
  processarUART();

  if (!pacoteValidoRecebido ||
      (uint32_t)(millis() - ultimaRecepcaoValidaMs) >
          TIMEOUT_COMUNICACAO_MS) {
    pararMotores();
  }

  const uint32_t agora = millis();
  if ((uint32_t)(agora - ultimaSerial) >= SERIAL_MS) {
    ultimaSerial = agora;
    imprimirDiagnostico();
  }
}
