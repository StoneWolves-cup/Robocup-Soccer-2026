#include <Arduino.h>
#include <IRProtocol.h>
#include <string.h>

namespace {
constexpr uint32_t UART_BAUD = 115200;
constexpr int UART_RX_PIN = 18;
constexpr int UART_TX_PIN = 17;
constexpr uint32_t TIMEOUT_COMUNICACAO_MS = 1000;

uint8_t bufferUART[sizeof(IRActivityPacket)] = {};
size_t bytesNoBuffer = 0;
uint32_t bytesRecebidos = 0;
uint32_t pacotesRecebidos = 0;
uint32_t ultimoDiagnostico = 0;
uint32_t ultimaRecepcaoValidaMs = 0;
bool pacoteValidoRecebido = false;
bool comunicacaoEstavaAtiva = false;
bool falhaComunicacaoInformada = false;

void imprimirPacote(const IRActivityPacket &pacote) {
  Serial.print("Seq: ");
  Serial.print(pacote.sequence);
  Serial.print(" | IR atividade (%): ");
  for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
    Serial.print(pacote.activity[i]);
    if (i + 1 < IR_SENSOR_COUNT) Serial.print(',');
  }

  Serial.print(" | Bola: ");
  Serial.print(pacote.ballDetected ? "SIM" : "NAO");
  if (pacote.ballDetected) {
    Serial.print(" | Direcao: ");
    Serial.print(pacote.directionTenthsDegrees / 10.0f, 1);
    Serial.print(" graus");
  }
  Serial.print(" | Confianca: ");
  Serial.print(pacote.confidencePercent);
  Serial.println('%');
}

bool pacoteValido(const IRActivityPacket &pacote) {
  return pacote.magic == IR_PACKET_MAGIC &&
         pacote.version == IR_PACKET_VERSION &&
         pacote.checksum == irPacketChecksum(pacote) &&
         pacote.ballDetected <= 1 &&
         pacote.confidencePercent <= 100 &&
         pacote.directionTenthsDegrees >= -1800 &&
         pacote.directionTenthsDegrees <= 1800;
}

void processarUART() {
  while (Serial1.available() > 0) {
    if (bytesNoBuffer == sizeof(bufferUART)) {
      memmove(bufferUART, bufferUART + 1, --bytesNoBuffer);
    }
    bufferUART[bytesNoBuffer++] = static_cast<uint8_t>(Serial1.read());
    bytesRecebidos++;

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
      if (pacoteValido(pacote)) {
        imprimirPacote(pacote);
        pacotesRecebidos++;
        ultimaRecepcaoValidaMs = millis();
        pacoteValidoRecebido = true;
        bytesNoBuffer -= sizeof(pacote);
        memmove(bufferUART, bufferUART + sizeof(pacote), bytesNoBuffer);
      } else {
        memmove(bufferUART, bufferUART + 1, --bytesNoBuffer);
      }
    }
  }
}
} // namespace

void setup() {
  Serial.begin(UART_BAUD);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("Receptor de dados IR pela UART1 iniciado.");
  Serial.println("Conecte TX GPIO17 do emissor ao RX GPIO18 e una os GNDs.");
}

void loop() {
  processarUART();
  const uint32_t agora = millis();
  if ((uint32_t)(agora - ultimoDiagnostico) >= 1000) {
    ultimoDiagnostico = agora;
    const bool comunicacaoAtiva = pacoteValidoRecebido &&
        (uint32_t)(agora - ultimaRecepcaoValidaMs) <= TIMEOUT_COMUNICACAO_MS;
    if (!comunicacaoAtiva) {
      Serial.print("FALHA DE COMUNICACAO UART1: ");
      if (bytesRecebidos == 0) {
        Serial.println(
            "nenhum byte recebido. Verifique TX17->RX18, GND comum e emissor ligado.");
      } else {
        Serial.println(
            "bytes recebidos, mas nenhum pacote IR valido. Verifique baudrate e protocolo.");
      }
      falhaComunicacaoInformada = true;
    } else if (!comunicacaoEstavaAtiva) {
      Serial.println(falhaComunicacaoInformada
          ? "COMUNICACAO UART1 RESTABELECIDA: pacotes IR validos recebidos."
          : "COMUNICACAO UART1 ATIVA: pacotes IR validos recebidos.");
      falhaComunicacaoInformada = false;
    }

    Serial.print("Diagnostico UART1: ");
    Serial.print(bytesRecebidos);
    Serial.print(" bytes e ");
    Serial.print(pacotesRecebidos);
    Serial.println(" pacotes IR validos no ultimo segundo.");
    comunicacaoEstavaAtiva = comunicacaoAtiva;
    bytesRecebidos = 0;
    pacotesRecebidos = 0;
  }
}
