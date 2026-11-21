#include <Arduino.h>
#include <BatteryEstimator.h>
#include <ConfigPinos.h>
#include <IRProtocol.h>
#include <Kicker.h>
#include <KickerLogica.h>
#include <math.h>
#include <string.h>

namespace {
constexpr uint32_t TIMEOUT_ULTRA_MS = 350;
constexpr uint32_t INTERVALO_DIAGNOSTICO_MS = 1000;
constexpr size_t TAMANHO_BUFFER =
    sizeof(IRActivityPacket) > sizeof(UltraDataPacket)
        ? (sizeof(IRActivityPacket) > sizeof(BNODataPacket)
               ? sizeof(IRActivityPacket) : sizeof(BNODataPacket))
        : (sizeof(UltraDataPacket) > sizeof(BNODataPacket)
               ? sizeof(UltraDataPacket) : sizeof(BNODataPacket));

BatteryEstimator bateria;
UltraDataPacket ultimoUltra{};
uint8_t buffer[TAMANHO_BUFFER]{};
size_t bytesNoBuffer = 0;
uint32_t ultimoUltraMs = 0;
uint32_t ultimaBateriaMs = 0;
uint32_t ultimoDiagnosticoMs = 0;
uint32_t quadrosInvalidos = 0;
bool ultraRecebido = false;

void descartarBytes(size_t quantidade) {
  bytesNoBuffer -= quantidade;
  memmove(buffer, buffer + quantidade, bytesNoBuffer);
}

bool validarQuadro(uint16_t magic) {
  if (magic == IR_PACKET_MAGIC) {
    IRActivityPacket pacote;
    memcpy(&pacote, buffer, sizeof(pacote));
    return pacote.version == IR_PACKET_VERSION &&
           pacote.checksum == irPacketChecksum(pacote);
  }
  if (magic == BNO_PACKET_MAGIC) {
    BNODataPacket pacote;
    memcpy(&pacote, buffer, sizeof(pacote));
    return pacote.version == BNO_PACKET_VERSION &&
           pacote.checksum == bnoPacketChecksum(pacote);
  }

  UltraDataPacket pacote;
  memcpy(&pacote, buffer, sizeof(pacote));
  if (pacote.version != ULTRA_PACKET_VERSION ||
      pacote.checksum != ultraPacketChecksum(pacote) ||
      (pacote.validMask & ~((1U << ULTRA_SENSOR_COUNT) - 1U)) != 0) {
    return false;
  }
  for (uint8_t i = 0; i < ULTRA_SENSOR_COUNT; ++i) {
    if ((pacote.validMask & (1U << i)) &&
        !KickerLogica::distanciaValida(pacote.distanceCm[i])) return false;
  }

  const uint32_t agora = millis();
  if (!ultraRecebido || (uint32_t)(agora - ultimoUltraMs) > TIMEOUT_ULTRA_MS ||
      (int16_t)(pacote.sequence - ultimoUltra.sequence) > 0) {
    ultimoUltra = pacote;
    ultimoUltraMs = agora;
    ultraRecebido = true;
  }
  return true;
}

void processarUART() {
  while (Serial1.available() > 0) {
    const int leitura = Serial1.read();
    if (leitura < 0) break;
    if (bytesNoBuffer == sizeof(buffer)) descartarBytes(1);
    buffer[bytesNoBuffer++] = static_cast<uint8_t>(leitura);

    while (bytesNoBuffer >= sizeof(uint16_t)) {
      uint16_t magic;
      memcpy(&magic, buffer, sizeof(magic));
      size_t tamanho = 0;
      if (magic == IR_PACKET_MAGIC) tamanho = sizeof(IRActivityPacket);
      else if (magic == BNO_PACKET_MAGIC) tamanho = sizeof(BNODataPacket);
      else if (magic == ULTRA_PACKET_MAGIC) tamanho = sizeof(UltraDataPacket);
      else {
        descartarBytes(1);
        continue;
      }
      if (bytesNoBuffer < tamanho) break;
      if (validarQuadro(magic)) {
        descartarBytes(tamanho);
      } else {
        ++quadrosInvalidos;
        descartarBytes(1);
      }
    }
  }
}
}  // namespace

void setup() {
  Serial.begin(460800);
  Serial1.setRxBufferSize(4096);
  Serial1.begin(HEAD_BODY_UART_BAUD, SERIAL_8N1,
                ConfigPinos::Geral::UART1::RX, ConfigPinos::Geral::UART1::TX);
  Kicker::begin();
  bateria.begin();
  bateria.update(0);
  ultimaBateriaMs = millis();
  Serial.println("Teste de bancada do kicker iniciado.");
  Serial.println("ATENCAO: sem IHM ou estrategias; o teste pode chutar sozinho.");
  Serial.println("Aguardando leituras ultrassonicas validas do Head.");
}

void loop() {
  processarUART();
  const uint32_t agora = millis();
  if ((uint32_t)(agora - ultimaBateriaMs) >= UPDATE_PERIOD_MS) {
    bateria.update(agora - ultimaBateriaMs);
    ultimaBateriaMs = agora;
  }

  const bool ultraValido = ultraRecebido &&
      (uint32_t)(agora - ultimoUltraMs) <= TIMEOUT_ULTRA_MS;
  const bool bateriaValida = bateria.hasReading() &&
      isfinite(bateria.getVoltage()) && isfinite(bateria.getPercentage());
  Kicker::atualizar(true, ultraValido, bateriaValida, bateria.getPercentage(),
                    ultraValido, ultimoUltra.distanceCm, ultimoUltra.validMask);

  if ((uint32_t)(agora - ultimoDiagnosticoMs) >= INTERVALO_DIAGNOSTICO_MS) {
    ultimoDiagnosticoMs = agora;
    Serial.printf("Ultra %s: direita=%.1f frente=%.1f esquerda=%.1f "
                  "mascara=0x%02X | quadros invalidos=%lu\n",
                  ultraValido ? "recente" : "ausente ou expirado",
                  ultimoUltra.distanceCm[ULTRA_INDEX_RIGHT],
                  ultimoUltra.distanceCm[ULTRA_INDEX_FRONT],
                  ultimoUltra.distanceCm[ULTRA_INDEX_LEFT],
                  ultimoUltra.validMask,
                  static_cast<unsigned long>(quadrosInvalidos));
    quadrosInvalidos = 0;
  }
}
