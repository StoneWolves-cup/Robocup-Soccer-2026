#ifndef IR_PROTOCOL_H
#define IR_PROTOCOL_H

#include <Arduino.h>

constexpr uint16_t IR_PACKET_MAGIC = 0x4952;
constexpr uint16_t BNO_PACKET_MAGIC = 0x424E;
constexpr uint16_t ULTRA_PACKET_MAGIC = 0x554C;
constexpr uint16_t ESPNOW_ACK_MAGIC = 0x4B41;
constexpr uint8_t IR_PACKET_VERSION = 2;
constexpr uint8_t BNO_PACKET_VERSION = 1;
constexpr uint8_t ULTRA_PACKET_VERSION = 1;
constexpr uint8_t ESPNOW_ACK_VERSION = 1;
constexpr uint8_t IR_SENSOR_COUNT = 16;
constexpr uint8_t ULTRA_SENSOR_COUNT = 4;
constexpr uint8_t ULTRA_INDEX_RIGHT = 0;  // ECHO1, sensor direito (U1).
constexpr uint8_t ULTRA_INDEX_FRONT = 1;  // ECHO2, sensor frontal (U2).
constexpr uint8_t ULTRA_INDEX_LEFT = 2;   // ECHO3, sensor esquerdo (U3).
constexpr uint8_t ULTRA_INDEX_REAR = 3;   // ECHO4, sensor traseiro (U4).
constexpr uint32_t HEAD_BODY_UART_BAUD = 460800;

// Quadros compactos transmitidos entre Head e Body. Magic/version identificam
// tipo e compatibilidade; sequence detecta repeticao/ordem; checksum XOR
// detecta corrupcao simples, mas nao substitui autenticacao.
#pragma pack(push, 1)
struct IRActivityPacket {
  uint16_t magic;
  uint8_t version;
  uint16_t sequence;  // Incremental modulo 65536 para descartar quadros antigos.
  uint8_t activity[IR_SENSOR_COUNT];  // Percentual LOW, 0-100 por receptor.
  uint8_t reserved;
  int16_t directionTenthsDegrees;  // Graus assinados x10, faixa [-1800, 1800].
  uint8_t confidencePercent;  // Concentracao vetorial, 0-100.
  uint8_t ballDetected;  // Booleano em byte para layout deterministico no UART.
  uint8_t checksum;  // XOR de todos os bytes anteriores.
};

// Orientacao e velocidade angular do BNO055. Transmitido em um quadro
// independente do anel infravermelho para cada task publicar sua leitura.
struct BNODataPacket {
  uint16_t magic;
  uint8_t version;
  uint16_t sequence;
  uint8_t reserved[3]; // Mantem floats alinhados em 4 bytes no layout empacotado.
  float headingDegrees;  // Rumo normalizado em [-180, 180).
  float gyroZDegreesPerSecond;  // Velocidade angular; a unidade faz parte do contrato.
  uint8_t checksum;
};

struct UltraDataPacket {
  uint16_t magic;
  uint8_t version;
  uint16_t sequence;
  uint8_t reserved[3];
  float distanceCm[ULTRA_SENSOR_COUNT];
  uint8_t validMask;
  uint8_t checksum;
};

struct ESPNowAckPacket {
  uint16_t magic;
  uint8_t version;
  uint16_t sequence;
  uint8_t checksum;
};
#pragma pack(pop)

inline uint8_t irPacketChecksum(const IRActivityPacket &packet) {
  // Exclui o ultimo byte, reservado ao proprio checksum do quadro.
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&packet);
  uint8_t checksum = 0;
  for (size_t i = 0; i < sizeof(IRActivityPacket) - 1; ++i) checksum ^= bytes[i];
  return checksum;
}

inline uint8_t bnoPacketChecksum(const BNODataPacket &packet) {
  // Mantem a mesma regra de integridade em ambos os lados do UART.
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&packet);
  uint8_t checksum = 0;
  for (size_t i = 0; i < sizeof(BNODataPacket) - 1; ++i) checksum ^= bytes[i];
  return checksum;
}

inline uint8_t ultraPacketChecksum(const UltraDataPacket &packet) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&packet);
  uint8_t checksum = 0;
  for (size_t i = 0; i < sizeof(UltraDataPacket) - 1; ++i) checksum ^= bytes[i];
  return checksum;
}

inline uint8_t espNowAckChecksum(const ESPNowAckPacket &packet) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&packet);
  uint8_t checksum = 0;
  for (size_t i = 0; i < sizeof(ESPNowAckPacket) - 1; ++i) checksum ^= bytes[i];
  return checksum;
}

#endif
