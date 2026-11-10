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

// Quadro UART: atividade LOW percentual, direcao e confianca calculadas
// pelo mesmo algoritmo do teste TSSP de 16 sensores.
#pragma pack(push, 1)
struct IRActivityPacket {
  uint16_t magic;
  uint8_t version;
  uint16_t sequence;
  uint8_t activity[IR_SENSOR_COUNT];
  uint8_t reserved;
  int16_t directionTenthsDegrees;
  uint8_t confidencePercent;
  uint8_t ballDetected;
  uint8_t checksum;
};

// Orientacao e velocidade angular do BNO055. Transmitido em um quadro
// independente do anel infravermelho para cada task publicar sua leitura.
struct BNODataPacket {
  uint16_t magic;
  uint8_t version;
  uint16_t sequence;
  uint8_t reserved[3]; // Mantem os valores float alinhados em 4 bytes.
  float headingDegrees;
  float gyroZDegreesPerSecond;
  uint8_t checksum;
};

// Distancias dos quatro sensores ultrassonicos e mascara de leituras validas.
struct UltraDataPacket {
  uint16_t magic;
  uint8_t version;
  uint16_t sequence;
  uint8_t reserved[3];
  float distanceCm[4];
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
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&packet);
  uint8_t checksum = 0;
  for (size_t i = 0; i < sizeof(IRActivityPacket) - 1; ++i) checksum ^= bytes[i];
  return checksum;
}

inline uint8_t bnoPacketChecksum(const BNODataPacket &packet) {
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
