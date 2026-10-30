#ifndef IR_PROTOCOL_H
#define IR_PROTOCOL_H

#include <Arduino.h>

constexpr uint16_t IR_PACKET_MAGIC = 0x4952;
constexpr uint8_t IR_PACKET_VERSION = 1;
constexpr uint8_t IR_SENSOR_COUNT = 16;

// Payload curto de ESP-NOW: atividade LOW percentual por sensor (0..100).
// O processamento angular ocorre somente no ESP32-S3 controlador.
#pragma pack(push, 1)
struct IRActivityPacket {
  uint16_t magic;
  uint8_t version;
  uint16_t sequence;
  uint8_t activity[IR_SENSOR_COUNT];
  uint8_t checksum;
};
#pragma pack(pop)

inline uint8_t irPacketChecksum(const IRActivityPacket &packet) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&packet);
  uint8_t checksum = 0;
  for (size_t i = 0; i < sizeof(IRActivityPacket) - 1; ++i) checksum ^= bytes[i];
  return checksum;
}

#endif
