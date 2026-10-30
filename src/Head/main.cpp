#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <TSSP.h>
#include <IRProtocol.h>

namespace {
constexpr uint8_t CANAL_ESPNOW = 1;
constexpr uint32_t INTERVALO_ENVIO_MS = 20;
const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

TSSP sensores;
IRActivityPacket pacote{};
uint16_t sequencia = 0;
uint32_t ultimoEnvio = 0;

bool iniciarEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_wifi_set_channel(CANAL_ESPNOW, WIFI_SECOND_CHAN_NONE) != ESP_OK) return false;
  if (esp_now_init() != ESP_OK) return false;

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, BROADCAST_MAC, sizeof(BROADCAST_MAC));
  peer.channel = CANAL_ESPNOW;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) return false;
  return true;
}

void transmitirAtividades() {
  pacote.magic = IR_PACKET_MAGIC;
  pacote.version = IR_PACKET_VERSION;
  pacote.sequence = sequencia++;
  for (uint8_t i = 0; i < IR_SENSOR_COUNT; ++i) {
    pacote.activity[i] = sensores.atividadePercentual(i);
  }
  pacote.checksum = irPacketChecksum(pacote);
  const esp_err_t resultado = esp_now_send(BROADCAST_MAC,
      reinterpret_cast<const uint8_t *>(&pacote), sizeof(pacote));
  if (resultado != ESP_OK) Serial.printf("Falha ao enfileirar ESP-NOW: %d\n", resultado);
}
} // namespace

void setup() {
  Serial.begin(115200);
  sensores.begin();
  if (!iniciarEspNow()) {
    Serial.println("ERRO: nao foi possivel iniciar ESP-NOW; reiniciando em 2 s");
    delay(2000);
    ESP.restart();
  }
  Serial.printf("Leitor IR pronto | MAC=%s | canal=%u\n",
                WiFi.macAddress().c_str(), CANAL_ESPNOW);
  ultimoEnvio = millis();
}

void loop() {
  // Mantem a aquisicao frequente e envia o quadro mais recente a 50 Hz.
  sensores.atualizar();
  const uint32_t agora = millis();
  if ((uint32_t)(agora - ultimoEnvio) >= INTERVALO_ENVIO_MS) {
    ultimoEnvio = agora;
    transmitirAtividades();
  }
}
