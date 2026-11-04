#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <TSSP.h>
#include <IRProtocol.h>
#include <math.h>

namespace {
constexpr uint8_t CANAL_ESPNOW = 1;
constexpr uint32_t INTERVALO_ENVIO_MS = 20;
constexpr uint32_t INTERVALO_SERIAL_MS = 100;
constexpr float LIMIAR_ATIVIDADE = 2.0f;
constexpr float CONFIANCA_MIN = 0.15f;
constexpr float ALPHA_DIRECAO = 0.25f;
const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const float ANGULOS_GRAUS[IR_SENSOR_COUNT] = {
  0.0f, 22.5f, 45.0f, 67.5f, 90.0f, 112.5f, 135.0f, 157.5f,
  180.0f, 202.5f, 225.0f, 247.5f, 270.0f, 292.5f, 315.0f, 337.5f
};

TSSP sensores;
IRActivityPacket pacote{};
uint16_t sequencia = 0;
uint32_t ultimoEnvio = 0;
uint32_t ultimaSerial = 0;
float vetorXFiltrado = 0.0f;
float vetorYFiltrado = 0.0f;
float somaPesosFiltrada = 0.0f;
bool filtroIniciado = false;

void imprimirDadosSensores() {
  float x = 0.0f, y = 0.0f, somaPesos = 0.0f;
  Serial.print("LOW%: ");
  for (uint8_t i = 0; i < IR_SENSOR_COUNT; ++i) {
    const float atividade = sensores.atividadePercentual(i);
    Serial.print(atividade, 1);
    if (i < IR_SENSOR_COUNT - 1) Serial.print(',');

    const float peso = fmaxf(0.0f, atividade - LIMIAR_ATIVIDADE);
    const float rad = ANGULOS_GRAUS[i] * DEG_TO_RAD;
    x += peso * cosf(rad);
    y += peso * sinf(rad);
    somaPesos += peso;
  }

  if (!filtroIniciado) {
    vetorXFiltrado = x;
    vetorYFiltrado = y;
    somaPesosFiltrada = somaPesos;
    filtroIniciado = true;
  } else {
        vetorXFiltrado = ALPHA_DIRECAO * x
                   + (1.0f - ALPHA_DIRECAO) * vetorXFiltrado;
    vetorYFiltrado = ALPHA_DIRECAO * y
                   + (1.0f - ALPHA_DIRECAO) * vetorYFiltrado;
    somaPesosFiltrada = ALPHA_DIRECAO * somaPesos
                      + (1.0f - ALPHA_DIRECAO) * somaPesosFiltrada;
  }

  float confianca = 0.0f;
  float direcao = 0.0f;
  const bool atividadeValida = somaPesos > 0.0f && somaPesosFiltrada > 0.0f;
  if (atividadeValida) {
    confianca = sqrtf(vetorXFiltrado * vetorXFiltrado + vetorYFiltrado * vetorYFiltrado)
                / somaPesosFiltrada;
    direcao = atan2f(vetorYFiltrado, vetorXFiltrado) * RAD_TO_DEG;
    if (direcao < 0.0f) direcao += 360.0f;
    if (direcao > 180.0f) direcao -= 360.0f;
  }
  const bool bolaDetectada = atividadeValida && confianca >= CONFIANCA_MIN;

  Serial.print(" | Bola: ");
  Serial.print(bolaDetectada ? "SIM" : "NAO");
  if (bolaDetectada) {
    Serial.print(" | Direcao: ");
    Serial.print(direcao, 1);
    Serial.print(" graus");
  }
  Serial.print(" | Confianca: ");
  Serial.println(confianca, 2);
}

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
  if ((uint32_t)(agora - ultimaSerial) >= INTERVALO_SERIAL_MS) {
    ultimaSerial = agora;
    imprimirDadosSensores();
  }
}
