#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <TSSP.h>
#include <IRProtocol.h>
#include <BNO055.h>
#include <Ultrassonicos.h>
#include <math.h>
#include <string.h>

namespace {
constexpr uint8_t CANAL_ESPNOW = 1;
constexpr uint32_t INTERVALO_ENVIO_MS = 20;
constexpr uint32_t INTERVALO_SERIAL_MS = 100;
constexpr uint32_t PERIODO_BNO_MS = 10;
constexpr uint32_t PERIODO_ULTRASSONICOS_MS = 100;
constexpr float LIMIAR_ATIVIDADE = 2.0f;
constexpr float CONFIANCA_MIN = 0.15f;
constexpr float ALPHA_DIRECAO = 0.25f;
const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const float ANGULOS_GRAUS[IR_SENSOR_COUNT] = {
  0.0f, 22.5f, 45.0f, 67.5f, 90.0f, 112.5f, 135.0f, 157.5f,
  180.0f, 202.5f, 225.0f, 247.5f, 270.0f, 292.5f, 315.0f, 337.5f
};

TSSP sensores;
BNO055 bno;
Ultrassonicos ultrassonicos;
uint16_t sequenciaIR = 0;
uint16_t sequenciaBNO = 0;
float vetorXFiltrado = 0.0f;
float vetorYFiltrado = 0.0f;
float somaPesosFiltrada = 0.0f;
bool filtroIniciado = false;
bool bnoDisponivel = false;

void tarefaLeituraUltrassonicos(void *) {
  float distancias[Ultrassonicos::QUANTIDADE] = {};
  bool validas[Ultrassonicos::QUANTIDADE] = {};
  for (;;) {
    ultrassonicos.ler(distancias, validas);
    Serial.print("Ultrassonicos cm:");
    for (uint8_t i = 0; i < Ultrassonicos::QUANTIDADE; ++i) {
      Serial.print(" ECHO");
      Serial.print(Ultrassonicos::PINOS_ECHO[i]);
      Serial.print('=');
      if (validas[i]) Serial.print(distancias[i], 1);
      else Serial.print("sem leitura");
    }
    Serial.println();
    vTaskDelay(pdMS_TO_TICKS(PERIODO_ULTRASSONICOS_MS));
  }
}

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
  IRActivityPacket pacote{};
  pacote.magic = IR_PACKET_MAGIC;
  pacote.version = IR_PACKET_VERSION;
  pacote.sequence = sequenciaIR++;
  for (uint8_t i = 0; i < IR_SENSOR_COUNT; ++i) {
    pacote.activity[i] = sensores.atividadePercentual(i);
  }
  pacote.checksum = irPacketChecksum(pacote);
  const esp_err_t resultado = esp_now_send(BROADCAST_MAC,
      reinterpret_cast<const uint8_t *>(&pacote), sizeof(pacote));
  if (resultado != ESP_OK) Serial.printf("Falha ao enfileirar ESP-NOW: %d\n", resultado);
}

void tarefaLeituraIR(void *) {
  uint32_t ultimoEnvio = millis();
  uint32_t ultimaSerial = millis();
  for (;;) {
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
    vTaskDelay(1);
  }
}

void tarefaLeituraBNO(void *) {
  BNODataPacket pacote{};
  for (;;) {
    if (!bnoDisponivel) {
      // Pinagem I2C do BNO055 no Head; a pinagem TSSP permanece inalterada.
      bnoDisponivel = bno.begin(8, 9);
      if (!bnoDisponivel) {
        Serial.println("BNO055 indisponivel; nova tentativa em 1 s");
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
      Serial.println("BNO055 OK no Head");
    }

    bno.update();
    pacote.magic = BNO_PACKET_MAGIC;
    pacote.version = BNO_PACKET_VERSION;
    pacote.sequence = sequenciaBNO++;
    pacote.headingDegrees = bno.getHeading();
    pacote.gyroZDegreesPerSecond = bno.getGyroZ();
    pacote.checksum = bnoPacketChecksum(pacote);
    const esp_err_t resultado = esp_now_send(BROADCAST_MAC,
        reinterpret_cast<const uint8_t *>(&pacote), sizeof(pacote));
    if (resultado != ESP_OK) Serial.printf("Falha ao enfileirar BNO ESP-NOW: %d\n", resultado);
    vTaskDelay(pdMS_TO_TICKS(PERIODO_BNO_MS));
  }
}
} // namespace

void setup() {
  Serial.begin(115200);
  sensores.begin();
  ultrassonicos.begin();
  if (!iniciarEspNow()) {
    Serial.println("ERRO: nao foi possivel iniciar ESP-NOW; reiniciando em 2 s");
    delay(2000);
    ESP.restart();
  }
  Serial.printf("Leitor IR pronto | MAC=%s | canal=%u\n",
                WiFi.macAddress().c_str(), CANAL_ESPNOW);
  xTaskCreatePinnedToCore(tarefaLeituraIR, "LeituraIR", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(tarefaLeituraBNO, "LeituraBNO", 4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(tarefaLeituraUltrassonicos, "LeituraUltrassonicos",
                          3072, nullptr, 1, nullptr, 1);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
