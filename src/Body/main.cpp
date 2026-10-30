#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <math.h>
#include <string.h>
#include <IRProtocol.h>
#include <Omni.h>
#include <Motores.h>

namespace {
constexpr uint8_t CANAL_ESPNOW = 1;
constexpr uint32_t TIMEOUT_PACOTE_MS = 250;
constexpr uint32_t INTERVALO_TELEMETRIA_MS = 150;
constexpr float LIMIAR_ATIVIDADE = 2.0f;
constexpr float CONFIANCA_MIN = 0.15f;
constexpr float ALPHA_FILTRO = 0.25f;
constexpr float VELOCIDADE_SEGUIR = 0.65f;
constexpr float ROTACAO_BUSCA = 0.20f;

// Ordem angular igual ao anel do TSSP.h: 0 frente, positivo para esquerda.
const float ANGULOS_GRAUS[IR_SENSOR_COUNT] = {
  0.0f, 22.5f, 45.0f, 67.5f, 90.0f, 112.5f, 135.0f, 157.5f,
  180.0f, 202.5f, 225.0f, 247.5f, 270.0f, 292.5f, 315.0f, 337.5f
};

struct QuadroRecebido {
  IRActivityPacket pacote;
  uint32_t recebidoEmMs;
  bool disponivel;
};

portMUX_TYPE travaQuadro = portMUX_INITIALIZER_UNLOCKED;
volatile QuadroRecebido quadroRecebido{};
uint16_t ultimaSequencia = 0;
bool sequenciaIniciada = false;
IRActivityPacket ultimoPacote{};
uint32_t ultimoRecebimentoMs = 0;
bool pacoteInicialRecebido = false;
uint32_t ultimaTelemetria = 0;
float vetorXFiltrado = 0.0f; // lateral: esquerda positiva
float vetorYFiltrado = 0.0f; // frente positiva
float somaPesosFiltrada = 0.0f;
bool filtroIniciado = false;

Motores motores;
Omni omni;

void receberPacote(const uint8_t *dados, int tamanho) {
  if (tamanho != (int)sizeof(IRActivityPacket)) return;
  IRActivityPacket copia;
  memcpy(&copia, dados, sizeof(copia));
  if (copia.magic != IR_PACKET_MAGIC || copia.version != IR_PACKET_VERSION ||
      copia.checksum != irPacketChecksum(copia)) return;

  portENTER_CRITICAL(&travaQuadro);
  memcpy((void *)&quadroRecebido.pacote, &copia, sizeof(copia));
  quadroRecebido.recebidoEmMs = millis();
  quadroRecebido.disponivel = true;
  portEXIT_CRITICAL(&travaQuadro);
}

#if ESP_IDF_VERSION_MAJOR >= 5
void callbackRecepcao(const esp_now_recv_info_t *, const uint8_t *dados, int tamanho) {
  receberPacote(dados, tamanho);
}
#else
void callbackRecepcao(const uint8_t *, const uint8_t *dados, int tamanho) {
  receberPacote(dados, tamanho);
}
#endif

bool iniciarEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_wifi_set_channel(CANAL_ESPNOW, WIFI_SECOND_CHAN_NONE) != ESP_OK) return false;
  if (esp_now_init() != ESP_OK) return false;
  return esp_now_register_recv_cb(callbackRecepcao) == ESP_OK;
}

bool obterQuadro(IRActivityPacket &destino, uint32_t &recebidoEmMs) {
  bool disponivel;
  portENTER_CRITICAL(&travaQuadro);
  disponivel = quadroRecebido.disponivel;
  if (disponivel) {
    memcpy(&destino, (const void *)&quadroRecebido.pacote, sizeof(destino));
    recebidoEmMs = quadroRecebido.recebidoEmMs;
    quadroRecebido.disponivel = false;
  }
  portEXIT_CRITICAL(&travaQuadro);
  return disponivel;
}

void aplicarRodas(float x, float y, float rotacao) {
  const ComandosMotores comando = omni.calcular(x, y, rotacao);
  motores.definir(comando.m1, comando.m2, comando.m3, comando.m4);
}

bool estimarDirecao(const IRActivityPacket &pacote, float &anguloGraus,
                    float &confianca, float &atividadeTotal) {
  float x = 0.0f, y = 0.0f, somaPesos = 0.0f;
  atividadeTotal = 0.0f;
  for (uint8_t i = 0; i < IR_SENSOR_COUNT; ++i) {
    const float atividade = pacote.activity[i];
    atividadeTotal += atividade;
    const float peso = fmaxf(0.0f, atividade - LIMIAR_ATIVIDADE);
    const float rad = ANGULOS_GRAUS[i] * DEG_TO_RAD;
    // Conversao para eixos de movimento: x lateral, y para frente.
    x += peso * sinf(rad);
    y += peso * cosf(rad);
    somaPesos += peso;
  }

  if (somaPesos <= 0.0f) {
    confianca = 0.0f;
    return false;
  }

  if (!filtroIniciado) {
    vetorXFiltrado = x;
    vetorYFiltrado = y;
    somaPesosFiltrada = somaPesos;
    filtroIniciado = true;
  } else {
    vetorXFiltrado = ALPHA_FILTRO * x + (1.0f - ALPHA_FILTRO) * vetorXFiltrado;
    vetorYFiltrado = ALPHA_FILTRO * y + (1.0f - ALPHA_FILTRO) * vetorYFiltrado;
    somaPesosFiltrada = ALPHA_FILTRO * somaPesos + (1.0f - ALPHA_FILTRO) * somaPesosFiltrada;
  }

  confianca = sqrtf(vetorXFiltrado * vetorXFiltrado + vetorYFiltrado * vetorYFiltrado)
              / somaPesosFiltrada;
  if (confianca < CONFIANCA_MIN) return false;

  anguloGraus = atan2f(vetorXFiltrado, vetorYFiltrado) * RAD_TO_DEG;
  if (anguloGraus < 0.0f) anguloGraus += 360.0f;
  return true;
}

void decidirMovimento(bool bolaDetectada, float anguloGraus) {
  if (!bolaDetectada) {
    aplicarRodas(0.0f, 0.0f, ROTACAO_BUSCA);
    return;
  }
  const float angulo = anguloGraus * DEG_TO_RAD;
  const float x = VELOCIDADE_SEGUIR * sinf(angulo);
  const float y = VELOCIDADE_SEGUIR * cosf(angulo);
  aplicarRodas(x, y, 0.0f);
}

void imprimirTelemetria(bool conectado, bool bola, float angulo,
                        float confianca, float atividadeMedia) {
  Serial.printf("link=%s bola=%s direcao=%.1f graus confianca=%.2f atividade_media=%.1f\n",
      conectado ? "ok" : "perdido", bola ? "sim" : "nao", angulo,
      confianca, atividadeMedia);
}
} // namespace

void setup() {
  Serial.begin(115200);
  motores.begin();
  motores.parar();

  if (!iniciarEspNow()) {
    Serial.println("ERRO ao iniciar ESP-NOW; motores permanecem parados");
    while (true) delay(1000);
  }
  Serial.printf("Controlador pronto | MAC=%s | canal=%u\n",
                WiFi.macAddress().c_str(), CANAL_ESPNOW);
}

void loop() {
  IRActivityPacket pacote{};
  uint32_t recebidoEmMs = 0;
  const uint32_t agora = millis();

  const bool chegouPacote = obterQuadro(pacote, recebidoEmMs);
  bool quadroNovo = chegouPacote;
  if (chegouPacote) {
    // Descarta pacotes repetidos/antigos, inclusive quando chegam fora de ordem.
    if (sequenciaIniciada && (int16_t)(pacote.sequence - ultimaSequencia) <= 0) {
      quadroNovo = false;
    } else {
      ultimaSequencia = pacote.sequence;
      sequenciaIniciada = true;
      ultimoPacote = pacote;
      ultimoRecebimentoMs = recebidoEmMs;
      pacoteInicialRecebido = true;
    }
  }

  if (!pacoteInicialRecebido ||
      (uint32_t)(agora - ultimoRecebimentoMs) > TIMEOUT_PACOTE_MS) {
    motores.parar(); // falha segura: perda de comunicacao interrompe movimento
    return;
  }
  if (!quadroNovo) return; // mantem o ultimo comando ate chegar novo quadro

  float angulo = 0.0f, confianca = 0.0f, atividadeTotal = 0.0f;
  const bool bola = estimarDirecao(ultimoPacote, angulo, confianca, atividadeTotal);
  decidirMovimento(bola, angulo);

  if ((uint32_t)(agora - ultimaTelemetria) >= INTERVALO_TELEMETRIA_MS) {
    ultimaTelemetria = agora;
    imprimirTelemetria(true, bola, angulo, confianca, atividadeTotal / IR_SENSOR_COUNT);
  }
}
