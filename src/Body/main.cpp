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
#include <BNO055.h>
#include <FuzzyPID.h>

namespace {
constexpr uint8_t CANAL_ESPNOW = 1;
constexpr uint32_t TIMEOUT_PACOTE_MS = 250;
constexpr uint32_t PERIODO_BNO_US = 10000;
constexpr float LIMIAR_ATIVIDADE = 2.0f;
constexpr float CONFIANCA_MIN = 0.15f;
constexpr float ALPHA_DIRECAO = 0.25f;
constexpr float VELOCIDADE_SEGUIR = 1.0f;
// Referencia local capturada na inicializacao; evita exigir alinhamento ao norte.
float rumoDesejadoGraus = 0.0f;
constexpr float CORRECAO_MAX_RODAS = 0.10f;
constexpr float GANHO_P = 0.0015f;
constexpr float GANHO_I = 0.00002f;
constexpr float GANHO_D = 0.0016f;

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
uint32_t ultimaLeituraBNOUs = 0;
float correcaoRotacao = 0.0f;
FuzzyPID fuzzyPidRumo(GANHO_P, GANHO_I, GANHO_D, -CORRECAO_MAX_RODAS, CORRECAO_MAX_RODAS);
float vetorXFiltrado = 0.0f; // lateral: esquerda positiva
float vetorYFiltrado = 0.0f; // frente positiva
float somaPesosFiltrada = 0.0f;
bool filtroIniciado = false;
bool bnoDisponivel = false;
bool ultimaBolaDetectada = false;
float ultimaDirecaoBola = 0.0f;

Motores motores;
Omni omni;
BNO055 bno;

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

void aplicarRodas(float x, float y, float correcao) {
  // A rotacao entra junto com a translacao na cinemática das rodas, para
  // corrigir o rumo no mesmo comando que movimenta o robo.
  const ComandosMotores comando = omni.calcular(x, y, correcao);
  motores.definir(comando.m1, comando.m2, comando.m3, comando.m4);
}

float erroAngular(float desejado, float atual) {
  float erro = desejado - atual;
  while (erro > 180.0f) erro -= 360.0f;
  while (erro < -180.0f) erro += 360.0f;
  return erro;
}

bool atualizarCorrecaoBNO() {
  if (!bnoDisponivel) return false;

  const uint32_t agoraUs = micros();
  const uint32_t decorridoUs = agoraUs - ultimaLeituraBNOUs;
  if (decorridoUs < PERIODO_BNO_US) return false;

  bno.update();
  const float erro = erroAngular(rumoDesejadoGraus, bno.getHeading());
  const float dt = decorridoUs * 1.0e-6f;
  // Fuzzy PID controla o rumo usando erro angular circular e giro Z atualizado.
  correcaoRotacao = fuzzyPidRumo.calculate(erro, bno.getGyroZ(), dt);
  ultimaLeituraBNOUs = agoraUs;
  return true;
}

bool estimarDirecao(const IRActivityPacket &pacote, float &anguloGraus,
                    float &confianca) {
  float x = 0.0f, y = 0.0f, somaPesos = 0.0f;
  for (uint8_t i = 0; i < IR_SENSOR_COUNT; ++i) {
    const float atividade = pacote.activity[i];
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
    vetorXFiltrado = ALPHA_DIRECAO * x
                   + (1.0f - ALPHA_DIRECAO) * vetorXFiltrado;
    vetorYFiltrado = ALPHA_DIRECAO * y
                   + (1.0f - ALPHA_DIRECAO) * vetorYFiltrado;
    somaPesosFiltrada = ALPHA_DIRECAO * somaPesos
                      + (1.0f - ALPHA_DIRECAO) * somaPesosFiltrada;
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
    // Mantem a busca ativa quando os sensores ainda nao tem direcao confiavel.
    aplicarRodas(0.0f, -VELOCIDADE_SEGUIR, correcaoRotacao);
    return;
  }
  const float angulo = anguloGraus * DEG_TO_RAD;
  const float x = VELOCIDADE_SEGUIR * sinf(angulo);
  const float y = VELOCIDADE_SEGUIR * cosf(angulo);
  // Soma a correcao de rumo a translacao omni; assim o robo continua
  // seguindo a bola enquanto corrige qualquer giro involuntario.
  aplicarRodas(x, y, correcaoRotacao);
}

} // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Inicializando Body...");

  motores.begin();
  motores.parar();

  bnoDisponivel = bno.begin();
  if (bnoDisponivel) {
    bno.update();
    // Usa imediatamente a primeira orientacao lida pelo BNO055 como referencia.
    // A correcao angular circular mantem esse rumo mesmo enquanto o magnetometro
    // ainda calibra; nao ha espera de calibracao antes de iniciar o robo.
    rumoDesejadoGraus = bno.getHeading();
    fuzzyPidRumo.reset();
    Serial.print("BNO055 OK, heading inicial: ");
    Serial.println(rumoDesejadoGraus);
  } else {
    Serial.println("ERRO: BNO055 nao encontrado em SDA=21, SCL=22, endereco 0x28");
  }
  ultimaLeituraBNOUs = micros();

  if (!iniciarEspNow()) {
    Serial.println("ERRO: falha ao inicializar ESP-NOW");
    while (true) delay(1000);
  }
  Serial.println("ESP-NOW OK");
}

void loop() {
  const bool rumoAtualizado = atualizarCorrecaoBNO();

  IRActivityPacket pacote{};
  uint32_t recebidoEmMs = 0;
  const bool chegouPacote = obterQuadro(pacote, recebidoEmMs);
  const uint32_t agora = millis();
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

  if (quadroNovo) {
    float confianca = 0.0f;
    ultimaBolaDetectada = estimarDirecao(ultimoPacote, ultimaDirecaoBola,
                                         confianca);
  }

  // Atualiza a rotacao pelo BNO entre os pacotes IR tambem, sem interromper
  // a translacao estimada no ultimo quadro recebido.
  if (quadroNovo || rumoAtualizado) {
    decidirMovimento(ultimaBolaDetectada, ultimaDirecaoBola);
  }

}
