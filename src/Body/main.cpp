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

namespace {
constexpr uint8_t CANAL_ESPNOW = 1;
constexpr uint8_t PINO_START = 18;
constexpr bool USAR_INTERRUPTOR_START = false;
constexpr uint32_t TIMEOUT_PACOTE_MS = 250;
constexpr uint32_t INTERVALO_TELEMETRIA_MS = 150;
constexpr uint32_t PERIODO_BNO_US = 10000;
constexpr float LIMIAR_ATIVIDADE = 2.0f;
constexpr float CONFIANCA_MIN = 0.15f;
constexpr float ALPHA_FILTRO = 0.25f;
constexpr float VELOCIDADE_SEGUIR = 0.65f;
// Referencia local capturada na inicializacao; evita exigir alinhamento ao norte.
float rumoDesejadoGraus = 0.0f;
constexpr float ZONA_MORTA_RUMO_GRAUS = 2.0f;
constexpr float CORRECAO_MAX_RODAS = 0.12f;
constexpr float GANHO_RUMO_RODAS = 0.0025f;
constexpr float AMORTECIMENTO_GIRO = 0.0010f;
constexpr float FILTRO_CORRECAO = 0.10f;

// Manobra autonoma executada uma vez apos o primeiro pacote valido do emissor.
// O trecho reto e seguido por meia volta parametrica, formando o gancho.
constexpr uint32_t TEMPO_HASTE_J_MS = 1200;
constexpr uint32_t TEMPO_CURVA_J_MS = 1800;
constexpr float VELOCIDADE_J = 0.45f;
constexpr float PASSO_ANGULAR_J = 3.14159265f / TEMPO_CURVA_J_MS;

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
uint32_t ultimaLeituraBNOUs = 0;
float correcaoRotacao = 0.0f;
float vetorXFiltrado = 0.0f; // lateral: esquerda positiva
float vetorYFiltrado = 0.0f; // frente positiva
float somaPesosFiltrada = 0.0f;
bool filtroIniciado = false;
bool bnoDisponivel = false;
bool ultimaBolaDetectada = false;
float ultimaDirecaoBola = 0.0f;
float ultimaConfianca = 0.0f;
float ultimaAtividadeMedia = 0.0f;
bool manobraJAtiva = false;
bool manobraJExecutada = false;
uint32_t inicioManobraJMs = 0;

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
  // Primeiro calcula apenas a translacao apontada pelos sensores IR.
  // Depois aplica a correcao diferencial, pouco a pouco, nos quatro motores.
  ComandosMotores comando = omni.calcular(x, y, 0.0f);
  comando.m1 += correcao;
  comando.m2 -= correcao;
  comando.m3 -= correcao;
  comando.m4 += correcao;

  const float maior = fmaxf(fmaxf(fabsf(comando.m1), fabsf(comando.m2)),
                            fmaxf(fabsf(comando.m3), fabsf(comando.m4)));
  if (maior > 1.0f) {
    comando.m1 /= maior;
    comando.m2 /= maior;
    comando.m3 /= maior;
    comando.m4 /= maior;
  }
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
  float erroControle = erro;
  if (fabsf(erroControle) <= ZONA_MORTA_RUMO_GRAUS) erroControle = 0.0f;

  // Correcao proporcional simples e limitada: pequenos erros mudam pouco
  // a velocidade das rodas e o BNO nao consegue comandar um giro brusco.
  float correcaoAlvo = erroControle * GANHO_RUMO_RODAS;
  // Amortece o comando conforme a velocidade de giro medida pelo giroscopio.
  correcaoAlvo -= bno.getGyroZ() * AMORTECIMENTO_GIRO;
  correcaoAlvo = constrain(correcaoAlvo, -CORRECAO_MAX_RODAS, CORRECAO_MAX_RODAS);
  correcaoRotacao += FILTRO_CORRECAO * (correcaoAlvo - correcaoRotacao);
  ultimaLeituraBNOUs = agoraUs;
  return true;
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
    // Recua enquanto o Head nao detecta a bola, mantendo o rumo.
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

bool atualizarManobraJ(uint32_t agoraMs) {
  if (!manobraJAtiva) return false;

  const uint32_t decorrido = agoraMs - inicioManobraJMs;
  if (decorrido < TEMPO_HASTE_J_MS) {
    // A bola esta atras do robo: o primeiro trecho deve sair para tras.
    aplicarRodas(0.0f, -VELOCIDADE_J, correcaoRotacao);
    return true;
  }

  const uint32_t tempoCurva = decorrido - TEMPO_HASTE_J_MS;
  if (tempoCurva < TEMPO_CURVA_J_MS) {
    // A curva x=R(1-cos(t)), y=-R sen(t) comeca para tras,
    // desloca o robo para a esquerda e termina avancando.
    const float theta = PASSO_ANGULAR_J * tempoCurva;
    aplicarRodas(VELOCIDADE_J * sinf(theta),
                 -VELOCIDADE_J * cosf(theta), correcaoRotacao);
    return true;
  }

  manobraJAtiva = false;
  motores.parar();
  return true;
}

void imprimirTelemetria(bool conectado, bool bola, float angulo,
                        float confianca, float atividadeMedia, float rumo,
                        float erro, float correcao, float giroZ,
                        uint8_t calSistema, uint8_t calGiro,
                        uint8_t calAcelerometro, uint8_t calMagnetometro) {
  Serial.printf("link=%s bola=%s IR=%.1f rumo=%.1f alvo=%.1f erro=%.1f gyroZ=%.1f correcao=%.3f cal=%u/%u/%u/%u confianca=%.2f atividade=%.1f\n",
      conectado ? "ok" : "perdido", bola ? "sim" : "nao", angulo,
      rumo, rumoDesejadoGraus, erro, giroZ, correcao,
      calSistema, calGiro, calAcelerometro, calMagnetometro,
      confianca, atividadeMedia);
}
} // namespace

void setup() {
  Serial.begin(115200);
  pinMode(PINO_START, INPUT_PULLDOWN);
  motores.begin();
  motores.parar();

  bnoDisponivel = bno.begin();
  if (!bnoDisponivel) {
    Serial.println("AVISO: BNO055 nao encontrado; seguindo sem correcao de rumo");
  } else {
    // Usa imediatamente a primeira orientacao lida pelo BNO055 como referencia.
    // A correcao angular circular mantem esse rumo mesmo enquanto o magnetometro
    // ainda calibra; nao ha espera de calibracao antes de iniciar o robo.
    rumoDesejadoGraus = bno.getHeading();
    uint8_t calSistema = 0, calGiro = 0, calAcelerometro = 0, calMagnetometro = 0;
    bno.getCalibration(calSistema, calGiro, calAcelerometro, calMagnetometro);
    Serial.printf("BNO055 calibracao sistema/giro/acel/mag=%u/%u/%u/%u; alvo inicial=%.1f graus\n",
                  calSistema, calGiro, calAcelerometro, calMagnetometro,
                  rumoDesejadoGraus);
  }
  ultimaLeituraBNOUs = micros();

  if (!iniciarEspNow()) {
    Serial.println("ERRO ao iniciar ESP-NOW; motores permanecem parados");
    while (true) delay(1000);
  }
  Serial.printf("Controlador pronto | MAC=%s | canal=%u\n",
                WiFi.macAddress().c_str(), CANAL_ESPNOW);
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

  // Inicia a manobra sem comandos externos assim que a comunicacao estiver valida.
  if (!manobraJExecutada) {
    manobraJExecutada = true;
    manobraJAtiva = true;
    inicioManobraJMs = agora;
    Serial.println("Manobra J iniciada automaticamente");
  }

  // Quando habilitado, o interruptor ligado = GPIO 18 em HIGH.
  if (USAR_INTERRUPTOR_START && digitalRead(PINO_START) == LOW) {
    motores.parar();
    return;
  }

  if (quadroNovo) {
    float atividadeTotal = 0.0f;
    ultimaBolaDetectada = estimarDirecao(ultimoPacote, ultimaDirecaoBola,
                                         ultimaConfianca, atividadeTotal);
    ultimaAtividadeMedia = atividadeTotal / IR_SENSOR_COUNT;
  }

  // Atualiza a rotacao pelo BNO entre os pacotes IR tambem, sem interromper
  // a translacao estimada no ultimo quadro recebido.
  if (quadroNovo || rumoAtualizado) {
    if (!atualizarManobraJ(agora)) {
      decidirMovimento(ultimaBolaDetectada, ultimaDirecaoBola);
    }
  } else if (manobraJAtiva) {
    atualizarManobraJ(agora);
  }

  if (quadroNovo && (uint32_t)(agora - ultimaTelemetria) >= INTERVALO_TELEMETRIA_MS) {
    ultimaTelemetria = agora;
    uint8_t calSistema = 0, calGiro = 0, calAcelerometro = 0, calMagnetometro = 0;
    if (bnoDisponivel) {
      bno.getCalibration(calSistema, calGiro, calAcelerometro, calMagnetometro);
    }
    imprimirTelemetria(true, ultimaBolaDetectada, ultimaDirecaoBola,
        ultimaConfianca, ultimaAtividadeMedia, bno.getHeading(),
        erroAngular(rumoDesejadoGraus, bno.getHeading()), correcaoRotacao,
        bno.getGyroZ(), calSistema, calGiro, calAcelerometro, calMagnetometro);
  }
}
