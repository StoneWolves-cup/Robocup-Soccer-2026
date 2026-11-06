#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <IRProtocol.h>
#include <Omni.h>
#include <Motores.h>
#include <FuzzyPID.h>

namespace {
constexpr int UART_RX_PIN = 7;
constexpr int UART_TX_PIN = 10;
constexpr uint32_t UART_BAUD = 115200;
constexpr uint32_t TIMEOUT_PACOTE_MS = 250;
constexpr uint32_t TIMEOUT_BNO_MS = 250;
constexpr float LIMIAR_ATIVIDADE = 2.0f;
constexpr float CONFIANCA_MIN = 0.15f;
constexpr float ALPHA_DIRECAO = 0.25f;
constexpr float VELOCIDADE_SEGUIR = 1.0f;
// Referencia capturada na primeira leitura remota do BNO.
float rumoDesejadoGraus = 0.0f;
// A saida do controlador e uma fracao do PWM. Com 0.10, erros comuns
// produziam menos que a zona morta dos motores (0.02) e nao geravam torque.
constexpr float CORRECAO_MAX_RODAS = 0.35f;
constexpr float GANHO_P = 0.67f;
constexpr float GANHO_I = 0.00002f;
constexpr float GANHO_D = 0.008f;

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
struct QuadroBNORecebido {
  BNODataPacket pacote;
  uint32_t recebidoEmMs;
  bool disponivel;
};

portMUX_TYPE travaQuadro = portMUX_INITIALIZER_UNLOCKED;
volatile QuadroRecebido quadroRecebido{};
volatile QuadroBNORecebido quadroBNORecebido{};
uint16_t ultimaSequenciaIR = 0;
uint16_t ultimaSequenciaBNO = 0;
bool sequenciaIRIniciada = false;
bool sequenciaBNOIniciada = false;
IRActivityPacket ultimoPacote{};
uint32_t ultimoRecebimentoMs = 0;
bool pacoteInicialRecebido = false;
uint32_t ultimoRecebimentoBNOMs = 0;
float correcaoRotacao = 0.0f;
FuzzyPID fuzzyPidRumo(GANHO_P, GANHO_I, GANHO_D, -CORRECAO_MAX_RODAS, CORRECAO_MAX_RODAS);
float vetorXFiltrado = 0.0f; // lateral: esquerda positiva
float vetorYFiltrado = 0.0f; // frente positiva
float somaPesosFiltrada = 0.0f;
bool filtroIniciado = false;
bool bnoDisponivel = false;
bool ultimaBolaDetectada = false;
float ultimaDirecaoBola = 0.0f;
uint8_t bufferUART[sizeof(IRActivityPacket) > sizeof(BNODataPacket)
                       ? sizeof(IRActivityPacket) : sizeof(BNODataPacket)] = {};
size_t bytesNoBufferUART = 0;
size_t tamanhoQuadroUART = 0;

Motores motores;
Omni omni;

void receberPacote(const uint8_t *dados, int tamanho) {
  if (tamanho < (int)sizeof(uint16_t)) return;
  uint16_t magic = 0;
  memcpy(&magic, dados, sizeof(magic));
  if (magic == IR_PACKET_MAGIC && tamanho == (int)sizeof(IRActivityPacket)) {
    IRActivityPacket copia;
    memcpy(&copia, dados, sizeof(copia));
    if (copia.version != IR_PACKET_VERSION || copia.checksum != irPacketChecksum(copia)) return;
    portENTER_CRITICAL(&travaQuadro);
    memcpy((void *)&quadroRecebido.pacote, &copia, sizeof(copia));
    quadroRecebido.recebidoEmMs = millis();
    quadroRecebido.disponivel = true;
    portEXIT_CRITICAL(&travaQuadro);
  } else if (magic == BNO_PACKET_MAGIC && tamanho == (int)sizeof(BNODataPacket)) {
    BNODataPacket copia;
    memcpy(&copia, dados, sizeof(copia));
    if (copia.version != BNO_PACKET_VERSION || copia.checksum != bnoPacketChecksum(copia)) return;
    portENTER_CRITICAL(&travaQuadro);
    memcpy((void *)&quadroBNORecebido.pacote, &copia, sizeof(copia));
    quadroBNORecebido.recebidoEmMs = millis();
    quadroBNORecebido.disponivel = true;
    portEXIT_CRITICAL(&travaQuadro);
  }
}

void processarUART() {
  while (Serial1.available() > 0) {
    const uint8_t byteRecebido = static_cast<uint8_t>(Serial1.read());

    if (bytesNoBufferUART == 0) {
      if (byteRecebido == static_cast<uint8_t>(IR_PACKET_MAGIC) ||
          byteRecebido == static_cast<uint8_t>(BNO_PACKET_MAGIC)) {
        bufferUART[0] = byteRecebido;
        bytesNoBufferUART = 1;
      }
      continue;
    }

    if (bytesNoBufferUART == 1) {
      bufferUART[1] = byteRecebido;
      uint16_t magic = 0;
      memcpy(&magic, bufferUART, sizeof(magic));
      if (magic == IR_PACKET_MAGIC) {
        tamanhoQuadroUART = sizeof(IRActivityPacket);
      } else if (magic == BNO_PACKET_MAGIC) {
        tamanhoQuadroUART = sizeof(BNODataPacket);
      } else {
        bytesNoBufferUART =
            (byteRecebido == static_cast<uint8_t>(IR_PACKET_MAGIC) ||
             byteRecebido == static_cast<uint8_t>(BNO_PACKET_MAGIC)) ? 1 : 0;
        if (bytesNoBufferUART == 1) bufferUART[0] = byteRecebido;
        continue;
      }
      bytesNoBufferUART = 2;
      continue;
    }

    bufferUART[bytesNoBufferUART++] = byteRecebido;
    if (bytesNoBufferUART == tamanhoQuadroUART) {
      receberPacote(bufferUART, static_cast<int>(tamanhoQuadroUART));
      bytesNoBufferUART = 0;
      tamanhoQuadroUART = 0;
    }
  }
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

bool obterQuadroBNO(BNODataPacket &destino, uint32_t &recebidoEmMs) {
  bool disponivel;
  portENTER_CRITICAL(&travaQuadro);
  disponivel = quadroBNORecebido.disponivel;
  if (disponivel) {
    memcpy(&destino, (const void *)&quadroBNORecebido.pacote, sizeof(destino));
    recebidoEmMs = quadroBNORecebido.recebidoEmMs;
    quadroBNORecebido.disponivel = false;
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

bool atualizarCorrecaoBNO(const BNODataPacket &pacote, uint32_t recebidoEmMs) {
  if (!bnoDisponivel) {
    rumoDesejadoGraus = pacote.headingDegrees;
    fuzzyPidRumo.reset();
    bnoDisponivel = true;
    ultimoRecebimentoBNOMs = recebidoEmMs;
    return true;
  }

  const uint32_t decorridoMs = recebidoEmMs - ultimoRecebimentoBNOMs;
  if (decorridoMs == 0) return false;
  const float erro = erroAngular(rumoDesejadoGraus, pacote.headingDegrees);
  const float dt = decorridoMs * 1.0e-3f;
  // Fuzzy PID usa o rumo e o giro Z recebidos no pacote independente do BNO.
  correcaoRotacao = fuzzyPidRumo.calculate(erro, pacote.gyroZDegreesPerSecond, dt);
  ultimoRecebimentoBNOMs = recebidoEmMs;
  return true;
}

bool estimarDirecao(const IRActivityPacket &pacote, float &anguloGraus,
                    float &confianca) {
  // Replica a estrategia do codigo de referencia: encontra o sensor mais
  // ativo, considera apenas sua vizinhanca de sete sensores e pondera a
  // direcao pelo quadrado da atividade para destacar o pico principal.
  uint8_t indiceMaximo = 0;
  uint8_t atividadeMaxima = 0;
  for (uint8_t i = 0; i < IR_SENSOR_COUNT; ++i) {
    if (pacote.activity[i] > atividadeMaxima) {
      atividadeMaxima = pacote.activity[i];
      indiceMaximo = i;
    }
  }

  if (atividadeMaxima <= LIMIAR_ATIVIDADE) {
    confianca = 0.0f;
    filtroIniciado = false;
    return false;
  }

  float x = 0.0f, y = 0.0f, somaPesos = 0.0f;
  for (int8_t deslocamento = -3; deslocamento <= 3; ++deslocamento) {
    const uint8_t indice = (indiceMaximo + IR_SENSOR_COUNT + deslocamento)
                           % IR_SENSOR_COUNT;
    const float atividade = pacote.activity[indice];
    if (atividade < atividadeMaxima * 0.25f) continue;

    const float peso = atividade * atividade;
    const float rad = ANGULOS_GRAUS[indice] * DEG_TO_RAD;
    // Conversao para eixos de movimento: x lateral, y para frente.
    x += peso * sinf(rad);
    y += peso * cosf(rad);
    somaPesos += peso;
  }

  if (somaPesos <= 0.0f) {
    confianca = 0.0f;
    filtroIniciado = false;
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
  if (confianca < CONFIANCA_MIN) {
    filtroIniciado = false;
    return false;
  }

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
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  delay(200);
  Serial.printf("Inicializando Body | UART1 %lu baud | RX=%d TX=%d\n",
                static_cast<unsigned long>(UART_BAUD), UART_RX_PIN, UART_TX_PIN);

  motores.begin();
  motores.parar();

}

void loop() {
  processarUART();

  BNODataPacket pacoteBNO{};
  uint32_t recebidoBNOEmMs = 0;
  bool rumoAtualizado = false;
  if (obterQuadroBNO(pacoteBNO, recebidoBNOEmMs)) {
    if (!sequenciaBNOIniciada ||
        (int16_t)(pacoteBNO.sequence - ultimaSequenciaBNO) > 0) {
      ultimaSequenciaBNO = pacoteBNO.sequence;
      sequenciaBNOIniciada = true;
      rumoAtualizado = atualizarCorrecaoBNO(pacoteBNO, recebidoBNOEmMs);
    }
  }

  IRActivityPacket pacote{};
  uint32_t recebidoEmMs = 0;
  const bool chegouPacote = obterQuadro(pacote, recebidoEmMs);
  const uint32_t agora = millis();
  bool quadroNovo = chegouPacote;
  if (chegouPacote) {
    // Descarta pacotes repetidos/antigos, inclusive quando chegam fora de ordem.
    if (sequenciaIRIniciada && (int16_t)(pacote.sequence - ultimaSequenciaIR) <= 0) {
      quadroNovo = false;
    } else {
      ultimaSequenciaIR = pacote.sequence;
      sequenciaIRIniciada = true;
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

  if (!bnoDisponivel || (uint32_t)(agora - ultimoRecebimentoBNOMs) > TIMEOUT_BNO_MS) {
    correcaoRotacao = 0.0f;
  }

  if (quadroNovo) {
    float confianca = 0.0f;
    ultimaBolaDetectada = estimarDirecao(ultimoPacote, ultimaDirecaoBola,
                                         confianca);
  }

  // Atualiza a rotacao com o pacote BNO sem esperar uma nova leitura IR.
  if (quadroNovo || rumoAtualizado) {
    decidirMovimento(ultimaBolaDetectada, ultimaDirecaoBola);
  }

}
