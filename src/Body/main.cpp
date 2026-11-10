#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <IRProtocol.h>
#include <Omni.h>
#include <Motores.h>
#include <PID.h>
#include <freertos/FreeRTOS.h>

void IHM_begin();
void IHM_update();
void IHM_atualizarTelemetria(float headingDegrees, float gyroZDegreesPerSecond,
                             bool bnoValido,
                             const float distanciasUltra[4], uint8_t mascaraUltra,
                             bool ultraValido, const uint8_t atividadesIR[IR_SENSOR_COUNT],
                             bool irValido);

namespace {
constexpr uint32_t UART_BAUD = 115200;
constexpr uint32_t TIMEOUT_PACOTE_MS = 250;
// GPIOs reservados para a UART, fora dos pinos atuais de motores e da IHM.
constexpr int UART_RX_PIN = 18;
constexpr int UART_TX_PIN = 17;
constexpr uint32_t TIMEOUT_BNO_MS = 250;
constexpr float POTENCIA_MAXIMA_MOTORES = 0.70f;
constexpr float VELOCIDADE_SEGUIR = 1.0f;
constexpr float KP_CORRECAO_RUMO = 0.469f;
constexpr float KI_CORRECAO_RUMO = 0.00002f;
constexpr float KD_CORRECAO_RUMO = 0.0056f;
constexpr float ZONA_MORTA_RUMO_GRAUS = 2.0f;
constexpr float ROTACAO_MAXIMA_CORRECAO = 0.30f;
constexpr float LIMITE_INTEGRAL_RUMO = 300.0f;

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
struct QuadroUltraRecebido {
  UltraDataPacket pacote;
  uint32_t recebidoEmMs;
  bool disponivel;
};

portMUX_TYPE travaQuadro = portMUX_INITIALIZER_UNLOCKED;
volatile QuadroRecebido quadroRecebido{};
volatile QuadroBNORecebido quadroBNORecebido{};
volatile QuadroUltraRecebido quadroUltraRecebido{};
uint16_t ultimaSequenciaIR = 0;
uint16_t ultimaSequenciaBNO = 0;
uint16_t ultimaSequenciaUltra = 0;
bool sequenciaIRIniciada = false;
bool sequenciaBNOIniciada = false;
bool sequenciaUltraIniciada = false;
IRActivityPacket ultimoPacote{};
BNODataPacket ultimoPacoteBNO{};
uint32_t ultimoRecebimentoMs = 0;
bool pacoteInicialRecebido = false;
uint32_t ultimoRecebimentoBNOMs = 0;
uint32_t ultimoRecebimentoUltraMs = 0;
UltraDataPacket ultimoPacoteUltra{};
uint8_t ultimaAtividadeIR[IR_SENSOR_COUNT] = {};
bool pacoteUltraInicialRecebido = false;
bool pacoteBNOInicialRecebido = false;
bool referenciaRumoDefinida = false;
float rumoDesejadoGraus = 0.0f;
uint32_t ultimoCalculoPIDUs = 0;
uint32_t ultimoPacoteUARTMs = 0;
bool pacoteUARTRecebido = false;
bool ledUARTVerde = false;
bool ultimaBolaDetectada = false;
float ultimaDirecaoBola = 0.0f;
constexpr size_t TAMANHO_MAXIMO_PACOTE =
    sizeof(IRActivityPacket) > sizeof(BNODataPacket)
        ? (sizeof(IRActivityPacket) > sizeof(UltraDataPacket)
               ? sizeof(IRActivityPacket) : sizeof(UltraDataPacket))
        : (sizeof(BNODataPacket) > sizeof(UltraDataPacket)
               ? sizeof(BNODataPacket) : sizeof(UltraDataPacket));
uint8_t bufferUART[TAMANHO_MAXIMO_PACOTE] = {};
size_t bytesNoBufferUART = 0;

Motores motores;
Omni omni;
PID pidCorrecaoRumo(KP_CORRECAO_RUMO, KI_CORRECAO_RUMO,
                    KD_CORRECAO_RUMO, -ROTACAO_MAXIMA_CORRECAO,
                    ROTACAO_MAXIMA_CORRECAO);

void atualizarLedUART() {
  const bool comunicacaoConfirmada = pacoteUARTRecebido &&
      (uint32_t)(millis() - ultimoPacoteUARTMs) <= TIMEOUT_PACOTE_MS;
  if (comunicacaoConfirmada == ledUARTVerde) return;

  neopixelWrite(RGB_BUILTIN,
                comunicacaoConfirmada ? 0 : RGB_BRIGHTNESS,
                comunicacaoConfirmada ? RGB_BRIGHTNESS : 0,
                0);
  ledUARTVerde = comunicacaoConfirmada;
}

void confirmarPacoteUART() {
  ultimoPacoteUARTMs = millis();
  pacoteUARTRecebido = true;
  atualizarLedUART();
}

void registrarFalhaUART(const char *motivo) {
  (void)motivo;
  pacoteUARTRecebido = false;
  atualizarLedUART();
}

bool receberPacote(const uint8_t *dados, int tamanho) {
  if (tamanho < (int)sizeof(uint16_t)) {
    registrarFalhaUART("quadro recebido menor que o cabecalho do protocolo");
    return false;
  }

  uint16_t magic = 0;
  memcpy(&magic, dados, sizeof(magic));
  if (magic == IR_PACKET_MAGIC) {
    if (tamanho != (int)sizeof(IRActivityPacket)) {
      registrarFalhaUART("tamanho incorreto no quadro IR");
      return false;
    }
    IRActivityPacket copia;
    memcpy(&copia, dados, sizeof(copia));
    if (copia.version != IR_PACKET_VERSION) {
      registrarFalhaUART("versao de protocolo IR nao suportada");
      return false;
    }
    if (copia.checksum != irPacketChecksum(copia)) {
      registrarFalhaUART("checksum invalido no quadro IR");
      return false;
    }
    if (copia.ballDetected > 1 || copia.confidencePercent > 100 ||
        copia.directionTenthsDegrees < -1800 ||
        copia.directionTenthsDegrees > 1800) {
      registrarFalhaUART("resultado TSSP invalido no quadro IR");
      return false;
    }
    confirmarPacoteUART();
    portENTER_CRITICAL(&travaQuadro);
    memcpy((void *)&quadroRecebido.pacote, &copia, sizeof(copia));
    quadroRecebido.recebidoEmMs = millis();
    quadroRecebido.disponivel = true;
    portEXIT_CRITICAL(&travaQuadro);
    return true;
  }

  if (magic == BNO_PACKET_MAGIC) {
    if (tamanho != (int)sizeof(BNODataPacket)) {
      registrarFalhaUART("tamanho incorreto no quadro BNO055");
      return false;
    }
    BNODataPacket copia;
    memcpy(&copia, dados, sizeof(copia));
    if (copia.version != BNO_PACKET_VERSION) {
      registrarFalhaUART("versao de protocolo BNO055 nao suportada");
      return false;
    }
    if (copia.checksum != bnoPacketChecksum(copia)) {
      registrarFalhaUART("checksum invalido no quadro BNO055");
      return false;
    }
    confirmarPacoteUART();
    portENTER_CRITICAL(&travaQuadro);
    memcpy((void *)&quadroBNORecebido.pacote, &copia, sizeof(copia));
    quadroBNORecebido.recebidoEmMs = millis();
    quadroBNORecebido.disponivel = true;
    portEXIT_CRITICAL(&travaQuadro);
    return true;
  }

  if (magic == ULTRA_PACKET_MAGIC) {
    if (tamanho != (int)sizeof(UltraDataPacket)) {
      registrarFalhaUART("tamanho incorreto no quadro ultrassonico");
      return false;
    }
    UltraDataPacket copia;
    memcpy(&copia, dados, sizeof(copia));
    if (copia.version != ULTRA_PACKET_VERSION) {
      registrarFalhaUART("versao de protocolo ultrassonico nao suportada");
      return false;
    }
    if (copia.checksum != ultraPacketChecksum(copia)) {
      registrarFalhaUART("checksum invalido no quadro ultrassonico");
      return false;
    }
    confirmarPacoteUART();
    portENTER_CRITICAL(&travaQuadro);
    memcpy((void *)&quadroUltraRecebido.pacote, &copia, sizeof(copia));
    quadroUltraRecebido.recebidoEmMs = millis();
    quadroUltraRecebido.disponivel = true;
    portEXIT_CRITICAL(&travaQuadro);
    return true;
  }

  registrarFalhaUART("magic desconhecido no cabecalho do quadro UART");
  return false;
}

bool byteEhInicioDeQuadro(uint8_t byteRecebido) {
  return byteRecebido == static_cast<uint8_t>(IR_PACKET_MAGIC) ||
         byteRecebido == static_cast<uint8_t>(BNO_PACKET_MAGIC) ||
         byteRecebido == static_cast<uint8_t>(ULTRA_PACKET_MAGIC);
}

void processarUART() {
  while (Serial1.available() > 0) {
    if (bytesNoBufferUART == sizeof(bufferUART)) {
      memmove(bufferUART, bufferUART + 1, --bytesNoBufferUART);
    }
    const uint8_t byteRecebido = static_cast<uint8_t>(Serial1.read());
    bufferUART[bytesNoBufferUART++] = byteRecebido;

    while (bytesNoBufferUART > 0) {
      if (!byteEhInicioDeQuadro(bufferUART[0])) {
        memmove(bufferUART, bufferUART + 1, --bytesNoBufferUART);
        continue;
      }
      if (bytesNoBufferUART < sizeof(uint16_t)) break;

      uint16_t magic = 0;
      memcpy(&magic, bufferUART, sizeof(magic));
      size_t tamanhoQuadro;
      if (magic == IR_PACKET_MAGIC) tamanhoQuadro = sizeof(IRActivityPacket);
      else if (magic == BNO_PACKET_MAGIC) tamanhoQuadro = sizeof(BNODataPacket);
      else if (magic == ULTRA_PACKET_MAGIC) tamanhoQuadro = sizeof(UltraDataPacket);
      else {
        memmove(bufferUART, bufferUART + 1, --bytesNoBufferUART);
        continue;
      }

      if (bytesNoBufferUART < tamanhoQuadro) break;
      if (receberPacote(bufferUART, static_cast<int>(tamanhoQuadro))) {
        const size_t bytesRestantes = bytesNoBufferUART - tamanhoQuadro;
        memmove(bufferUART, bufferUART + tamanhoQuadro, bytesRestantes);
        bytesNoBufferUART = bytesRestantes;
      } else {
        memmove(bufferUART, bufferUART + 1, --bytesNoBufferUART);
      }
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

bool obterQuadroUltra(UltraDataPacket &destino, uint32_t &recebidoEmMs) {
  bool disponivel;
  portENTER_CRITICAL(&travaQuadro);
  disponivel = quadroUltraRecebido.disponivel;
  if (disponivel) {
    memcpy(&destino, (const void *)&quadroUltraRecebido.pacote, sizeof(destino));
    recebidoEmMs = quadroUltraRecebido.recebidoEmMs;
    quadroUltraRecebido.disponivel = false;
  }
  portEXIT_CRITICAL(&travaQuadro);
  return disponivel;
}

float erroAngular(float desejado, float atual) {
  float erro = desejado - atual;
  while (erro > 180.0f) erro -= 360.0f;
  while (erro < -180.0f) erro += 360.0f;
  return erro;
}

float calcularCorrecaoRumo(bool bolaDetectada, float heading, float gyroZ) {
  const uint32_t agoraUs = micros();
  const uint32_t decorridoUs = agoraUs - ultimoCalculoPIDUs;
  ultimoCalculoPIDUs = agoraUs;

  if (!bolaDetectada) {
    pidCorrecaoRumo.reset();
    return 0.0f;
  }

  float erro = erroAngular(rumoDesejadoGraus, heading);
  if (fabsf(erro) <= ZONA_MORTA_RUMO_GRAUS) erro = 0.0f;

  const float dt = constrain(decorridoUs / 1000000.0f, 0.001f, 0.1f);
  return pidCorrecaoRumo.calculate(erro, gyroZ, dt);
}

void aplicarRodas(float x, float y, float rotacao) {
  const ComandosMotores comando = omni.calcular(x, y, rotacao);
  motores.definir(comando.m1 * POTENCIA_MAXIMA_MOTORES,
                  comando.m2 * POTENCIA_MAXIMA_MOTORES,
                  comando.m3 * POTENCIA_MAXIMA_MOTORES,
                  comando.m4 * POTENCIA_MAXIMA_MOTORES);
}

void imprimirDadosIR(const IRActivityPacket &pacote) {
  if (pacote.ballDetected) {
    Serial.printf("Direcao da bola: %.1f graus | Angulo BNO: %.1f graus\n",
                  pacote.directionTenthsDegrees / 10.0f,
                  ultimoPacoteBNO.headingDegrees);
  } else {
    Serial.printf("Direcao da bola: nao detectada | Angulo BNO: %.1f graus\n",
                  ultimoPacoteBNO.headingDegrees);
  }
}

void decidirMovimento(bool bolaDetectada, float anguloGraus, float rotacao) {
  if (!bolaDetectada) {
    motores.parar();
    return;
  }
  const float angulo = anguloGraus * DEG_TO_RAD;
  const float x = VELOCIDADE_SEGUIR * sinf(angulo);
  const float y = VELOCIDADE_SEGUIR * cosf(angulo);
  aplicarRodas(x, y, rotacao);
}

} // namespace

void setup() {
  neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);
  Serial.begin(115200);
  pidCorrecaoRumo.setIntegralLimit(LIMITE_INTEGRAL_RUMO);
  ultimoCalculoPIDUs = micros();
  motores.begin();
  motores.parar();
  IHM_begin();

  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
}

void loop() {
  processarUART();
  atualizarLedUART();
  IHM_update();

  BNODataPacket pacoteBNO{};
  uint32_t recebidoBNOEmMs = 0;
  if (obterQuadroBNO(pacoteBNO, recebidoBNOEmMs)) {
    if (!sequenciaBNOIniciada ||
        (int16_t)(pacoteBNO.sequence - ultimaSequenciaBNO) > 0) {
      if (!isfinite(pacoteBNO.headingDegrees) ||
          !isfinite(pacoteBNO.gyroZDegreesPerSecond) ||
          pacoteBNO.headingDegrees < -180.0f ||
          pacoteBNO.headingDegrees >= 180.0f) {
        registrarFalhaUART("heading ou gyro invalido no quadro BNO055");
      } else {
        ultimaSequenciaBNO = pacoteBNO.sequence;
        sequenciaBNOIniciada = true;
        ultimoPacoteBNO = pacoteBNO;
        if (!referenciaRumoDefinida) {
          rumoDesejadoGraus = pacoteBNO.headingDegrees;
          referenciaRumoDefinida = true;
        }
        pacoteBNOInicialRecebido = true;
        ultimoRecebimentoBNOMs = recebidoBNOEmMs;
      }
    }
  }

  UltraDataPacket pacoteUltra{};
  uint32_t recebidoUltraEmMs = 0;
  if (obterQuadroUltra(pacoteUltra, recebidoUltraEmMs) &&
      (!sequenciaUltraIniciada ||
       (int16_t)(pacoteUltra.sequence - ultimaSequenciaUltra) > 0)) {
    ultimaSequenciaUltra = pacoteUltra.sequence;
    sequenciaUltraIniciada = true;
    ultimoPacoteUltra = pacoteUltra;
    ultimoRecebimentoUltraMs = recebidoUltraEmMs;
    pacoteUltraInicialRecebido = true;
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
      memcpy(ultimaAtividadeIR, pacote.activity, sizeof(ultimaAtividadeIR));
      ultimaBolaDetectada = pacote.ballDetected != 0;
      ultimaDirecaoBola = pacote.directionTenthsDegrees / 10.0f;
      ultimoRecebimentoMs = recebidoEmMs;
      pacoteInicialRecebido = true;
    }
  }

  const bool bnoTelemetriaValida = pacoteBNOInicialRecebido &&
      (uint32_t)(agora - ultimoRecebimentoBNOMs) <= TIMEOUT_BNO_MS;
  const bool ultraTelemetriaValida = pacoteUltraInicialRecebido &&
      (uint32_t)(agora - ultimoRecebimentoUltraMs) <= TIMEOUT_PACOTE_MS;
  const bool irTelemetriaValida = pacoteInicialRecebido &&
      (uint32_t)(agora - ultimoRecebimentoMs) <= TIMEOUT_PACOTE_MS;
  IHM_atualizarTelemetria(
      ultimoPacoteBNO.headingDegrees, ultimoPacoteBNO.gyroZDegreesPerSecond,
      bnoTelemetriaValida, ultimoPacoteUltra.distanceCm,
      ultimoPacoteUltra.validMask, ultraTelemetriaValida,
      ultimaAtividadeIR, irTelemetriaValida);

  if (!pacoteInicialRecebido ||
      (uint32_t)(agora - ultimoRecebimentoMs) > TIMEOUT_PACOTE_MS ||
      !bnoTelemetriaValida || !referenciaRumoDefinida) {
    // Sem dados IR/BNO recentes, interrompe o movimento por seguranca.
    motores.parar();
    return;
  }

  if (quadroNovo) {
    imprimirDadosIR(ultimoPacote);
  }

  if (quadroNovo) {
    const float correcaoRumo = calcularCorrecaoRumo(
        ultimaBolaDetectada,
        ultimoPacoteBNO.headingDegrees,
        ultimoPacoteBNO.gyroZDegreesPerSecond);
    decidirMovimento(ultimaBolaDetectada, ultimaDirecaoBola, correcaoRumo);
  }

}
