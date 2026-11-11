#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <IRProtocol.h>
#include <Omni.h>
#include <Motores.h>
#include <FuzzyPID.h>
#include <freertos/FreeRTOS.h>

void IHM_begin();
void IHM_update();
void IHM_atualizarTelemetria(float headingDegrees, float gyroZDegreesPerSecond,
                             bool bnoValido,
                             const float distanciasUltra[4], uint8_t mascaraUltra,
                             bool ultraValido, const uint8_t atividadesIR[IR_SENSOR_COUNT],
                             bool irValido);

namespace {
constexpr uint32_t USB_SERIAL_BAUD = 460800;
constexpr uint32_t UART_BAUD = 115200;
constexpr uint32_t TIMEOUT_PACOTE_MS = 250;
constexpr uint32_t INTERVALO_DIAGNOSTICO_UART_MS = 1000;
constexpr size_t TAMANHO_BUFFER_RX_UART = 4096;
constexpr bool USAR_LDR = false;
// GPIOs reservados para a UART, fora dos pinos atuais de motores e da IHM.
constexpr int UART_RX_PIN = 18;
constexpr int UART_TX_PIN = 17;
constexpr uint32_t TIMEOUT_BNO_MS = 250;
constexpr float POTENCIA_MAXIMA_MOTORES = 0.80f;
constexpr float VELOCIDADE_CORRECAO_LINHA = 1.0f;
constexpr float KP_CORRECAO_RUMO = 0.00582f;
constexpr float KI_CORRECAO_RUMO = 0.0000032f;
constexpr float KD_CORRECAO_RUMO = 0.00074f;
constexpr float ESCALA_ERRO_FUZZY_GRAUS = 18.5414f;
constexpr float ESCALA_GYRO_FUZZY_GRAUS_POR_SEGUNDO = 74.1656f;
constexpr float ZONA_MORTA_RUMO_GRAUS = 1.2360f;
constexpr float ROTACAO_MAXIMA_CORRECAO = 0.80f;
constexpr float LIMITE_INTEGRAL_RUMO = 300.0f;
constexpr uint8_t LDR_SENSOR_COUNT = 15;
constexpr uint8_t LDR_REGION_COUNT = 4;
constexpr uint8_t LDR_MUX_SIG = 14;
constexpr uint8_t LDR_MUX_S0 = 10;
constexpr uint8_t LDR_MUX_S1 = 11;
constexpr uint8_t LDR_MUX_S2 = 12;
constexpr uint8_t LDR_MUX_S3 = 13;
constexpr uint8_t LDR_SAMPLE_COUNT = 12;
constexpr uint8_t LDR_DISCARD_COUNT = 2;
constexpr uint8_t LDR_CALIBRATION_SAMPLES = 8;
constexpr uint8_t LDR_HYSTERESIS_DIVISOR = 8;

enum RegiaoLDR : uint8_t {
  LDR_CENTRO,
  LDR_ESQUERDA,
  LDR_DIREITA,
  LDR_INFERIOR
};

const char *const NOMES_REGIOES_LDR[LDR_REGION_COUNT] = {
    "Centro", "Esquerda", "Direita", "Inferior"};
const uint8_t CANAIS_REGIOES_LDR[LDR_REGION_COUNT][9] = {
    {0, 1, 2, 3, 4, 5, 6, 7, 14},
    {12, 13},
    {8, 9},
    {10, 11}};
const uint8_t CONTAGENS_REGIOES_LDR[LDR_REGION_COUNT] = {9, 2, 2, 2};

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
uint32_t ultimoPacoteIRUARTMs = 0;
uint32_t ultimoPacoteBNOUARTMs = 0;
bool pacoteIRUARTRecebido = false;
bool pacoteBNOUARTRecebido = false;
bool referenciaRumoDefinida = false;
float rumoDesejadoGraus = 0.0f;
uint32_t ultimoCalculoPIDUs = 0;
bool ledUARTVerde = false;
uint32_t bytesUARTDesdeDiagnostico = 0;
uint32_t pacotesIRDesdeDiagnostico = 0;
uint32_t pacotesBNODesteDiagnostico = 0;
uint32_t pacotesUltraDesdeDiagnostico = 0;
uint32_t quadrosInvalidosDesdeDiagnostico = 0;
uint32_t ultimoDiagnosticoUARTMs = 0;
const char *ultimoErroUART = nullptr;
const char *ultimoBloqueioMovimento = nullptr;
uint32_t ultimaMensagemBloqueioMs = 0;
bool ultimaBolaDetectada = false;
float ultimaDirecaoBola = 0.0f;
float ultimaCorrecaoRumo = 0.0f;
uint16_t referenciaBrancaLDR[LDR_SENSOR_COUNT] = {};
uint16_t referenciaVerdeLDR[LDR_SENSOR_COUNT] = {};
uint16_t limiarLDR[LDR_SENSOR_COUNT] = {};
uint16_t referenciaBrancaRegiaoLDR[LDR_REGION_COUNT] = {};
uint16_t referenciaVerdeRegiaoLDR[LDR_REGION_COUNT] = {};
uint16_t limiarRegiaoLDR[LDR_REGION_COUNT] = {};
bool referenciaBrancaCapturadaLDR = false;
bool referenciaVerdeCapturadaLDR = false;
bool brancoAcimaLDR[LDR_REGION_COUNT] = {};
bool regiaoBrancaLDR[LDR_REGION_COUNT] = {};
bool sensorBrancoLDR[LDR_SENSOR_COUNT] = {};
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
FuzzyPID pidCorrecaoRumo(
    KP_CORRECAO_RUMO, KI_CORRECAO_RUMO, KD_CORRECAO_RUMO,
    -ROTACAO_MAXIMA_CORRECAO, ROTACAO_MAXIMA_CORRECAO,
    ESCALA_ERRO_FUZZY_GRAUS, ESCALA_GYRO_FUZZY_GRAUS_POR_SEGUNDO);

void atualizarLedUART() {
  const uint32_t agoraMs = millis();
  const bool comunicacaoConfirmada = pacoteIRUARTRecebido &&
      pacoteBNOUARTRecebido &&
      (uint32_t)(agoraMs - ultimoPacoteIRUARTMs) <= TIMEOUT_PACOTE_MS &&
      (uint32_t)(agoraMs - ultimoPacoteBNOUARTMs) <= TIMEOUT_BNO_MS;
  if (comunicacaoConfirmada == ledUARTVerde) return;

  neopixelWrite(RGB_BUILTIN,
                comunicacaoConfirmada ? 0 : RGB_BRIGHTNESS,
                comunicacaoConfirmada ? RGB_BRIGHTNESS : 0,
                0);
  ledUARTVerde = comunicacaoConfirmada;
}

void confirmarPacoteIRUART() {
  ultimoPacoteIRUARTMs = millis();
  pacoteIRUARTRecebido = true;
  atualizarLedUART();
}

void confirmarPacoteBNOUART() {
  ultimoPacoteBNOUARTMs = millis();
  pacoteBNOUARTRecebido = true;
  atualizarLedUART();
}

void registrarFalhaUART(const char *motivo) {
  ++quadrosInvalidosDesdeDiagnostico;
  ultimoErroUART = motivo;
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
    ++pacotesIRDesdeDiagnostico;
    confirmarPacoteIRUART();
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
    if (!isfinite(copia.headingDegrees) ||
        !isfinite(copia.gyroZDegreesPerSecond) ||
        copia.headingDegrees < -180.0f || copia.headingDegrees >= 180.0f) {
      registrarFalhaUART("heading ou gyro invalido no quadro BNO055");
      return false;
    }
    ++pacotesBNODesteDiagnostico;
    confirmarPacoteBNOUART();
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
    ++pacotesUltraDesdeDiagnostico;
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
    const int leitura = Serial1.read();
    if (leitura < 0) return;
    const uint8_t byteRecebido = static_cast<uint8_t>(leitura);
    ++bytesUARTDesdeDiagnostico;
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

void imprimirDiagnosticoUART() {
  const uint32_t agoraMs = millis();
  if ((uint32_t)(agoraMs - ultimoDiagnosticoUARTMs) <
      INTERVALO_DIAGNOSTICO_UART_MS) {
    return;
  }
  ultimoDiagnosticoUARTMs = agoraMs;

  Serial.printf(
      "UART1/s: %lu bytes, IR=%lu, BNO=%lu, ultra=%lu, invalidos=%lu, pendentes=%u",
      static_cast<unsigned long>(bytesUARTDesdeDiagnostico),
      static_cast<unsigned long>(pacotesIRDesdeDiagnostico),
      static_cast<unsigned long>(pacotesBNODesteDiagnostico),
      static_cast<unsigned long>(pacotesUltraDesdeDiagnostico),
      static_cast<unsigned long>(quadrosInvalidosDesdeDiagnostico),
      static_cast<unsigned>(bytesNoBufferUART));
  if (ultimoErroUART != nullptr) {
    Serial.printf(" | ultimo erro: %s", ultimoErroUART);
  }
  Serial.println();

  bytesUARTDesdeDiagnostico = 0;
  pacotesIRDesdeDiagnostico = 0;
  pacotesBNODesteDiagnostico = 0;
  pacotesUltraDesdeDiagnostico = 0;
  quadrosInvalidosDesdeDiagnostico = 0;
  ultimoErroUART = nullptr;
}

void informarBloqueioMovimento(const char *motivo) {
  const uint32_t agoraMs = millis();
  if (ultimoBloqueioMovimento == nullptr ||
      strcmp(ultimoBloqueioMovimento, motivo) != 0 ||
      (uint32_t)(agoraMs - ultimaMensagemBloqueioMs) >= 1000) {
    Serial.printf("MOVIMENTO BLOQUEADO: %s\n", motivo);
    ultimoBloqueioMovimento = motivo;
    ultimaMensagemBloqueioMs = agoraMs;
  }
}

void informarMovimentoLiberado() {
  if (ultimoBloqueioMovimento != nullptr) {
    Serial.println("MOVIMENTO LIBERADO.");
    ultimoBloqueioMovimento = nullptr;
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

void selecionarCanalLDR(uint8_t canal) {
  digitalWrite(LDR_MUX_S0, canal & 0x01);
  digitalWrite(LDR_MUX_S1, (canal >> 1) & 0x01);
  digitalWrite(LDR_MUX_S2, (canal >> 2) & 0x01);
  digitalWrite(LDR_MUX_S3, (canal >> 3) & 0x01);
  delayMicroseconds(10);
}

uint16_t lerSensorLDR(uint8_t canal) {
  selecionarCanalLDR(canal);
  for (uint8_t i = 0; i < LDR_DISCARD_COUNT; ++i) analogRead(LDR_MUX_SIG);

  uint32_t total = 0;
  for (uint8_t i = 0; i < LDR_SAMPLE_COUNT; ++i) {
    total += analogRead(LDR_MUX_SIG);
    delayMicroseconds(100);
  }
  return total / LDR_SAMPLE_COUNT;
}

void capturarReferenciaLDR(uint16_t referencia[LDR_SENSOR_COUNT]) {
  for (uint8_t sensor = 0; sensor < LDR_SENSOR_COUNT; ++sensor) {
    uint32_t total = 0;
    for (uint8_t amostra = 0; amostra < LDR_CALIBRATION_SAMPLES; ++amostra) {
      total += lerSensorLDR(sensor);
      delay(2);
    }
    referencia[sensor] = total / LDR_CALIBRATION_SAMPLES;
  }
}

void prepararClassificacaoLDR() {
  for (uint8_t sensor = 0; sensor < LDR_SENSOR_COUNT; ++sensor) {
    limiarLDR[sensor] =
        (referenciaBrancaLDR[sensor] + referenciaVerdeLDR[sensor]) / 2;
    sensorBrancoLDR[sensor] =
        referenciaBrancaLDR[sensor] >= referenciaVerdeLDR[sensor];
  }

  for (uint8_t regiao = 0; regiao < LDR_REGION_COUNT; ++regiao) {
    uint32_t totalBranco = 0;
    uint32_t totalVerde = 0;
    uint32_t totalLimiar = 0;
    for (uint8_t i = 0; i < CONTAGENS_REGIOES_LDR[regiao]; ++i) {
      const uint8_t canal = CANAIS_REGIOES_LDR[regiao][i];
      totalBranco += referenciaBrancaLDR[canal];
      totalVerde += referenciaVerdeLDR[canal];
      totalLimiar += limiarLDR[canal];
    }

    const uint8_t quantidade = CONTAGENS_REGIOES_LDR[regiao];
    referenciaBrancaRegiaoLDR[regiao] = totalBranco / quantidade;
    referenciaVerdeRegiaoLDR[regiao] = totalVerde / quantidade;
    limiarRegiaoLDR[regiao] = totalLimiar / quantidade;
    brancoAcimaLDR[regiao] =
        referenciaBrancaRegiaoLDR[regiao] >= referenciaVerdeRegiaoLDR[regiao];
    regiaoBrancaLDR[regiao] = brancoAcimaLDR[regiao];
  }
}

bool processarComandosLDR() {
  bool comandoExecutado = false;
  while (Serial.available() > 0) {
    const char comando = Serial.read();
    if (comando == 'w' || comando == 'W') {
      comandoExecutado = true;
      motores.parar();
      Serial.println("Capturando referencia branca; mantenha os LDRs sobre branco.");
      capturarReferenciaLDR(referenciaBrancaLDR);
      referenciaBrancaCapturadaLDR = true;
      if (referenciaVerdeCapturadaLDR) prepararClassificacaoLDR();
      Serial.println("Referencia branca LDR capturada.");
    } else if (comando == 'g' || comando == 'G') {
      comandoExecutado = true;
      motores.parar();
      Serial.println("Capturando referencia verde; mantenha os LDRs sobre verde.");
      capturarReferenciaLDR(referenciaVerdeLDR);
      referenciaVerdeCapturadaLDR = true;
      if (referenciaBrancaCapturadaLDR) prepararClassificacaoLDR();
      Serial.println("Referencia verde LDR capturada.");
    } else if (comando == 'r' || comando == 'R') {
      comandoExecutado = true;
      motores.parar();
      referenciaBrancaCapturadaLDR = false;
      referenciaVerdeCapturadaLDR = false;
      Serial.println("Calibracao LDR reiniciada; motores permanecem parados.");
    }
  }
  return comandoExecutado;
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

void decidirMovimento(bool bolaDetectada, float anguloGraus, float rotacao,
                      bool imprimirDiagnostico) {
  if (!bolaDetectada) {
    informarBloqueioMovimento(
        "Head nao detecta bola (atividade abaixo do limiar de confianca).");
    motores.parar();
    return;
  }

  float x = 0.0f;
  float y = 0.0f;
  const char *movimento;
  if (anguloGraus >= -45.0f && anguloGraus <= 45.0f) {
    y = VELOCIDADE_CORRECAO_LINHA;
    movimento = "para frente";
  } else if (anguloGraus < -45.0f && anguloGraus >= -135.0f) {
    x = -VELOCIDADE_CORRECAO_LINHA;
    movimento = "para direita";
  } else if (anguloGraus > 45.0f && anguloGraus <= 135.0f) {
    x = VELOCIDADE_CORRECAO_LINHA;
    movimento = "para esquerda";
  } else {
    y = -VELOCIDADE_CORRECAO_LINHA;
    movimento = "para tras";
  }

  if (imprimirDiagnostico) {
    Serial.printf("Perseguindo bola por IR: %s | rumo=%.1f graus | rotacao=%.3f\n",
                  movimento, ultimoPacoteBNO.headingDegrees, rotacao);
  }

  informarMovimentoLiberado();
  aplicarRodas(x, y, rotacao);
}

} // namespace

void setup() {
  neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);
  Serial.begin(USB_SERIAL_BAUD);
  const size_t tamanhoBufferUARTConfigurado =
      Serial1.setRxBufferSize(TAMANHO_BUFFER_RX_UART);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  pidCorrecaoRumo.setIntegralLimit(LIMITE_INTEGRAL_RUMO);
  ultimoCalculoPIDUs = micros();
  motores.begin();
  motores.parar();
  if (USAR_LDR) {
    pinMode(LDR_MUX_SIG, INPUT);
    pinMode(LDR_MUX_S0, OUTPUT);
    pinMode(LDR_MUX_S1, OUTPUT);
    pinMode(LDR_MUX_S2, OUTPUT);
    pinMode(LDR_MUX_S3, OUTPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(LDR_MUX_SIG, ADC_11db);
  }
  IHM_begin();

  Serial.println("Body UART1: 115200 baud, 8N1, RX=18 TX=17.");
  if (tamanhoBufferUARTConfigurado < TAMANHO_BUFFER_RX_UART) {
    Serial.printf("ERRO: buffer RX UART1 configurado com %u de %u bytes.\n",
                  static_cast<unsigned>(tamanhoBufferUARTConfigurado),
                  static_cast<unsigned>(TAMANHO_BUFFER_RX_UART));
  }
  Serial.println("LED verde requer pacotes IR e BNO validos e recentes.");
  Serial.println(USAR_LDR
      ? "LDR habilitado: calibra com 'w' (branco) e 'g' (verde)."
      : "LDR desativado; movimento controlado somente por IR e BNO.");
}

void loop() {
  processarUART();
  atualizarLedUART();
  imprimirDiagnosticoUART();
  IHM_update();
  if (USAR_LDR && processarComandosLDR()) {
    motores.parar();
    return;
  }

  BNODataPacket pacoteBNO{};
  uint32_t recebidoBNOEmMs = 0;
  if (obterQuadroBNO(pacoteBNO, recebidoBNOEmMs)) {
    if (!sequenciaBNOIniciada ||
        (int16_t)(pacoteBNO.sequence - ultimaSequenciaBNO) > 0) {
      ultimaSequenciaBNO = pacoteBNO.sequence;
      sequenciaBNOIniciada = true;
      ultimoPacoteBNO = pacoteBNO;
      if (!referenciaRumoDefinida) {
        rumoDesejadoGraus = pacoteBNO.headingDegrees;
      }
      referenciaRumoDefinida = true;
      pacoteBNOInicialRecebido = true;
      ultimoRecebimentoBNOMs = recebidoBNOEmMs;
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

  if (quadroNovo) {
    imprimirDadosIR(ultimoPacote);
  }

  if (!pacoteInicialRecebido) {
    informarBloqueioMovimento("aguardando primeiro pacote IR valido.");
    motores.parar();
    return;
  }
  if ((uint32_t)(agora - ultimoRecebimentoMs) > TIMEOUT_PACOTE_MS) {
    informarBloqueioMovimento("pacotes IR expiraram.");
    motores.parar();
    return;
  }
  if (!bnoTelemetriaValida || !referenciaRumoDefinida) {
    informarBloqueioMovimento("aguardando telemetria BNO valida e recente.");
    motores.parar();
    return;
  }

  if (quadroNovo) {
    ultimaCorrecaoRumo = calcularCorrecaoRumo(
        ultimaBolaDetectada,
        ultimoPacoteBNO.headingDegrees,
        ultimoPacoteBNO.gyroZDegreesPerSecond);
  }
  decidirMovimento(ultimaBolaDetectada, ultimaDirecaoBola, ultimaCorrecaoRumo,
                   quadroNovo);
}
