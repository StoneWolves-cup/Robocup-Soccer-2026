#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <IRProtocol.h>
#include <Omni.h>
#include <Motores.h>
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
constexpr uint32_t TEMPO_INICIAL_DIAGNOSTICO_MS = 1000;
constexpr uint32_t INTERVALO_LOG_FALHA_MS = 1000;
constexpr float POTENCIA_MAXIMA_MOTORES = 0.70f;
constexpr float VELOCIDADE_SEGUIR = 1.0f;
constexpr float KP_CORRECAO_RUMO = 0.0015f;
constexpr float KD_CORRECAO_RUMO = 0.0020f;
constexpr float ZONA_MORTA_RUMO_GRAUS = 2.0f;
constexpr float ROTACAO_MAXIMA_CORRECAO = 0.30f;

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
portMUX_TYPE travaContadoresRX = portMUX_INITIALIZER_UNLOCKED;
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
uint32_t ultimoPacoteUARTMs = 0;
bool pacoteUARTRecebido = false;
bool ledUARTVerde = false;
uint32_t inicioDiagnosticoUARTMs = 0;
uint32_t ultimaFalhaUARTMs = 0;
uint32_t ultimoRelatorioPacotesMs = 0;
uint32_t bytesUARTRecebidos = 0;
uint32_t pacotesIRRecebidos = 0;
uint32_t pacotesBNORecebidos = 0;
uint32_t pacotesUltraRecebidos = 0;
struct EstadoDiagnostico {
  bool reportado = false;
  bool funcionando = false;
};
EstadoDiagnostico estadoLinkUART;
EstadoDiagnostico estadoIRUART;
EstadoDiagnostico estadoBNOUART;
EstadoDiagnostico estadoUltraUART;
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

void registrarFalhaUART(const char *motivo) {
  const uint32_t agora = millis();
  if (ultimaFalhaUARTMs != 0 &&
      (uint32_t)(agora - ultimaFalhaUARTMs) < INTERVALO_LOG_FALHA_MS) return;
  ultimaFalhaUARTMs = agora;
  Serial.printf("UART RX: %s\n", motivo);
}

void informarEstadoCanal(const char *nome, bool jaRecebeu,
                         uint32_t ultimoRecebimentoMs, uint32_t timeoutMs,
                         EstadoDiagnostico &estado, uint32_t agora) {
  const bool funcionando = jaRecebeu &&
      (uint32_t)(agora - ultimoRecebimentoMs) <= timeoutMs;
  const bool prazoInicialExpirou =
      (uint32_t)(agora - inicioDiagnosticoUARTMs) >= TEMPO_INICIAL_DIAGNOSTICO_MS;
  if (!funcionando && !jaRecebeu && !prazoInicialExpirou) return;
  if (estado.reportado && estado.funcionando == funcionando) return;

  estado.reportado = true;
  estado.funcionando = funcionando;
  Serial.printf("Canal %s UART: %s\n", nome,
                funcionando ? "OK" : "sem dados recentes");
}

void atualizarDiagnosticoUART(uint32_t agora) {
  const bool linkAtivo = pacoteUARTRecebido &&
      (uint32_t)(agora - ultimoPacoteUARTMs) <= TIMEOUT_PACOTE_MS;
  const bool prazoInicialExpirou =
      (uint32_t)(agora - inicioDiagnosticoUARTMs) >= TIMEOUT_PACOTE_MS;
  if (linkAtivo || prazoInicialExpirou) {
    if (!estadoLinkUART.reportado || estadoLinkUART.funcionando != linkAtivo) {
      estadoLinkUART.reportado = true;
      estadoLinkUART.funcionando = linkAtivo;
      Serial.printf("Link UART: %s\n", linkAtivo ? "OK" : "sem dados recentes");
    }
  }

  informarEstadoCanal("IR", pacoteInicialRecebido, ultimoRecebimentoMs,
                      TIMEOUT_PACOTE_MS, estadoIRUART, agora);
  informarEstadoCanal("BNO055", pacoteBNOInicialRecebido,
                      ultimoRecebimentoBNOMs, TIMEOUT_BNO_MS,
                      estadoBNOUART, agora);
  informarEstadoCanal("ultrassonico", pacoteUltraInicialRecebido,
                      ultimoRecebimentoUltraMs, TIMEOUT_PACOTE_MS,
                      estadoUltraUART, agora);

}

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

void registrarPacoteRecebido(uint16_t magic) {
  portENTER_CRITICAL(&travaContadoresRX);
  if (magic == IR_PACKET_MAGIC) {
    ++pacotesIRRecebidos;
  } else if (magic == BNO_PACKET_MAGIC) {
    ++pacotesBNORecebidos;
  } else if (magic == ULTRA_PACKET_MAGIC) {
    ++pacotesUltraRecebidos;
  }
  portEXIT_CRITICAL(&travaContadoresRX);
}

void relatarPacotesRecebidos(uint32_t agora) {
  if ((uint32_t)(agora - ultimoRelatorioPacotesMs) < 1000) return;
  ultimoRelatorioPacotesMs = agora;

  uint32_t ir, bno, ultra;
  portENTER_CRITICAL(&travaContadoresRX);
  ir = pacotesIRRecebidos;
  bno = pacotesBNORecebidos;
  ultra = pacotesUltraRecebidos;
  pacotesIRRecebidos = 0;
  pacotesBNORecebidos = 0;
  pacotesUltraRecebidos = 0;
  portEXIT_CRITICAL(&travaContadoresRX);

  const uint32_t bytes = bytesUARTRecebidos;
  bytesUARTRecebidos = 0;
  Serial.printf("UART RX/s: bytes=%lu IR=%lu BNO=%lu ultra=%lu\n",
                static_cast<unsigned long>(bytes),
                static_cast<unsigned long>(ir),
                static_cast<unsigned long>(bno),
                static_cast<unsigned long>(ultra));
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
    registrarPacoteRecebido(IR_PACKET_MAGIC);
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
    registrarPacoteRecebido(BNO_PACKET_MAGIC);
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
    registrarPacoteRecebido(ULTRA_PACKET_MAGIC);
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
    ++bytesUARTRecebidos;
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

float calcularCorrecaoRumo(float heading, float gyroZ) {
  float erro = erroAngular(rumoDesejadoGraus, heading);
  if (fabsf(erro) <= ZONA_MORTA_RUMO_GRAUS) erro = 0.0f;

  return constrain(KP_CORRECAO_RUMO * erro - KD_CORRECAO_RUMO * gyroZ,
                   -ROTACAO_MAXIMA_CORRECAO, ROTACAO_MAXIMA_CORRECAO);
}

void aplicarRodas(float x, float y, float rotacao) {
  const ComandosMotores comando = omni.calcular(x, y, rotacao);
  motores.definir(comando.m1 * POTENCIA_MAXIMA_MOTORES,
                  comando.m2 * POTENCIA_MAXIMA_MOTORES,
                  comando.m3 * POTENCIA_MAXIMA_MOTORES,
                  comando.m4 * POTENCIA_MAXIMA_MOTORES);
}

void imprimirDadosIR(const IRActivityPacket &pacote) {
  Serial.print("IR LOW%:");
  for (uint8_t i = 0; i < IR_SENSOR_COUNT; ++i) {
    Serial.printf(" %u", pacote.activity[i]);
    if (i + 1 < IR_SENSOR_COUNT) Serial.print(',');
  }
  const float angulo = pacote.directionTenthsDegrees / 10.0f;
  const float confianca = pacote.confidencePercent / 100.0f;
  Serial.printf(" | bola=%s | direcao=%.1f | confianca=%.2f\n",
                pacote.ballDetected ? "SIM" : "NAO", angulo, confianca);
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
  inicioDiagnosticoUARTMs = millis();
  ultimoRelatorioPacotesMs = inicioDiagnosticoUARTMs;
  motores.begin();
  motores.parar();
  IHM_begin();

  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.printf("Body iniciado: UART1 %lu baud, 8N1, RX=%d TX=%d\n",
                static_cast<unsigned long>(UART_BAUD), UART_RX_PIN,
                UART_TX_PIN);
}

void loop() {
  processarUART();
  relatarPacotesRecebidos(millis());
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
          Serial.printf("Referencia de rumo definida: %.2f graus\n",
                        rumoDesejadoGraus);
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
  atualizarDiagnosticoUART(agora);

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
        ultimoPacoteBNO.headingDegrees,
        ultimoPacoteBNO.gyroZDegreesPerSecond);
    decidirMovimento(ultimaBolaDetectada, ultimaDirecaoBola, correcaoRumo);
    Serial.printf("Rumo=%.2f alvo=%.2f gyroZ=%.2f correcao=%.3f\n",
                  ultimoPacoteBNO.headingDegrees, rumoDesejadoGraus,
                  ultimoPacoteBNO.gyroZDegreesPerSecond, correcaoRumo);
  }

}
