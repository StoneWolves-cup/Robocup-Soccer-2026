#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <IRProtocol.h>
#include <Omni.h>
#include <Motores.h>
#include <FuzzyPID.h>
#include <LDRFixed.h>
#include <ConfigPinos.h>
#include <freertos/FreeRTOS.h>
#include <esp_system.h>

void IHM_begin();
void IHM_update();
void IHM_conectarLDR(LDRFixed *ldr);
void IHM_atualizarTelemetria(float headingDegrees, float gyroZDegreesPerSecond,
                             bool bnoValido,
                             const float distanciasUltra[4], uint8_t mascaraUltra,
                             bool ultraValido,
                             const uint8_t atividadesIR[IR_SENSOR_COUNT],
                             float direcaoIR,
                             bool bolaDetectada, bool irValido);
void IHM_atualizarLDR(const bool regioesBrancas[4]);
bool IHM_testeTSSPAtivo();
void IHM_configurarRoboDefesa();
bool IHM_estrategiaDefesaAtiva();

namespace {
constexpr uint32_t USB_SERIAL_BAUD = 460800;
constexpr uint32_t UART_BAUD = 115200;
constexpr uint32_t TIMEOUT_PACOTE_MS = 250;
constexpr uint32_t INTERVALO_DIAGNOSTICO_UART_MS = 1000;
constexpr size_t TAMANHO_BUFFER_RX_UART = 4096;
constexpr bool USAR_LDR = true;
constexpr bool USAR_BNO = true;
constexpr bool USAR_IR = true;
constexpr bool USAR_ULTRA = true;
// GPIOs reservados para a UART, fora dos pinos atuais de motores e da IHM.
constexpr int UART_RX_PIN = ConfigPinos::Geral::UART1::RX;
constexpr int UART_TX_PIN = ConfigPinos::Geral::UART1::TX;
constexpr uint32_t TIMEOUT_BNO_MS = 250;
constexpr float POTENCIA_MAXIMA_MOTORES = 0.80f;
constexpr float VELOCIDADE_CORRECAO_LINHA = 0.70f;
constexpr float KP_CORRECAO_RUMO = 0.0044f;
constexpr float KI_CORRECAO_RUMO = 0.0000022f;
constexpr float KD_CORRECAO_RUMO = 0.00012f;
constexpr float ESCALA_ERRO_FUZZY_GRAUS = 18.5414f;
constexpr float ESCALA_GYRO_FUZZY_GRAUS_POR_SEGUNDO = 74.1656f;
constexpr float ZONA_MORTA_RUMO_GRAUS = 1.2360f;
constexpr float ROTACAO_MAXIMA_CORRECAO = 0.80f;
constexpr float LIMITE_INTEGRAL_RUMO = 300.0f;
constexpr uint8_t ULTRA_INDICE_ESQUERDO = 0;  // ECHO1, GPIO11
constexpr uint8_t ULTRA_INDICE_DIREITO = 1;   // ECHO2, GPIO12
constexpr uint8_t ULTRA_INDICE_TRASEIRO = 2;  // ECHO3, GPIO13
constexpr uint32_t DURACAO_MANOBRA_DUPLA_MS = 400;
constexpr float POTENCIA_RECUO_APOS_MANOBRA = 0.50f;

enum class ManobraDupla { Inativa, Avancando, AguardandoUltra, Desviando };
ManobraDupla manobraDupla = ManobraDupla::Inativa;
uint32_t inicioManobraDuplaMs = 0;
uint32_t fimAvancoDuploMs = 0;
float xDesvioDuplo = 0.0f;
bool deteccaoDuplaPendenteLimpeza = false;
bool buscandoLinhaAposManobra = false;

class Defesa {
public:
  enum class Movimento { Parado, Tras, Frente, Esquerda, Direita };

  void reiniciar() {
    buscandoParaFrente_ = false;
    interromperMovimento();
  }

  void interromperMovimento() {
    ultimoMovimentoNaLinha_ = Movimento::Parado;
    recuperacaoAtiva_ = false;
  }

  void registrarMovimentoAplicado(bool linhaInferior, Movimento movimento) {
    ultimoMovimentoNaLinha_ = linhaInferior && !recuperacaoAtiva_
        ? movimento : Movimento::Parado;
  }

  bool recuperacaoAtiva() const { return recuperacaoAtiva_; }

  static Movimento movimentoBola(bool bolaDetectada, float anguloBolaGraus) {
    if (!bolaDetectada) return Movimento::Parado;
    const float lateral = sinf(anguloBolaGraus * 0.017453292519943295f);
    if (lateral > 0.08715574274765817f) return Movimento::Esquerda;
    if (lateral < -0.08715574274765817f) return Movimento::Direita;
    return Movimento::Parado;
  }

  Movimento decidir(bool linhaInferior, bool ultraValido, float distanciaCm,
                    bool bolaDetectada, float anguloBolaGraus,
                    uint32_t agoraMs) {
    if (recuperacaoAtiva_) {
      if (static_cast<uint32_t>(agoraMs - inicioRecuperacaoMs_) <
          DURACAO_RECUPERACAO_MS) {
        return movimentoRecuperacao_;
      }
      recuperacaoAtiva_ = false;
      ultimoMovimentoNaLinha_ = Movimento::Parado;
    }

    if (!linhaInferior &&
        (ultimoMovimentoNaLinha_ == Movimento::Esquerda ||
         ultimoMovimentoNaLinha_ == Movimento::Direita)) {
      movimentoRecuperacao_ = ultimoMovimentoNaLinha_ == Movimento::Esquerda
          ? Movimento::Direita : Movimento::Esquerda;
      ultimoMovimentoNaLinha_ = Movimento::Parado;
      inicioRecuperacaoMs_ = agoraMs;
      recuperacaoAtiva_ = true;
      return movimentoRecuperacao_;
    }

    if (linhaInferior) {
      buscandoParaFrente_ = false;
      return movimentoBola(bolaDetectada, anguloBolaGraus);
    }

    if (!ultraValido || !isfinite(distanciaCm) ||
        distanciaCm < 2.0f || distanciaCm > 430.0f) {
      return Movimento::Parado;
    }
    if (distanciaCm <= 30.0f) buscandoParaFrente_ = true;
    return buscandoParaFrente_ ? Movimento::Frente : Movimento::Tras;
  }

private:
  static constexpr uint32_t DURACAO_RECUPERACAO_MS = 600;
  bool buscandoParaFrente_ = false;
  bool recuperacaoAtiva_ = false;
  uint32_t inicioRecuperacaoMs_ = 0;
  Movimento movimentoRecuperacao_ = Movimento::Parado;
  Movimento ultimoMovimentoNaLinha_ = Movimento::Parado;
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
LDRFixed ldrFixed(ConfigLDR::Defesa);
Defesa defesa;
FuzzyPID pidCorrecaoRumo(
    KP_CORRECAO_RUMO, KI_CORRECAO_RUMO, KD_CORRECAO_RUMO,
    -ROTACAO_MAXIMA_CORRECAO, ROTACAO_MAXIMA_CORRECAO,
    ESCALA_ERRO_FUZZY_GRAUS, ESCALA_GYRO_FUZZY_GRAUS_POR_SEGUNDO);

void atualizarLedUART() {
  const uint32_t agoraMs = millis();
  const bool algumSensorUART = USAR_IR || USAR_BNO || USAR_ULTRA;
  const bool comunicacaoConfirmada = algumSensorUART &&
      (!USAR_IR || (pacoteIRUARTRecebido &&
                    (uint32_t)(agoraMs - ultimoPacoteIRUARTMs) <=
                        TIMEOUT_PACOTE_MS)) &&
      (!USAR_BNO || (pacoteBNOUARTRecebido &&
                     (uint32_t)(agoraMs - ultimoPacoteBNOUARTMs) <=
                         TIMEOUT_BNO_MS)) &&
      (!USAR_ULTRA || (pacoteUltraInicialRecebido &&
                       (uint32_t)(agoraMs - ultimoRecebimentoUltraMs) <=
                           TIMEOUT_PACOTE_MS));
  if (comunicacaoConfirmada == ledUARTVerde) return;

  rgbLedWrite(RGB_BUILTIN,
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

const char *nomeMotivoReset(esp_reset_reason_t motivo) {
  switch (motivo) {
    case ESP_RST_UNKNOWN: return "desconhecido";
    case ESP_RST_POWERON: return "ligacao/alimentacao";
    case ESP_RST_EXT: return "pino externo";
    case ESP_RST_SW: return "reinicio por software";
    case ESP_RST_PANIC: return "panic/crash";
    case ESP_RST_INT_WDT: return "watchdog de interrupcao";
    case ESP_RST_TASK_WDT: return "watchdog de tarefa";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_DEEPSLEEP: return "retorno de deep sleep";
    case ESP_RST_BROWNOUT: return "brownout/tensao baixa";
    case ESP_RST_SDIO: return "SDIO";
    default: return "motivo nao mapeado";
  }
}

void imprimirDadosIR(const IRActivityPacket &pacote) {
  if (USAR_BNO && pacote.ballDetected) {
    Serial.printf("Direcao da bola: %.1f graus | Angulo BNO: %.1f graus\n",
                  pacote.directionTenthsDegrees / 10.0f,
                  ultimoPacoteBNO.headingDegrees);
  } else if (USAR_BNO) {
    Serial.printf("Direcao da bola: nao detectada | Angulo BNO: %.1f graus\n",
                  ultimoPacoteBNO.headingDegrees);
  } else if (pacote.ballDetected) {
    Serial.printf("Direcao da bola: %.1f graus\n",
                  pacote.directionTenthsDegrees / 10.0f);
  } else {
    Serial.println("Direcao da bola: nao detectada");
  }
}

} // namespace

void setup() {
  rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);
  Serial.begin(USB_SERIAL_BAUD);
  const esp_reset_reason_t motivoReset = esp_reset_reason();
  Serial.printf("Motivo do reset: %d (%s)\n", static_cast<int>(motivoReset),
                nomeMotivoReset(motivoReset));
  const size_t tamanhoBufferUARTConfigurado =
      Serial1.setRxBufferSize(TAMANHO_BUFFER_RX_UART);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  pidCorrecaoRumo.setIntegralLimit(LIMITE_INTEGRAL_RUMO);
  ultimoCalculoPIDUs = micros();
  motores.begin();
  motores.parar();
  if (USAR_LDR) {
    ldrFixed.begin();
    IHM_conectarLDR(&ldrFixed);
  }
  Serial.println("Iniciando IHM.");
  IHM_configurarRoboDefesa();
  IHM_begin();
  Serial.println("IHM inicializada; entrando no loop.");

  Serial.println("Body UART1: 115200 baud, 8N1, RX=18 TX=17.");
  if (tamanhoBufferUARTConfigurado < TAMANHO_BUFFER_RX_UART) {
    Serial.printf("ERRO: buffer RX UART1 configurado com %u de %u bytes.\n",
                  static_cast<unsigned>(tamanhoBufferUARTConfigurado),
                  static_cast<unsigned>(TAMANHO_BUFFER_RX_UART));
  }
  Serial.println(
      "LED verde requer pacotes validos e recentes dos sensores UART habilitados.");
  Serial.printf("Sensores: IR=%s, BNO=%s, ultra=%s, LDR=%s\n",
                USAR_IR ? "ligado" : "desligado",
                USAR_BNO ? "ligado" : "desligado",
                USAR_ULTRA ? "ligado" : "desligado",
                USAR_LDR ? "ligado" : "desligado");
  Serial.println(USAR_LDR
      ? "LDR habilitado: classificacao por limiares fixos e histerese."
      : "LDR desativado.");
}

void loop() {
  processarUART();
  atualizarLedUART();
  imprimirDiagnosticoUART();
  IHM_update();
  if (USAR_LDR) {
    ldrFixed.update();
    IHM_atualizarLDR(ldrFixed.regioesBrancas());
  }

  BNODataPacket pacoteBNO{};
  uint32_t recebidoBNOEmMs = 0;
  if (obterQuadroBNO(pacoteBNO, recebidoBNOEmMs) && USAR_BNO) {
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
  if (obterQuadroUltra(pacoteUltra, recebidoUltraEmMs) && USAR_ULTRA &&
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
  const bool chegouPacote = obterQuadro(pacote, recebidoEmMs) && USAR_IR;
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
      ultimaBolaDetectada = pacote.ballDetected != 0;
      ultimaDirecaoBola = pacote.directionTenthsDegrees / 10.0f;
      ultimoRecebimentoMs = recebidoEmMs;
      pacoteInicialRecebido = true;
    }
  }

  const bool bnoTelemetriaValida = USAR_BNO && pacoteBNOInicialRecebido &&
      (uint32_t)(agora - ultimoRecebimentoBNOMs) <= TIMEOUT_BNO_MS;
  const bool ultraTelemetriaValida = USAR_ULTRA && pacoteUltraInicialRecebido &&
      (uint32_t)(agora - ultimoRecebimentoUltraMs) <= TIMEOUT_PACOTE_MS;
  const bool irTelemetriaValida = USAR_IR && pacoteInicialRecebido &&
      (uint32_t)(agora - ultimoRecebimentoMs) <= TIMEOUT_PACOTE_MS;
  IHM_atualizarTelemetria(
      ultimoPacoteBNO.headingDegrees, ultimoPacoteBNO.gyroZDegreesPerSecond,
      bnoTelemetriaValida, ultimoPacoteUltra.distanceCm,
      ultimoPacoteUltra.validMask, ultraTelemetriaValida, ultimoPacote.activity,
      ultimaDirecaoBola, ultimaBolaDetectada, irTelemetriaValida);

  if (quadroNovo) {
    imprimirDadosIR(ultimoPacote);
  }

  if (!IHM_estrategiaDefesaAtiva() || IHM_testeTSSPAtivo()) {
    defesa.reiniciar();
    manobraDupla = ManobraDupla::Inativa;
    deteccaoDuplaPendenteLimpeza = false;
    buscandoLinhaAposManobra = false;
    pidCorrecaoRumo.reset();
    ultimoCalculoPIDUs = micros();
    motores.parar();
    return;
  }

  if (!USAR_LDR) {
    informarBloqueioMovimento("defesa requer o sensor LDR habilitado.");
    defesa.interromperMovimento();
    motores.parar();
    return;
  }
  const bool linhaInferior = ldrFixed.regiaoBranca(LDRFixed::Inferior);
  const float distanciaTraseira =
      ultimoPacoteUltra.distanceCm[ULTRA_INDICE_TRASEIRO];
  const bool traseiraValida = ultraTelemetriaValida &&
      (ultimoPacoteUltra.validMask & (1U << ULTRA_INDICE_TRASEIRO)) &&
      isfinite(distanciaTraseira) &&
      distanciaTraseira >= 2.0f && distanciaTraseira <= 430.0f;
  if (USAR_BNO &&
      (!bnoTelemetriaValida || !referenciaRumoDefinida)) {
    informarBloqueioMovimento("aguardando telemetria BNO valida e recente.");
    defesa.interromperMovimento();
    manobraDupla = ManobraDupla::Inativa;
    deteccaoDuplaPendenteLimpeza = false;
    motores.parar();
    return;
  }

  const bool linhaEsquerda = ldrFixed.regiaoBranca(LDRFixed::Esquerda);
  const bool linhaDireita = ldrFixed.regiaoBranca(LDRFixed::Direita);
  if (!linhaEsquerda && !linhaDireita) deteccaoDuplaPendenteLimpeza = false;
  if (manobraDupla == ManobraDupla::Inativa && linhaEsquerda && linhaDireita &&
      !deteccaoDuplaPendenteLimpeza) {
    defesa.interromperMovimento();
    manobraDupla = ManobraDupla::Avancando;
    inicioManobraDuplaMs = agora;
    deteccaoDuplaPendenteLimpeza = true;
    Serial.println("LDRs laterais brancos: avancando por 400 ms.");
  }

  if (manobraDupla == ManobraDupla::Avancando &&
      (uint32_t)(agora - inicioManobraDuplaMs) >= DURACAO_MANOBRA_DUPLA_MS) {
    fimAvancoDuploMs = agora;
    manobraDupla = ManobraDupla::AguardandoUltra;
    motores.parar();
    Serial.println("Avanco concluido; aguardando ultrassons laterais.");
  }
  if (manobraDupla == ManobraDupla::AguardandoUltra) {
    const float esquerda = ultimoPacoteUltra.distanceCm[ULTRA_INDICE_ESQUERDO];
    const float direita = ultimoPacoteUltra.distanceCm[ULTRA_INDICE_DIREITO];
    const uint8_t mascaraLaterais =
        (1U << ULTRA_INDICE_ESQUERDO) | (1U << ULTRA_INDICE_DIREITO);
    if (!ultraTelemetriaValida ||
        static_cast<int32_t>(ultimoRecebimentoUltraMs - fimAvancoDuploMs) <= 0 ||
        (ultimoPacoteUltra.validMask & mascaraLaterais) != mascaraLaterais ||
        !isfinite(esquerda) || esquerda < 2.0f || esquerda > 430.0f ||
        !isfinite(direita) || direita < 2.0f || direita > 430.0f) {
      informarBloqueioMovimento("aguardando ultrassons laterais validos apos o avanco.");
      motores.parar();
      return;
    }
    if (esquerda == direita) {
      informarBloqueioMovimento("ultrassons laterais com distancias iguais.");
      motores.parar();
      return;
    }
    xDesvioDuplo = esquerda < direita
        ? -VELOCIDADE_CORRECAO_LINHA : VELOCIDADE_CORRECAO_LINHA;
    inicioManobraDuplaMs = agora;
    manobraDupla = ManobraDupla::Desviando;
    Serial.printf("Desvio por 400 ms: esquerda=%.1f cm, direita=%.1f cm.\n",
                  esquerda, direita);
  }
  if (manobraDupla == ManobraDupla::Desviando &&
      (uint32_t)(agora - inicioManobraDuplaMs) >= DURACAO_MANOBRA_DUPLA_MS) {
    manobraDupla = ManobraDupla::Inativa;
    buscandoLinhaAposManobra = true;
    Serial.println("Desvio concluido; retomando alinhamento e busca da linha.");
  }
  if (manobraDupla != ManobraDupla::Inativa) {
    ultimaCorrecaoRumo = USAR_BNO
        ? calcularCorrecaoRumo(true, ultimoPacoteBNO.headingDegrees,
                               ultimoPacoteBNO.gyroZDegreesPerSecond)
        : 0.0f;
    informarMovimentoLiberado();
    aplicarRodas(manobraDupla == ManobraDupla::Desviando ? xDesvioDuplo : 0.0f,
                 manobraDupla == ManobraDupla::Avancando
                     ? VELOCIDADE_CORRECAO_LINHA : 0.0f,
                 ultimaCorrecaoRumo);
    return;
  }

  if (linhaEsquerda != linhaDireita) {
    defesa.interromperMovimento();
    ultimaCorrecaoRumo = USAR_BNO
        ? calcularCorrecaoRumo(true, ultimoPacoteBNO.headingDegrees,
                               ultimoPacoteBNO.gyroZDegreesPerSecond)
        : 0.0f;
    const float x = linhaEsquerda
        ? -VELOCIDADE_CORRECAO_LINHA : VELOCIDADE_CORRECAO_LINHA;
    if (quadroNovo) {
      Serial.printf("Fuga LDR lateral: linha=%s | vetor=(%.3f, 0) | "
                    "rotacao=%.3f\n",
                    linhaEsquerda ? "esquerda" : "direita", x,
                    ultimaCorrecaoRumo);
    }
    informarMovimentoLiberado();
    aplicarRodas(x, 0.0f, ultimaCorrecaoRumo);
    return;
  }

  const Defesa::Movimento movimento = defesa.decidir(
      linhaInferior, traseiraValida, distanciaTraseira,
      linhaInferior && irTelemetriaValida && ultimaBolaDetectada,
      ultimaDirecaoBola, agora);
  const bool recuperacaoAtiva = defesa.recuperacaoAtiva();
  if (linhaInferior) buscandoLinhaAposManobra = false;

  if (!recuperacaoAtiva && !linhaInferior && !traseiraValida) {
    informarBloqueioMovimento("aguardando ultrassom traseiro valido e recente.");
    defesa.interromperMovimento();
    motores.parar();
    return;
  }
  if (!recuperacaoAtiva && linhaInferior && !irTelemetriaValida) {
    informarBloqueioMovimento("aguardando telemetria IR valida e recente.");
    defesa.interromperMovimento();
    motores.parar();
    return;
  }

  if (!recuperacaoAtiva && linhaInferior && !ultimaBolaDetectada) {
    informarBloqueioMovimento("bola nao detectada pelo IR.");
    defesa.interromperMovimento();
    pidCorrecaoRumo.reset();
    ultimoCalculoPIDUs = micros();
    motores.parar();
    return;
  }

  float x = 0.0f;
  float y = 0.0f;
  switch (movimento) {
    case Defesa::Movimento::Tras:
      y = buscandoLinhaAposManobra
          ? -POTENCIA_RECUO_APOS_MANOBRA / POTENCIA_MAXIMA_MOTORES
          : -VELOCIDADE_CORRECAO_LINHA;
      break;
    case Defesa::Movimento::Frente: y = VELOCIDADE_CORRECAO_LINHA; break;
    case Defesa::Movimento::Esquerda: x = VELOCIDADE_CORRECAO_LINHA; break;
    case Defesa::Movimento::Direita: x = -VELOCIDADE_CORRECAO_LINHA; break;
    case Defesa::Movimento::Parado: break;
  }
  if (buscandoLinhaAposManobra && !linhaInferior && irTelemetriaValida) {
    const Defesa::Movimento lateral =
        Defesa::movimentoBola(ultimaBolaDetectada, ultimaDirecaoBola);
    if (lateral == Defesa::Movimento::Esquerda) x = VELOCIDADE_CORRECAO_LINHA;
    if (lateral == Defesa::Movimento::Direita) x = -VELOCIDADE_CORRECAO_LINHA;
  }
  ultimaCorrecaoRumo = USAR_BNO
      ? calcularCorrecaoRumo(true, ultimoPacoteBNO.headingDegrees,
                             ultimoPacoteBNO.gyroZDegreesPerSecond)
      : 0.0f;
  if (quadroNovo) {
    Serial.printf("Defesa: recuperacao=%s | linha inferior=%s | traseira=%s%.1f cm | "
                  "bola=%.1f graus | vetor=(%.3f, %.3f) | rotacao=%.3f\n",
                  recuperacaoAtiva ? "sim" : "nao",
                  linhaInferior ? "branco" : "verde",
                  traseiraValida ? "" : "invalida ", distanciaTraseira,
                  ultimaDirecaoBola, x, y, ultimaCorrecaoRumo);
  }
  informarMovimentoLiberado();
  aplicarRodas(x, y, ultimaCorrecaoRumo);
  defesa.registrarMovimentoAplicado(linhaInferior, movimento);
}
