#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <IRProtocol.h>
#include <Omni.h>
#include <Motores.h>
#include <PID.h>
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
bool IHM_testeBNOMoveAtivo();
bool IHM_testeBNOMoveExecutando();
void IHM_pararTesteBNOMove();
void IHM_obterGanhosPID(float &kp, float &ki, float &kd);
bool IHM_estrategiaAtaqueAtiva();
bool IHM_estrategiaDefesaAtiva();

// O Body valida a telemetria recebida antes de mover. Sensores desligaveis por
// configuracao continuam representados por flags, para que bloqueios explicitos
// sejam preferidos a assumir que um pacote ausente contem uma leitura segura.
namespace { 
constexpr uint32_t USB_SERIAL_BAUD = 460800;
constexpr uint32_t UART_BAUD = 115200;
constexpr uint32_t TIMEOUT_PACOTE_MS = 250;
constexpr uint32_t INTERVALO_DIAGNOSTICO_UART_MS = 1000;
constexpr size_t TAMANHO_BUFFER_RX_UART = 4096;
// Permitem isolar hardware durante manutencao; estrategia exige os sensores
// explicitamente verificados mais abaixo antes de liberar os motores.
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
constexpr float ANGULO_SETOR_FRONTAL_GRAUS = 45.0f;
constexpr float ANGULO_LATERAL_GRAUS = 90.0f;
constexpr float ANGULO_TRASEIRO_GRAUS = 180.0f;
constexpr float ANGULO_MAXIMO_CURVA_GRAUS = 35.0f;
constexpr float ROTACAO_MAXIMA_BNO_MOVE = 0.80f;
constexpr float ZONA_MORTA_BNO_MOVE_GRAUS = 0.70f;
constexpr float PASSO_ROTACAO_BNO_MOVE = 0.005f;
constexpr uint32_t INTERVALO_PID_BNO_MOVE_US = 10000;
constexpr float ZONA_MORTA_RUMO_GRAUS = 1.2360f;
constexpr float ROTACAO_MAXIMA_CORRECAO = 0.80f;
constexpr float LIMITE_INTEGRAL_RUMO = 300.0f;
constexpr uint32_t TEMPO_MINIMO_FUGA_LDR_MS = 300;
constexpr uint32_t TEMPO_LINHA_LIMPA_MS = 120;
constexpr uint32_t JANELA_CONTINUAR_CORRECAO_LDR_MS = 3000;
constexpr float MARGEM_DOMINANCIA_LATERAL = 0.001f;
// Indices do quadro UltraDataPacket; mantenha alinhados com o chicote fisico.
constexpr uint8_t ULTRA_INDICE_ESQUERDO = 0;  // ECHO1, GPIO11
constexpr uint8_t ULTRA_INDICE_DIREITO = 1;   // ECHO2, GPIO12
constexpr uint8_t ULTRA_INDICE_TRASEIRO = 2;  // ECHO3, GPIO13
constexpr uint8_t ULTRA_INDICE_FRONTAL = 3;   // ECHO4, GPIO14
constexpr float DISTANCIA_CONTORNO_ULTRA_CM = 15.0f;
constexpr float ANGULO_CONTORNO_LDR_GRAUS = 22.0f;
// Defesa: rumo fixo na bussola (BNO055 em NDOF, 0 graus = norte), distancia
// constante ao fundo pelo ultrassom traseiro e deslocamento lateral
// acompanhando a bola.
constexpr float RUMO_DEFESA_GRAUS = 0.0f;
constexpr float DISTANCIA_DEFESA_CM = 40.0f;
constexpr float ZONA_MORTA_DISTANCIA_DEFESA_CM = 2.0f;
constexpr float GANHO_DISTANCIA_DEFESA = 0.04f;
constexpr float ZONA_MORTA_BOLA_DEFESA = 0.03489949670250097f;  // sen(2 graus)
constexpr float GANHO_LATERAL_BOLA_DEFESA = 3.0f;

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
bool fugaLDRAtiva = false;
bool temporizadorLinhaLimpaAtivo = false;
uint32_t inicioFugaLDRMs = 0;
uint32_t inicioLinhaLimpaMs = 0;
float ultimoVetorFugaX = 0.0f;
float ultimoVetorFugaY = 0.0f;
float ultimaCorrecaoPerifericaLDRX = 0.0f;
float ultimaCorrecaoPerifericaLDRY = 0.0f;
uint32_t ultimaDeteccaoPerifericaLDRMs = 0;
bool correcaoPerifericaLDRValida = false;
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
LDRFixed ldrFixed;
PID pidRumo(PIDConfig::KP_PADRAO, PIDConfig::KI_PADRAO, PIDConfig::KD_PADRAO,
            -ROTACAO_MAXIMA_BNO_MOVE, ROTACAO_MAXIMA_BNO_MOVE);
float rumoReferenciaBnoMove = 0.0f;
float rotacaoBnoMove = 0.0f;
uint32_t ultimoCalculoBnoMoveUs = 0;
bool controleBnoMoveIniciado = false;

// Acende verde apenas quando todos os sensores UART habilitados publicaram
// recentemente; timeout transforma falha silenciosa de comunicacao em indicacao visivel.
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

// Registra recepcao recente de cada fluxo para diagnostico e watchdog visual.
void confirmarPacoteIRUART() {
  ultimoPacoteIRUARTMs = millis();
  pacoteIRUARTRecebido = true;
  atualizarLedUART();
}

// Atualiza o mesmo indicador para o canal BNO.
void confirmarPacoteBNOUART() {
  ultimoPacoteBNOUARTMs = millis();
  pacoteBNOUARTRecebido = true;
  atualizarLedUART();
}

// Acumula erro e preserva o ultimo motivo para relatorio periodico, evitando
// imprimir uma linha por byte/quadro defeituoso.
void registrarFalhaUART(const char *motivo) {
  ++quadrosInvalidosDesdeDiagnostico;
  ultimoErroUART = motivo;
}

// Valida tamanho, versao, checksum e dominio dos dados antes de publicar cada
// tipo de pacote sob secao critica; o loop nunca consome um quadro parcial.
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

// O magic identifica candidatos ao inicio de quadro no fluxo sem delimitadores.
bool byteEhInicioDeQuadro(uint8_t byteRecebido) {
  return byteRecebido == static_cast<uint8_t>(IR_PACKET_MAGIC) ||
         byteRecebido == static_cast<uint8_t>(BNO_PACKET_MAGIC) ||
         byteRecebido == static_cast<uint8_t>(ULTRA_PACKET_MAGIC);
}

// Reconstroi quadros a partir do fluxo UART: descarta bytes ate magic, espera
// o tamanho do tipo e avanca um byte em erro para recuperar sincronismo.
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

// Resume trafego e falhas em intervalos fixos; contadores sao zerados apos cada
// amostra para que o proximo log represente somente o periodo seguinte.
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

// Emite bloqueios apenas quando mudam ou quando o periodo de lembrete vence,
// mantendo diagnostico util sem inundar a serial em cada iteracao do loop.
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

// Fecha o episodio de bloqueio no log quando os pre-requisitos voltam a ser validos.
void informarMovimentoLiberado() {
  if (ultimoBloqueioMovimento != nullptr) {
    Serial.println("MOVIMENTO LIBERADO.");
    ultimoBloqueioMovimento = nullptr;
  }
}

// Retira um unico quadro IR do buffer ISR/main dentro de secao critica curta.
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

// Consome snapshot BNO e timestamp atomicos para verificar idade no loop.
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

// Consome snapshot ultrassonico independente do fluxo IR e BNO.
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

// Escolhe o menor erro angular equivalente para que o controlador nao gire
// quase uma volta completa ao cruzar a descontinuidade +/-180 graus.
float erroAngular(float desejado, float atual) {
  float erro = desejado - atual;
  while (erro > 180.0f) erro -= 360.0f;
  while (erro < -180.0f) erro += 360.0f;
  return erro;
}

// Converte qualquer angulo para [0, 360), util para comparacoes de setores.
float normalizarAngulo360(float anguloGraus) {
  anguloGraus = fmodf(anguloGraus, 360.0f);
  return anguloGraus < 0.0f ? anguloGraus + 360.0f : anguloGraus;
}

// Trata os dois lados de zero como o mesmo setor frontal.
bool bolaNoSetorFrontal(float anguloGraus) {
  const float angulo360 = normalizarAngulo360(anguloGraus);
  return angulo360 >= 360.0f - ANGULO_SETOR_FRONTAL_GRAUS ||
         angulo360 <= ANGULO_SETOR_FRONTAL_GRAUS;
}

// Mapeia a direcao da bola em um vetor de translacao continuo: curva ao redor
// da bola nas laterais e aponta para captura quando ela esta a frente.
float calcularAnguloMovimentoJ(float anguloBolaGraus) {
  const float anguloAssinado = erroAngular(anguloBolaGraus, 0.0f);
  const float magnitudeBola = fabsf(anguloAssinado);
  float magnitudeMovimento;

  if (magnitudeBola <= ANGULO_SETOR_FRONTAL_GRAUS) {
    magnitudeMovimento =
        magnitudeBola * ANGULO_TRASEIRO_GRAUS /
        ANGULO_SETOR_FRONTAL_GRAUS;
  } else if (magnitudeBola <= ANGULO_LATERAL_GRAUS) {
    const float progressoCurva =
        (magnitudeBola - ANGULO_SETOR_FRONTAL_GRAUS) /
        (ANGULO_LATERAL_GRAUS - ANGULO_SETOR_FRONTAL_GRAUS);
    magnitudeMovimento =
        ANGULO_TRASEIRO_GRAUS -
        ANGULO_MAXIMO_CURVA_GRAUS * progressoCurva;
  } else {
    const float progressoTraseiro =
        (magnitudeBola - ANGULO_LATERAL_GRAUS) /
        (ANGULO_TRASEIRO_GRAUS - ANGULO_LATERAL_GRAUS);
    magnitudeMovimento =
        ANGULO_TRASEIRO_GRAUS +
        ANGULO_MAXIMO_CURVA_GRAUS * progressoTraseiro;
  }

  return copysignf(magnitudeMovimento, anguloAssinado);
}

// Corrige rumo fixado na primeira leitura valida. O gyro amortece a rotacao e
// a zona morta reduz acionamentos por pequenas variacoes do magnetometro.
float calcularCorrecaoRumo(bool bolaDetectada, float heading, float gyroZ,
                           float rumoAlvoGraus = rumoDesejadoGraus) {
  const uint32_t agoraUs = micros();
  const uint32_t decorridoUs = agoraUs - ultimoCalculoPIDUs;
  ultimoCalculoPIDUs = agoraUs;

  if (!bolaDetectada) {
    pidRumo.reset();
    return 0.0f;
  }

  float kp = PIDConfig::KP_PADRAO;
  float ki = PIDConfig::KI_PADRAO;
  float kd = PIDConfig::KD_PADRAO;
  IHM_obterGanhosPID(kp, ki, kd);
  pidRumo.setGains(kp, ki, kd);

  float erro = erroAngular(rumoAlvoGraus, heading);
  if (fabsf(erro) <= ZONA_MORTA_RUMO_GRAUS) erro = 0.0f;

  const float dt = constrain(decorridoUs / 1000000.0f, 0.001f, 0.1f);
  return constrain(pidRumo.calculate(erro, gyroZ, dt),
                   -ROTACAO_MAXIMA_CORRECAO,
                   ROTACAO_MAXIMA_CORRECAO);
}

// Aplica a cinematica e escala global de potencia propria do perfil do robo.
void aplicarRodas(float x, float y, float rotacao) {
  const ComandosMotores comando = omni.calcular(x, y, rotacao);
  motores.definir(comando.m1 * POTENCIA_MAXIMA_MOTORES,
                  comando.m2 * POTENCIA_MAXIMA_MOTORES,
                  comando.m3 * POTENCIA_MAXIMA_MOTORES,
                  comando.m4 * POTENCIA_MAXIMA_MOTORES);
}

// Mantem o rumo capturado no inicio do teste; qualquer falha de telemetria
// interrompe o giro e exige que o operador inicie novamente.
void executarTesteBNOMove(bool bnoTelemetriaValida) {
  if (!IHM_testeBNOMoveExecutando()) {
    controleBnoMoveIniciado = false;
    rotacaoBnoMove = 0.0f;
    pidRumo.reset();
    motores.parar();
    return;
  }

  if (!bnoTelemetriaValida ||
      !isfinite(ultimoPacoteBNO.headingDegrees) ||
      !isfinite(ultimoPacoteBNO.gyroZDegreesPerSecond)) {
    Serial.println("BNO-Move interrompido: telemetria BNO invalida ou expirada.");
    IHM_pararTesteBNOMove();
    controleBnoMoveIniciado = false;
    rotacaoBnoMove = 0.0f;
    pidRumo.reset();
    motores.parar();
    return;
  }

  float kp = PIDConfig::KP_PADRAO;
  float ki = PIDConfig::KI_PADRAO;
  float kd = PIDConfig::KD_PADRAO;
  IHM_obterGanhosPID(kp, ki, kd);
  pidRumo.setGains(kp, ki, kd);

  if (!controleBnoMoveIniciado) {
    rumoReferenciaBnoMove = ultimoPacoteBNO.headingDegrees;
    controleBnoMoveIniciado = true;
    ultimoCalculoBnoMoveUs = micros();
    rotacaoBnoMove = 0.0f;
    pidRumo.reset();
    motores.parar();
    Serial.printf("BNO-Move iniciado; rumo inicial %.2f graus.\n",
                  rumoReferenciaBnoMove);
    return;
  }

  const uint32_t agoraUs = micros();
  const uint32_t decorridoUs = agoraUs - ultimoCalculoBnoMoveUs;
  if (decorridoUs >= INTERVALO_PID_BNO_MOVE_US) {
    ultimoCalculoBnoMoveUs = agoraUs;
    float erro = erroAngular(rumoReferenciaBnoMove,
                             ultimoPacoteBNO.headingDegrees);
    if (fabsf(erro) <= ZONA_MORTA_BNO_MOVE_GRAUS) erro = 0.0f;

    const float dt = constrain(decorridoUs / 1000000.0f, 0.001f, 0.1f);
    const float alvo = pidRumo.calculate(
        erro, ultimoPacoteBNO.gyroZDegreesPerSecond, dt);
    rotacaoBnoMove += constrain(alvo - rotacaoBnoMove,
                                -PASSO_ROTACAO_BNO_MOVE,
                                PASSO_ROTACAO_BNO_MOVE);
  }

  const ComandosMotores rodas = omni.calcular(0.0f, 0.0f, rotacaoBnoMove);
  motores.definir(rodas.m1, rodas.m2, rodas.m3, rodas.m4);
}

// Nega comandos que apontariam para linha branca detectada na frente/lado
// correspondente; a seguranca da borda prevalece sobre a perseguicao da bola.
bool vetorBloqueadoPeloLDR(float x, float y) {
  if (ldrFixed.regiaoBranca(LDRFixed::Centro)) return true;
  const bool movimentoLateral =
      fabsf(x) > fabsf(y) + MARGEM_DOMINANCIA_LATERAL;
  if (movimentoLateral && x > 0.0f &&
      ldrFixed.regiaoBranca(LDRFixed::Esquerda)) return true;
  if (movimentoLateral && x < 0.0f &&
      ldrFixed.regiaoBranca(LDRFixed::Direita)) return true;
  if (y < 0.0f && ldrFixed.regiaoBranca(LDRFixed::Inferior)) return true;
  return false;
}

// Tenta um pequeno desvio apenas quando um sensor lateral bloqueia o vetor e
// ultrassom recente distingue qual contorno esta livre; ambiguidades nao movem.
bool calcularContornoLDR(float anguloGraus, const UltraDataPacket &ultra,
                         bool ultraRecente, float &anguloContornoGraus) {
  if (!USAR_ULTRA || !ultraRecente) return false;

  const float anguloRad = anguloGraus * DEG_TO_RAD;
  const float xOriginal = sinf(anguloRad);
  const float yOriginal = cosf(anguloRad);
  const bool movimentoLateral =
      fabsf(xOriginal) > fabsf(yOriginal) + MARGEM_DOMINANCIA_LATERAL;
  const bool lateralBloqueado =
      movimentoLateral &&
      ((xOriginal > 0.0f && ldrFixed.regiaoBranca(LDRFixed::Esquerda)) ||
       (xOriginal < 0.0f && ldrFixed.regiaoBranca(LDRFixed::Direita)));
  if (!lateralBloqueado) return false;

  const bool frenteValida =
      ultra.validMask & (1U << ULTRA_INDICE_FRONTAL);
  const bool traseiraValida =
      ultra.validMask & (1U << ULTRA_INDICE_TRASEIRO);
  const bool frenteProxima = frenteValida &&
      ultra.distanceCm[ULTRA_INDICE_FRONTAL] < DISTANCIA_CONTORNO_ULTRA_CM;
  const bool traseiraProxima = traseiraValida &&
      ultra.distanceCm[ULTRA_INDICE_TRASEIRO] < DISTANCIA_CONTORNO_ULTRA_CM;
  if (frenteProxima == traseiraProxima) return false;

  const float deslocamentoGraus = frenteProxima
      ? -ANGULO_CONTORNO_LDR_GRAUS
      : ANGULO_CONTORNO_LDR_GRAUS;
  const float candidatoGraus = anguloGraus + deslocamentoGraus;
  const float candidatoRad = candidatoGraus * DEG_TO_RAD;
  const float xCandidato = sinf(candidatoRad);
  const float yCandidato = cosf(candidatoRad);
  if (vetorBloqueadoPeloLDR(xCandidato, yCandidato)) return false;

  anguloContornoGraus = candidatoGraus;
  return true;
}

// Soma vetores de afastamento de cada regiao branca para escapar em diagonal
// quando varios LDRs detectam a borda ao mesmo tempo.
bool calcularVetorFugaLDR(float &x, float &y, bool &centroDetectado,
                          bool &periferiaDetectada, float &periferiaX,
                          float &periferiaY) {
  x = 0.0f;
  y = 0.0f;
  centroDetectado = ldrFixed.regiaoBranca(LDRFixed::Centro);
  periferiaDetectada = false;
  periferiaX = 0.0f;
  periferiaY = 0.0f;
  bool brancoDetectado = false;

  if (centroDetectado) {
    y -= VELOCIDADE_CORRECAO_LINHA;
    brancoDetectado = true;
  }
  if (ldrFixed.regiaoBranca(LDRFixed::Esquerda)) {
    x -= VELOCIDADE_CORRECAO_LINHA;
    periferiaX -= VELOCIDADE_CORRECAO_LINHA;
    periferiaDetectada = true;
    brancoDetectado = true;
  }
  if (ldrFixed.regiaoBranca(LDRFixed::Direita)) {
    x += VELOCIDADE_CORRECAO_LINHA;
    periferiaX += VELOCIDADE_CORRECAO_LINHA;
    periferiaDetectada = true;
    brancoDetectado = true;
  }
  if (ldrFixed.regiaoBranca(LDRFixed::Inferior)) {
    y += VELOCIDADE_CORRECAO_LINHA;
    periferiaY += VELOCIDADE_CORRECAO_LINHA;
    periferiaDetectada = true;
    brancoDetectado = true;
  }
  return brancoDetectado;
}

// Se centro e periferia detectam a borda simultaneamente, reutiliza por pouco
// tempo a ultima direcao periferica para evitar perder a orientacao de fuga.
void aplicarMemoriaCorrecaoLDR(bool centroDetectado,
                               bool periferiaDetectada, float periferiaX,
                               float periferiaY, float &x, float &y,
                               uint32_t agoraMs) {
  if (periferiaDetectada &&
      (periferiaX != 0.0f || periferiaY != 0.0f)) {
    ultimaCorrecaoPerifericaLDRX = periferiaX;
    ultimaCorrecaoPerifericaLDRY = periferiaY;
    ultimaDeteccaoPerifericaLDRMs = agoraMs;
    correcaoPerifericaLDRValida = true;
  }

  if (!centroDetectado) return;

  const bool memoriaRecente = correcaoPerifericaLDRValida &&
      (uint32_t)(agoraMs - ultimaDeteccaoPerifericaLDRMs) <=
          JANELA_CONTINUAR_CORRECAO_LDR_MS;
  if (memoriaRecente) {
    x = ultimaCorrecaoPerifericaLDRX;
    y = ultimaCorrecaoPerifericaLDRY;
  }
}

// Mantem a fuga ate cumprir tempo minimo e linha limpa estavel; isso evita
// alternar estrategia por uma amostra isolada ou ruido no limite do campo.
bool atualizarFugaLDR(bool linhaDetectada, float x, float y,
                      uint32_t agoraMs) {
  if (linhaDetectada) {
    if (!fugaLDRAtiva) {
      fugaLDRAtiva = true;
      inicioFugaLDRMs = agoraMs;
      Serial.println("Fuga LDR ativada; aguardando sensores indicarem linha limpa.");
    }
    ultimoVetorFugaX = x;
    ultimoVetorFugaY = y;
    temporizadorLinhaLimpaAtivo = false;
    return true;
  }

  if (!fugaLDRAtiva) return false;

  if (!temporizadorLinhaLimpaAtivo) {
    inicioLinhaLimpaMs = agoraMs;
    temporizadorLinhaLimpaAtivo = true;
  }

  const bool tempoMinimoCumprido =
      (uint32_t)(agoraMs - inicioFugaLDRMs) >= TEMPO_MINIMO_FUGA_LDR_MS;
  const bool linhaLimpaEstavel =
      (uint32_t)(agoraMs - inicioLinhaLimpaMs) >= TEMPO_LINHA_LIMPA_MS;
  if (tempoMinimoCumprido && linhaLimpaEstavel) {
    fugaLDRAtiva = false;
    temporizadorLinhaLimpaAtivo = false;
    Serial.println("Fuga LDR encerrada; retomando perseguicao.");
    return false;
  }

  return true;
}

// Traduz codigo de reset do ESP32 para texto de diagnostico de bancada.
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

// Exibe somente dados IR novos, evitando repeticao de diagnostico a cada loop.
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

// Converte rumo da bola em comando omnidirecional; sem bola confiavel, para
// explicitamente os motores em vez de perseguir um vetor antigo.
void decidirMovimento(bool bolaDetectada, float anguloGraus, float rotacao,
                      bool imprimirDiagnostico) {
  if (!bolaDetectada) {
    informarBloqueioMovimento(
        "Head nao detecta bola (atividade abaixo do limiar de confianca).");
    motores.parar();
    return;
  }

  const bool emCurva = !bolaNoSetorFrontal(anguloGraus);
  const float anguloMovimentoGraus =
      calcularAnguloMovimentoJ(anguloGraus);
  const float anguloMovimentoRad = anguloMovimentoGraus * DEG_TO_RAD;
  const float x = sinf(anguloMovimentoRad) * VELOCIDADE_CORRECAO_LINHA;
  const float y = cosf(anguloMovimentoRad) * VELOCIDADE_CORRECAO_LINHA;

  if (imprimirDiagnostico) {
    if (USAR_BNO) {
      Serial.printf(
          "Perseguindo bola por IR: direcao=%.1f graus | modo=%s | "
          "movimento=%.1f graus | vetor=(%.3f, %.3f) | rumo=%.1f graus | "
          "alvoRumoFixo=%.1f graus | rotacao=%.3f\n",
          normalizarAngulo360(anguloGraus),
          emCurva ? "curva-J" : "captura", anguloMovimentoGraus,
          x, y, ultimoPacoteBNO.headingDegrees, rumoDesejadoGraus, rotacao);
    } else {
      Serial.printf(
          "Perseguindo bola por IR: direcao=%.1f graus | modo=%s | "
          "movimento=%.1f graus | vetor=(%.3f, %.3f) | rotacao=%.3f\n",
          normalizarAngulo360(anguloGraus),
          emCurva ? "curva-J" : "captura", anguloMovimentoGraus, x, y,
          rotacao);
    }
  }

  informarMovimentoLiberado();
  aplicarRodas(x, y, rotacao);
}

// Defesa: mantem o rumo fixo da bussola, a distancia constante ao fundo pelo
// ultrassom traseiro e so se desloca lateralmente para alinhar-se a bola. A
// fuga de linha branca detectada pelos LDRs tem prioridade sobre a bola.
void executarEstrategiaDefesa(uint32_t agora, bool quadroNovo,
                              bool bnoTelemetriaValida,
                              bool ultraTelemetriaValida,
                              bool irTelemetriaValida) {
  if (!USAR_LDR) {
    informarBloqueioMovimento("defesa requer o sensor LDR habilitado.");
    motores.parar();
    return;
  }
  const float distanciaTraseira =
      ultimoPacoteUltra.distanceCm[ULTRA_INDICE_TRASEIRO];
  const bool traseiraValida = ultraTelemetriaValida &&
      (ultimoPacoteUltra.validMask & (1U << ULTRA_INDICE_TRASEIRO)) &&
      isfinite(distanciaTraseira) &&
      distanciaTraseira >= 2.0f && distanciaTraseira <= 430.0f;
  if (USAR_BNO &&
      (!bnoTelemetriaValida || !referenciaRumoDefinida)) {
    informarBloqueioMovimento("aguardando telemetria BNO valida e recente.");
    pidRumo.reset();
    ultimoCalculoPIDUs = micros();
    motores.parar();
    return;
  }

  const bool linhaEsquerda = ldrFixed.regiaoBranca(LDRFixed::Esquerda);
  const bool linhaDireita = ldrFixed.regiaoBranca(LDRFixed::Direita);
  ultimaCorrecaoRumo = USAR_BNO
      ? calcularCorrecaoRumo(true, ultimoPacoteBNO.headingDegrees,
                             ultimoPacoteBNO.gyroZDegreesPerSecond,
                             RUMO_DEFESA_GRAUS)
      : 0.0f;

  // Mesma fuga do ataque: esquerda -> direita, direita -> esquerda, inferior ->
  // frente, centro -> tras, com memoria e tempo minimo; o rumo continua fixo.
  float xFuga = 0.0f;
  float yFuga = 0.0f;
  bool centroDetectado = false;
  bool periferiaDetectada = false;
  float periferiaX = 0.0f;
  float periferiaY = 0.0f;
  const bool linhaDetectada = calcularVetorFugaLDR(
      xFuga, yFuga, centroDetectado, periferiaDetectada, periferiaX,
      periferiaY);
  if (linhaDetectada) {
    aplicarMemoriaCorrecaoLDR(centroDetectado, periferiaDetectada, periferiaX,
                              periferiaY, xFuga, yFuga, agora);
  }
  if (atualizarFugaLDR(linhaDetectada, xFuga, yFuga, agora)) {
    if (quadroNovo) {
      Serial.printf("Defesa: fuga LDR esq=%s dir=%s | vetor=(%.3f, %.3f) | "
                    "rotacao=%.3f\n",
                    linhaEsquerda ? "branco" : "verde",
                    linhaDireita ? "branco" : "verde",
                    ultimoVetorFugaX, ultimoVetorFugaY, ultimaCorrecaoRumo);
    }
    informarMovimentoLiberado();
    aplicarRodas(ultimoVetorFugaX, ultimoVetorFugaY, ultimaCorrecaoRumo);
    return;
  }

  if (!traseiraValida) {
    informarBloqueioMovimento("aguardando ultrassom traseiro valido e recente.");
    motores.parar();
    return;
  }
  informarMovimentoLiberado();

  const float erroDistanciaCm = DISTANCIA_DEFESA_CM - distanciaTraseira;
  float y = 0.0f;
  if (fabsf(erroDistanciaCm) > ZONA_MORTA_DISTANCIA_DEFESA_CM) {
    y = constrain(erroDistanciaCm * GANHO_DISTANCIA_DEFESA,
                  -VELOCIDADE_CORRECAO_LINHA, VELOCIDADE_CORRECAO_LINHA);
  }

  const bool bolaValida = irTelemetriaValida && ultimaBolaDetectada;
  float x = 0.0f;
  if (bolaValida) {
    const float lateral = sinf(ultimaDirecaoBola * DEG_TO_RAD);
    if (fabsf(lateral) > ZONA_MORTA_BOLA_DEFESA) {
      x = constrain(lateral * GANHO_LATERAL_BOLA_DEFESA, -1.0f, 1.0f) *
          VELOCIDADE_CORRECAO_LINHA;
    }
  }

  if (quadroNovo) {
    Serial.printf("Defesa: traseira=%.1f cm | bola=%s%.1f graus | "
                  "rumo=%.1f graus | vetor=(%.3f, %.3f) | rotacao=%.3f\n",
                  distanciaTraseira, bolaValida ? "" : "ausente ",
                  ultimaDirecaoBola, ultimoPacoteBNO.headingDegrees, x, y,
                  ultimaCorrecaoRumo);
  }
  aplicarRodas(x, y, ultimaCorrecaoRumo);
}

} // namespace

// Inicializa UART, controle, sensores e IHM; motores iniciam parados para que
// a ausencia de telemetria ou uma inicializacao incompleta nao gere movimento.
void setup() {
  rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);
  Serial.begin(USB_SERIAL_BAUD);
  const esp_reset_reason_t motivoReset = esp_reset_reason();
  Serial.printf("Motivo do reset: %d (%s)\n", static_cast<int>(motivoReset),
                nomeMotivoReset(motivoReset));
  const size_t tamanhoBufferUARTConfigurado =
      Serial1.setRxBufferSize(TAMANHO_BUFFER_RX_UART);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  pidRumo.setIntegralLimit(LIMITE_INTEGRAL_RUMO);
  ultimoCalculoPIDUs = micros();
  motores.begin();
  motores.parar();
  if (USAR_LDR) {
    ldrFixed.begin();
    IHM_conectarLDR(&ldrFixed);
  }
  Serial.println("Iniciando IHM.");
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

// Consome telemetria UART, atualiza a IHM e despacha exatamente a estrategia
// selecionada. Pacotes antigos sao ignorados e sensores expirados bloqueiam
// movimento; fora de uma estrategia ativa o estado e reiniciado e motores param.
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

  if (IHM_testeTSSPAtivo()) {
    motores.parar();
    return;
  }

  if (IHM_testeBNOMoveAtivo()) {
    executarTesteBNOMove(bnoTelemetriaValida);
    return;
  }

  controleBnoMoveIniciado = false;
  rotacaoBnoMove = 0.0f;
  pidRumo.reset();

  if (IHM_estrategiaAtaqueAtiva()) {
    float xFugaLDR = 0.0f;
    float yFugaLDR = 0.0f;
    bool centroDetectado = false;
    bool periferiaDetectada = false;
    float periferiaX = 0.0f;
    float periferiaY = 0.0f;
    const bool linhaDetectada =
        USAR_LDR && calcularVetorFugaLDR(
                        xFugaLDR, yFugaLDR, centroDetectado,
                        periferiaDetectada, periferiaX, periferiaY);
    if (linhaDetectada) {
      aplicarMemoriaCorrecaoLDR(
          centroDetectado, periferiaDetectada, periferiaX, periferiaY,
          xFugaLDR, yFugaLDR, agora);
    }
    if (atualizarFugaLDR(linhaDetectada, xFugaLDR, yFugaLDR, agora)) {
      const bool irRecente = USAR_IR && pacoteInicialRecebido &&
          (uint32_t)(agora - ultimoRecebimentoMs) <= TIMEOUT_PACOTE_MS;
      const bool bnoPronto = !USAR_BNO ||
          (bnoTelemetriaValida && referenciaRumoDefinida);
      float anguloContornoGraus = ultimaDirecaoBola;
      if (linhaDetectada && !centroDetectado && irRecente && bnoPronto &&
          ultimaBolaDetectada &&
          calcularContornoLDR(ultimaDirecaoBola, ultimoPacoteUltra,
                              ultraTelemetriaValida, anguloContornoGraus)) {
        if (quadroNovo) {
          Serial.printf(
              "Contorno LDR via ultrassom: direcao IR %.1f -> %.1f graus\n",
              normalizarAngulo360(ultimaDirecaoBola),
              normalizarAngulo360(anguloContornoGraus));
          ultimaCorrecaoRumo = USAR_BNO
              ? calcularCorrecaoRumo(
                    ultimaBolaDetectada, ultimoPacoteBNO.headingDegrees,
                    ultimoPacoteBNO.gyroZDegreesPerSecond)
              : 0.0f;
        }
        informarMovimentoLiberado();
        decidirMovimento(ultimaBolaDetectada, anguloContornoGraus,
                         ultimaCorrecaoRumo, quadroNovo);
        return;
      }

      if (quadroNovo) {
        Serial.printf("Fuga da linha por LDR: vetor=(%.3f, %.3f)\n",
                      ultimoVetorFugaX, ultimoVetorFugaY);
      }
      informarMovimentoLiberado();
      aplicarRodas(ultimoVetorFugaX, ultimoVetorFugaY, 0.0f);
      return;
    }

    if (!USAR_IR) {
      informarBloqueioMovimento(
          "movimento de perseguicao requer o sensor IR habilitado.");
      motores.parar();
      return;
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
    if (USAR_BNO &&
        (!bnoTelemetriaValida || !referenciaRumoDefinida)) {
      informarBloqueioMovimento("aguardando telemetria BNO valida e recente.");
      motores.parar();
      return;
    }

    if (quadroNovo) {
      ultimaCorrecaoRumo = USAR_BNO
          ? calcularCorrecaoRumo(
                ultimaBolaDetectada, ultimoPacoteBNO.headingDegrees,
                ultimoPacoteBNO.gyroZDegreesPerSecond)
          : 0.0f;
    }
    decidirMovimento(ultimaBolaDetectada, ultimaDirecaoBola,
                     ultimaCorrecaoRumo, quadroNovo);
    return;
  }

  if (IHM_estrategiaDefesaAtiva()) {
    executarEstrategiaDefesa(agora, quadroNovo, bnoTelemetriaValida,
                            ultraTelemetriaValida, irTelemetriaValida);
    return;
  }

  fugaLDRAtiva = false;
  temporizadorLinhaLimpaAtivo = false;
  correcaoPerifericaLDRValida = false;
  pidRumo.reset();
  ultimoCalculoPIDUs = micros();
  motores.parar();
}
