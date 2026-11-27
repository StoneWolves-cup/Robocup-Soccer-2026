#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <IRProtocol.h>
#include <Omni.h>
#include <Motores.h>
#include <PID.h>
#include <FuzzyPID.h>
#include <Angulo.h>
#include <MovimentoBola.h>
#include <MovimentoDefesa.h>
#include <ProximidadeIR.h>
#include <LDRFixed.h>
#include <FugaLinha.h>
#include <ConfigPinos.h>
#include <Kicker.h>
#include <ESPNowLink.h>
#include <freertos/FreeRTOS.h>
#include <esp_system.h>

void IHM_begin();
void IHM_update();
void IHM_conectarLDR(LDRFixed *ldr);
void IHM_atualizarTelemetria(float headingDegrees, float gyroZDegreesPerSecond,
                             bool bnoValido, const float distanciasUltra[4],
                             uint8_t mascaraUltra, bool ultraValido,
                             const uint8_t atividadesIR[IR_SENSOR_COUNT],
                             float direcaoIR,
                             bool bolaDetectada, bool irValido,
                             const ProximidadeIR::Estimativa &proximidade);
void IHM_atualizarLDR(const bool regioesBrancas[LDRFixed::QuantidadeRegioes]);
bool IHM_testeTSSPAtivo();
bool IHM_testeBNOMoveAtivo();
bool IHM_testeBNOMoveExecutando();
void IHM_pararTesteBNOMove();
void IHM_obterGanhosPID(float &kp, float &ki, float &kd);
bool IHM_estrategiaAtaqueAtiva();
bool IHM_estrategiaDefesaAtiva();
bool IHM_obterBateria(float &percentual);
bool IHM_testeKickerAutomaticoAtivo();
bool IHM_testeKickerFimDeCursoAtivo();
void IHM_atualizarTesteKicker(bool comunicacaoValida);

// O Body valida a telemetria recebida antes de mover. Sensores desligaveis por
// configuracao continuam representados por flags, para que bloqueios explicitos
// sejam preferidos a assumir que um pacote ausente contem uma leitura segura.
namespace { 
  constexpr uint32_t USB_SERIAL_BAUD = 921600;
  constexpr uint32_t TIMEOUT_PACOTE_MS = 250;
  constexpr uint32_t TIMEOUT_ULTRA_MS = 350;
  constexpr uint32_t INTERVALO_DIAGNOSTICO_UART_MS = 1000;
  constexpr size_t TAMANHO_BUFFER_RX_UART = 4096;
  constexpr uint8_t AMOSTRAS_MEDIA_ULTRA_DEFESA = 3;
  // IR e BNO sao requisitos da estrategia e bloqueiam o movimento quando ausentes.
  constexpr bool USAR_LDR = true;
  constexpr bool USAR_ESPNOW = false;
  constexpr bool USAR_BNO = true;
  constexpr bool USAR_IR = true;
  constexpr bool USAR_ULTRASSONICOS = true;
  constexpr bool HABILITAR_LED_RGB = true;
  constexpr bool HABILITAR_AVANCO_BOLA_DEFESA = true;
  // GPIOs reservados para a UART, fora dos pinos atuais de motores e da IHM.
  constexpr int UART_RX_PIN = ConfigPinos::Geral::UART1::RX;
  constexpr int UART_TX_PIN = ConfigPinos::Geral::UART1::TX;
  constexpr uint32_t TIMEOUT_BNO_MS = 250;
  constexpr float POTENCIA_MAXIMA_MOTORES = 1.0;
  constexpr float FATOR_AJUSTE_KP_RUMO = 1.05f;
  constexpr float VELOCIDADE_CORRECAO_LINHA = 0.80f;
  constexpr float ROTACAO_MAXIMA_BNO_MOVE = 0.80f;
  constexpr float ZONA_MORTA_BNO_MOVE_GRAUS = 0.9f;
  constexpr float PASSO_ROTACAO_BNO_MOVE = 0.005f;
  constexpr uint32_t INTERVALO_PID_BNO_MOVE_US = 3000;
  constexpr float ZONA_MORTA_RUMO_GRAUS = 1.4;
  constexpr float ROTACAO_MAXIMA_CORRECAO = 0.80f;
  constexpr float LIMITE_INTEGRAL_RUMO = 300.0f;
  constexpr float ESCALA_ERRO_FUZZY_GRAUS = 18.5414f;
  constexpr float ESCALA_GYRO_FUZZY_GRAUS_POR_SEGUNDO = 74.1656f;
  constexpr float VELOCIDADE_RECUO_LDR_INFERIOR = 0.80f;
  constexpr float KP_LATERAL_IR = 0.025f;
  constexpr float KI_LATERAL_IR = 0.001f;
  constexpr float KD_LATERAL_IR = 0.002f;
  constexpr float LIMITE_INTEGRAL_LATERAL_IR = 80.0f;
  constexpr float LIMITE_DERIVADA_IR = 180.0f;
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
  UltraDataPacket ultimoPacoteUltra{};
  float leiturasUltraDefesa[ULTRA_SENSOR_COUNT]
                           [AMOSTRAS_MEDIA_ULTRA_DEFESA] = {};
  float somasUltraDefesa[ULTRA_SENSOR_COUNT] = {};
  uint8_t indicesUltraDefesa[ULTRA_SENSOR_COUNT] = {};
  uint8_t quantidadesUltraDefesa[ULTRA_SENSOR_COUNT] = {};
  uint32_t ultimoRecebimentoMs = 0;
  bool pacoteInicialRecebido = false;
  uint32_t ultimoRecebimentoBNOMs = 0;
  bool pacoteBNOInicialRecebido = false;
  uint32_t ultimoRecebimentoUltraMs = 0;
  bool pacoteUltraInicialRecebido = false;
  bool uartReconectando = true;
  bool referenciaRumoDefinida = false;
  float rumoDesejadoGraus = 0.0f;
  bool defesaRumoDefinida = false;
  float rumoDefesaGraus = 0.0f;
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
  ProximidadeIR::Filtro filtroProximidadeIR;
  MovimentoBola::ControladorJ controladorJ;
  MovimentoDefesa::AvancoBolaTemporizado avancoBolaDefesa;
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
  LDRFixed ldrFixed;
  FugaLinha fugaLinha(ldrFixed, VELOCIDADE_CORRECAO_LINHA);
  FuzzyPID pidRumo(PIDConfig::KP_PADRAO, PIDConfig::KI_PADRAO,
                   PIDConfig::KD_PADRAO, -ROTACAO_MAXIMA_CORRECAO,
                   ROTACAO_MAXIMA_CORRECAO, ESCALA_ERRO_FUZZY_GRAUS,
                   ESCALA_GYRO_FUZZY_GRAUS_POR_SEGUNDO);
  PID pidBnoMove(PIDConfig::KP_PADRAO, PIDConfig::KI_PADRAO,
                 PIDConfig::KD_PADRAO, -ROTACAO_MAXIMA_BNO_MOVE,
                 ROTACAO_MAXIMA_BNO_MOVE);
  PID pidLateralIR(KP_LATERAL_IR, KI_LATERAL_IR, KD_LATERAL_IR,
                   -MovimentoDefesa::VELOCIDADE_LATERAL_MAXIMA,
                   MovimentoDefesa::VELOCIDADE_LATERAL_MAXIMA);
  bool controleLateralIRIniciado = false;
  float anguloLateralIRAnterior = 0.0f;
  float comandoLateralIR = 0.0f;
  uint32_t ultimoCalculoLateralIRMs = 0;
  float rumoReferenciaBnoMove = 0.0f;
  float rotacaoBnoMove = 0.0f;
  uint32_t ultimoCalculoBnoMoveUs = 0;
  bool controleBnoMoveIniciado = false;

  void resetarControleLateralIR() {
    pidLateralIR.reset();
    controleLateralIRIniciado = false;
    comandoLateralIR = 0.0f;
  }

  float calcularComandoLateralIR(bool quadroNovo, float anguloGraus,
                                 uint32_t agoraMs) {
    const float erro = erroAngular(anguloGraus, 0.0f);
    if (fabsf(erro) <= MovimentoDefesa::ANGULO_ZONA_MORTA_GRAUS) {
      resetarControleLateralIR();
      return 0.0f;
    }
    if (controleLateralIRIniciado && !quadroNovo) return comandoLateralIR;

    float dt = 0.02f;
    float derivadaErro = 0.0f;
    if (controleLateralIRIniciado) {
      const uint32_t decorridoMs = agoraMs - ultimoCalculoLateralIRMs;
      if (decorridoMs == 0) return comandoLateralIR;
      dt = decorridoMs / 1000.0f;
      derivadaErro = constrain(
          erroAngular(erro, anguloLateralIRAnterior) / dt,
          -LIMITE_DERIVADA_IR, LIMITE_DERIVADA_IR);
    }
    controleLateralIRIniciado = true;
    anguloLateralIRAnterior = erro;
    ultimoCalculoLateralIRMs = agoraMs;
    comandoLateralIR = pidLateralIR.calculate(erro, -derivadaErro, dt);
    return comandoLateralIR;
  }

  const char *descricaoOrigemFuga(FugaLinha::OrigemCorrecao origem) {
    switch (origem) {
      case FugaLinha::OrigemCorrecao::Direta:
        return "direta";
      case FugaLinha::OrigemCorrecao::MemoriaPeriferica:
        return "memoria-periferica";
      case FugaLinha::OrigemCorrecao::FallbackCentro:
        return "fallback-centro";
      case FugaLinha::OrigemCorrecao::ParadaFrontal:
        return "parada-frontal";
      case FugaLinha::OrigemCorrecao::RetornoFrontal:
        return "retorno-frontal";
      case FugaLinha::OrigemCorrecao::Nenhuma:
      default:
        return "nenhuma";
    }
  }

  // Acende verde somente depois que os fluxos obrigatorios foram restabelecidos.
  void atualizarLedUART() {
    const bool comunicacaoConfirmada = !uartReconectando;
    if (comunicacaoConfirmada == ledUARTVerde) return;

    if (HABILITAR_LED_RGB) {
      rgbLedWrite(RGB_BUILTIN,
                  comunicacaoConfirmada ? 0 : RGB_BRIGHTNESS,
                  comunicacaoConfirmada ? RGB_BRIGHTNESS : 0,
                  0);
    }
    ledUARTVerde = comunicacaoConfirmada;
  }

  // Descarta quadros pendentes e reinicia as referencias de sequencia para
  // aceitar os contadores que voltam a zero quando o Head reinicia.
  void iniciarReconexaoUART() {
    uartReconectando = true;
    sequenciaIRIniciada = false;
    sequenciaBNOIniciada = false;
    sequenciaUltraIniciada = false;
    pacoteInicialRecebido = false;
    pacoteBNOInicialRecebido = false;
    pacoteUltraInicialRecebido = false;
    ultimoRecebimentoMs = 0;
    ultimoRecebimentoBNOMs = 0;
    ultimoRecebimentoUltraMs = 0;
    referenciaRumoDefinida = false;
    filtroProximidadeIR.resetar();
    controladorJ.resetar();
    fugaLinha.resetar();
    controleBnoMoveIniciado = false;
    pidRumo.reset();
    pidBnoMove.reset();
    resetarControleLateralIR();
    ultimaCorrecaoRumo = 0.0f;
    ultimoCalculoPIDUs = micros();
    bytesNoBufferUART = 0;

    const int bytesPendentes = Serial1.available();
    for (int i = 0; i < bytesPendentes; ++i) {
      if (Serial1.read() < 0) break;
    }

    portENTER_CRITICAL(&travaQuadro);
    quadroRecebido.disponivel = false;
    quadroBNORecebido.disponivel = false;
    quadroUltraRecebido.disponivel = false;
    portEXIT_CRITICAL(&travaQuadro);

    Serial.println(
        "UART desconectada; aguardando pacotes validos de IR e BNO.");
  }

  // IR e BNO sao necessarios para perseguir a bola e manter o rumo.
  void atualizarEstadoUART(bool irValido, bool bnoValido) {
    const bool comunicacaoValida =
        (!USAR_IR || irValido) && (!USAR_BNO || bnoValido);

    if (!comunicacaoValida) {
      if (!uartReconectando) iniciarReconexaoUART();
      return;
    }

    if (uartReconectando) {
      uartReconectando = false;
      Serial.println("UART reconectada; fluxos obrigatorios IR e BNO validos.");
    }
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
      for (uint8_t atividade : copia.activity) {
        if (atividade > 100) {
          registrarFalhaUART("atividade TSSP invalida no quadro IR");
          return false;
        }
      }
      ++pacotesIRDesdeDiagnostico;
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
      if ((copia.validMask & ~((1U << ULTRA_SENSOR_COUNT) - 1U)) != 0) {
        registrarFalhaUART("mascara de validade ultrassonica invalida");
        return false;
      }
      for (uint8_t i = 0; i < ULTRA_SENSOR_COUNT; ++i) {
        if ((copia.validMask & (1U << i)) &&
            (!isfinite(copia.distanceCm[i]) ||
             copia.distanceCm[i] < 2.0f ||
             copia.distanceCm[i] > 430.0f)) {
          registrarFalhaUART("distancia ultrassonica invalida");
          return false;
        }
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
        else if (magic == ULTRA_PACKET_MAGIC) {
          tamanhoQuadro = sizeof(UltraDataPacket);
        }
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
        "UART1/s: %lu bytes, IR=%lu, BNO=%lu, US=%lu, invalidos=%lu, "
        "pendentes=%u",
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

  bool obterQuadroUltra(UltraDataPacket &destino, uint32_t &recebidoEmMs) {
    bool disponivel;
    portENTER_CRITICAL(&travaQuadro);
    disponivel = quadroUltraRecebido.disponivel;
    if (disponivel) {
      memcpy(&destino, (const void *)&quadroUltraRecebido.pacote,
             sizeof(destino));
      recebidoEmMs = quadroUltraRecebido.recebidoEmMs;
      quadroUltraRecebido.disponivel = false;
    }
    portEXIT_CRITICAL(&travaQuadro);
    return disponivel;
  }

  void resetarMediaUltrassomDefesa() {
    memset(leiturasUltraDefesa, 0, sizeof(leiturasUltraDefesa));
    memset(somasUltraDefesa, 0, sizeof(somasUltraDefesa));
    memset(indicesUltraDefesa, 0, sizeof(indicesUltraDefesa));
    memset(quantidadesUltraDefesa, 0, sizeof(quantidadesUltraDefesa));
  }

  void atualizarMediaUltrassomDefesa(const UltraDataPacket &pacote) {
    for (uint8_t i = 0; i < ULTRA_SENSOR_COUNT; ++i) {
      if ((pacote.validMask & (1U << i)) == 0) {
        somasUltraDefesa[i] = 0.0f;
        indicesUltraDefesa[i] = 0;
        quantidadesUltraDefesa[i] = 0;
        continue;
      }

      const uint8_t indice = indicesUltraDefesa[i];
      if (quantidadesUltraDefesa[i] == AMOSTRAS_MEDIA_ULTRA_DEFESA) {
        somasUltraDefesa[i] -= leiturasUltraDefesa[i][indice];
      } else {
        ++quantidadesUltraDefesa[i];
      }
      leiturasUltraDefesa[i][indice] = pacote.distanceCm[i];
      somasUltraDefesa[i] += pacote.distanceCm[i];
      indicesUltraDefesa[i] =
          (indice + 1) % AMOSTRAS_MEDIA_ULTRA_DEFESA;
    }
  }

  bool obterMediaUltrassomDefesa(uint8_t indice, float &distanciaCm) {
    if (quantidadesUltraDefesa[indice] == 0) return false;
    distanciaCm = somasUltraDefesa[indice] / quantidadesUltraDefesa[indice];
    return isfinite(distanciaCm) && distanciaCm >= 2.0f &&
        distanciaCm <= 430.0f;
  }

  // Converte qualquer angulo para [0, 360), util para comparacoes de setores.
  float normalizarAngulo360(float anguloGraus) {
    anguloGraus = fmodf(anguloGraus, 360.0f);
    return anguloGraus < 0.0f ? anguloGraus + 360.0f : anguloGraus;
  }

  // Corrige rumo fixado na primeira leitura valida. O gyro amortece a rotacao e
  // a zona morta reduz acionamentos por pequenas variacoes do magnetometro.
  float calcularCorrecaoRumo(bool controleAtivo, bool quadroBnoNovo,
                            float heading, float gyroZ,
                            float rumoAlvoGraus = rumoDesejadoGraus) {
    const uint32_t agoraUs = micros();

    if (!controleAtivo) {
      pidRumo.reset();
      ultimoCalculoPIDUs = agoraUs;
      return 0.0f;
    }

    if (!quadroBnoNovo) return ultimaCorrecaoRumo;
    const uint32_t decorridoUs = agoraUs - ultimoCalculoPIDUs;
    ultimoCalculoPIDUs = agoraUs;

    float kp = PIDConfig::KP_PADRAO;
    float ki = PIDConfig::KI_PADRAO;
    float kd = PIDConfig::KD_PADRAO;
    IHM_obterGanhosPID(kp, ki, kd);
    pidRumo.setGains(kp * FATOR_AJUSTE_KP_RUMO, ki, kd);

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

  void executarEstrategiaDefesa(bool quadroNovo, bool quadroBnoNovo,
                                 bool quadroUltraNovo,
                                 bool bnoTelemetriaValida,
                                 bool irTelemetriaValida,
                                 bool ultraTelemetriaValida) {
    if (!USAR_ULTRASSONICOS) {
      resetarControleLateralIR();
      informarBloqueioMovimento(
          "estrategia de defesa requer ultrassonicos habilitados.");
      avancoBolaDefesa.interromper(millis());
      motores.parar();
      return;
    }

    if (!ultraTelemetriaValida) {
      resetarMediaUltrassomDefesa();
    } else if (quadroUltraNovo) {
      atualizarMediaUltrassomDefesa(ultimoPacoteUltra);
    }

    if (USAR_BNO &&
        (!bnoTelemetriaValida || !defesaRumoDefinida)) {
      resetarControleLateralIR();
      pidRumo.reset();
      ultimoCalculoPIDUs = micros();
      informarBloqueioMovimento(
          "aguardando rumo de defesa e telemetria BNO validos.");
      avancoBolaDefesa.interromper(millis());
      motores.parar();
      return;
    }

    if (USAR_BNO && bnoTelemetriaValida) {
      ultimaCorrecaoRumo = calcularCorrecaoRumo(
          true, quadroBnoNovo, ultimoPacoteBNO.headingDegrees,
          ultimoPacoteBNO.gyroZDegreesPerSecond, rumoDefesaGraus);
    } else if (!USAR_BNO) {
      ultimaCorrecaoRumo = 0.0f;
    }

    float distanciaTraseiraCm = 0.0f;
    const bool ultrassomTraseiroValido =
        ultraTelemetriaValida &&
        (ultimoPacoteUltra.validMask & (1U << ULTRA_INDEX_REAR)) != 0 &&
        isfinite(ultimoPacoteUltra.distanceCm[ULTRA_INDEX_REAR]) &&
        ultimoPacoteUltra.distanceCm[ULTRA_INDEX_REAR] >= 2.0f &&
        ultimoPacoteUltra.distanceCm[ULTRA_INDEX_REAR] <= 430.0f &&
        obterMediaUltrassomDefesa(ULTRA_INDEX_REAR, distanciaTraseiraCm);
    if (!ultrassomTraseiroValido) {
      resetarControleLateralIR();
      informarBloqueioMovimento(
          "aguardando ultrassom traseiro valido e recente.");
      avancoBolaDefesa.interromper(millis());
      motores.parar();
      return;
    }

    float distanciaDireitaCm = 0.0f;
    const bool ultrassomDireitoValido =
        ultraTelemetriaValida &&
        (ultimoPacoteUltra.validMask & (1U << ULTRA_INDEX_RIGHT)) != 0 &&
        isfinite(ultimoPacoteUltra.distanceCm[ULTRA_INDEX_RIGHT]) &&
        ultimoPacoteUltra.distanceCm[ULTRA_INDEX_RIGHT] >= 2.0f &&
        ultimoPacoteUltra.distanceCm[ULTRA_INDEX_RIGHT] <= 430.0f &&
        obterMediaUltrassomDefesa(ULTRA_INDEX_RIGHT, distanciaDireitaCm);
    float distanciaEsquerdaCm = 0.0f;
    const bool ultrassomEsquerdoValido =
        ultraTelemetriaValida &&
        (ultimoPacoteUltra.validMask & (1U << ULTRA_INDEX_LEFT)) != 0 &&
        isfinite(ultimoPacoteUltra.distanceCm[ULTRA_INDEX_LEFT]) &&
        ultimoPacoteUltra.distanceCm[ULTRA_INDEX_LEFT] >= 2.0f &&
        ultimoPacoteUltra.distanceCm[ULTRA_INDEX_LEFT] <= 430.0f &&
        obterMediaUltrassomDefesa(ULTRA_INDEX_LEFT, distanciaEsquerdaCm);
    const bool faixaCentralU1 = USAR_LDR &&
        MovimentoDefesa::faixaCentralU1(
            distanciaDireitaCm, ultrassomDireitoValido);
    const bool inferior = faixaCentralU1 &&
        ldrFixed.regiaoBranca(LDRFixed::Inferior);
    const bool bolaValida =
        USAR_IR && irTelemetriaValida && ultimaBolaDetectada;
    const bool bolaNaFaixaAvanco = bolaValida &&
        filtroProximidadeIR.estimativa().valida &&
        filtroProximidadeIR.estimativa().indice >=
            ProximidadeIR::INDICE_MINIMO_MEDIA_DISTANTE;
    const float proximidadeBola = filtroProximidadeIR.estimativa().valida
        ? filtroProximidadeIR.estimativa().indice
        : 0.0f;
    const bool avancoBolaAtivo = HABILITAR_AVANCO_BOLA_DEFESA &&
        avancoBolaDefesa.atualizar(millis(), bolaValida, bolaNaFaixaAvanco,
                                   proximidadeBola, ultimaDirecaoBola);
    float x;
    if (bolaValida && !avancoBolaAtivo) {
      x = calcularComandoLateralIR(quadroNovo, ultimaDirecaoBola, millis());
    } else {
      resetarControleLateralIR();
      x = MovimentoDefesa::comandoCentroU1(
          distanciaDireitaCm, ultrassomDireitoValido);
    }
    float y = -MovimentoDefesa::comandoDistancia(distanciaTraseiraCm);

    if (avancoBolaAtivo) {
      MovimentoDefesa::vetorAvancoBola(ultimaDirecaoBola, x, y);
    }

    const float comandoLateralAntesLimite = x;
    x = MovimentoDefesa::limitarComandoLateral(
        x, distanciaDireitaCm, ultrassomDireitoValido,
        distanciaEsquerdaCm, ultrassomEsquerdoValido);
    if (x != comandoLateralAntesLimite) resetarControleLateralIR();
    const char *fase = x != comandoLateralAntesLimite
        ? "limite lateral 43 cm"
        : avancoBolaAtivo
        ? "avancando para a bola"
        : bolaValida
        ? (x != 0.0f ? "acompanhando bola" : "bola alinhada")
        : (x != 0.0f ? "retornando ao centro" : "centro alinhado");

    if (inferior) {
      y = fmaxf(y, VELOCIDADE_RECUO_LDR_INFERIOR);
      fase = "recuando do LDR inferior";
    }

    if (quadroNovo) {
      Serial.printf("Defesa: %s | U1=%.1f cm | U3=%.1f cm | "
                    "faixa-central=%c | "
                    "LDR-inferior=%c | traseira=%.1f cm | "
                    "bola=%s%.1f graus | vetor=(%.3f, %.3f) | "
                    "rumo=%.1f graus | rotacao=%.3f\n",
                    fase,
                    ultrassomDireitoValido ? distanciaDireitaCm : -1.0f,
                    ultrassomEsquerdoValido ? distanciaEsquerdaCm : -1.0f,
                    faixaCentralU1 ? 'S' : 'N',
                    faixaCentralU1 ? (inferior ? 'B' : 'V') : '-',
                    distanciaTraseiraCm,
                    bolaValida
                        ? ""
                        : "ausente ",
                    ultimaDirecaoBola, x, y, ultimoPacoteBNO.headingDegrees,
                    ultimaCorrecaoRumo);
    }

    informarMovimentoLiberado();
    aplicarRodas(x, y, ultimaCorrecaoRumo);
  }

  // Mantem o rumo capturado no inicio do teste; qualquer falha de telemetria
  // interrompe o giro e exige que o operador inicie novamente.
  void executarTesteBNOMove(bool bnoTelemetriaValida) {
    if (!IHM_testeBNOMoveExecutando()) {
      controleBnoMoveIniciado = false;
      rotacaoBnoMove = 0.0f;
      pidBnoMove.reset();
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
      pidBnoMove.reset();
      motores.parar();
      return;
    }

    float kp = PIDConfig::KP_PADRAO;
    float ki = PIDConfig::KI_PADRAO;
    float kd = PIDConfig::KD_PADRAO;
    IHM_obterGanhosPID(kp, ki, kd);
    pidBnoMove.setGains(kp, ki, kd);

    if (!controleBnoMoveIniciado) {
      rumoReferenciaBnoMove = ultimoPacoteBNO.headingDegrees;
      controleBnoMoveIniciado = true;
      ultimoCalculoBnoMoveUs = micros();
      rotacaoBnoMove = 0.0f;
      pidBnoMove.reset();
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
      const float alvo = pidBnoMove.calculate(
          erro, ultimoPacoteBNO.gyroZDegreesPerSecond, dt);
      rotacaoBnoMove += constrain(alvo - rotacaoBnoMove,
                                  -PASSO_ROTACAO_BNO_MOVE,
                                  PASSO_ROTACAO_BNO_MOVE);
    }

    const ComandosMotores rodas = omni.calcular(0.0f, 0.0f, rotacaoBnoMove);
    motores.definir(rodas.m1, rodas.m2, rodas.m3, rodas.m4);
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
      controladorJ.resetar();
      motores.parar();
      return;
    }

    const ProximidadeIR::Estimativa &proximidade =
        filtroProximidadeIR.estimativa();
    const float anguloMovimentoGraus =
        controladorJ.calcularAngulo(anguloGraus, proximidade.indice);
    const bool emCurva = fabsf(
        anguloMovimentoGraus - erroAngular(anguloGraus, 0.0f)) >= 0.5f;
    const float anguloMovimentoRad = anguloMovimentoGraus * DEG_TO_RAD;
    const float x = sinf(anguloMovimentoRad) * VELOCIDADE_CORRECAO_LINHA;
    const float y = cosf(anguloMovimentoRad) * VELOCIDADE_CORRECAO_LINHA;

    if (imprimirDiagnostico) {
      if (USAR_BNO) {
        Serial.printf(
            "Perseguindo bola por IR: direcao=%.1f graus | modo=%s | "
            "movimento=%.1f graus | vetor=(%.3f, %.3f) | rumo=%.1f graus | "
            "alvoRumoFixo=%.1f graus | rotacao=%.3f | IR max=%u%% "
            "sensores=%u conf=%u%% prox=%s (%.0f%%)\n",
            normalizarAngulo360(anguloGraus),
            emCurva ? "curva-J" : "captura", anguloMovimentoGraus,
            x, y, ultimoPacoteBNO.headingDegrees, rumoDesejadoGraus, rotacao,
            proximidade.atividadeMaxima, proximidade.sensoresAtivos,
            ultimoPacote.confidencePercent,
            ProximidadeIR::descricao(proximidade.indice),
            proximidade.indice * 100.0f);
      } else {
        Serial.printf(
            "Perseguindo bola por IR: direcao=%.1f graus | modo=%s | "
            "movimento=%.1f graus | vetor=(%.3f, %.3f) | rotacao=%.3f | "
            "IR max=%u%% sensores=%u conf=%u%% prox=%s (%.0f%%)\n",
            normalizarAngulo360(anguloGraus),
            emCurva ? "curva-J" : "captura", anguloMovimentoGraus, x, y,
            rotacao, proximidade.atividadeMaxima, proximidade.sensoresAtivos,
            ultimoPacote.confidencePercent,
            ProximidadeIR::descricao(proximidade.indice),
            proximidade.indice * 100.0f);
      }
    }

    informarMovimentoLiberado();
    aplicarRodas(x, y, rotacao);
  }

} // namespace

// Inicializa UART, controle, sensores e IHM; motores iniciam parados para que
// a ausencia de telemetria ou uma inicializacao incompleta nao gere movimento.
void setup() {
  Serial.begin(USB_SERIAL_BAUD);
  Kicker::begin();
  if (HABILITAR_LED_RGB) {
    rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);
  }
  const esp_reset_reason_t motivoReset = esp_reset_reason();
  Serial.printf("Motivo do reset: %d (%s)\n", static_cast<int>(motivoReset),
                nomeMotivoReset(motivoReset));
  const size_t tamanhoBufferUARTConfigurado =
      Serial1.setRxBufferSize(TAMANHO_BUFFER_RX_UART);
  Serial1.begin(HEAD_BODY_UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  pidRumo.setIntegralLimit(LIMITE_INTEGRAL_RUMO);
  pidBnoMove.setIntegralLimit(LIMITE_INTEGRAL_RUMO);
  pidLateralIR.setIntegralLimit(LIMITE_INTEGRAL_LATERAL_IR);
  ultimoCalculoPIDUs = micros();
  motores.begin();
  motores.parar();
  if (USAR_LDR) {
    ldrFixed.begin();
    IHM_conectarLDR(&ldrFixed);
  }
  Serial.println("Iniciando IHM.");
  IHM_begin();
  if (USAR_ESPNOW) {
    ESPNowLink::begin();
  }
  Serial.println("IHM inicializada; entrando no loop.");

  Serial.printf("Body UART1: %lu baud, 8N1, RX=18 TX=17.\n",
                static_cast<unsigned long>(HEAD_BODY_UART_BAUD));
  if (tamanhoBufferUARTConfigurado < TAMANHO_BUFFER_RX_UART) {
    Serial.printf("ERRO: buffer RX UART1 configurado com %u de %u bytes.\n",
                  static_cast<unsigned>(tamanhoBufferUARTConfigurado),
                  static_cast<unsigned>(TAMANHO_BUFFER_RX_UART));
  }
  Serial.println(
      "LED verde requer pacotes validos e recentes de IR e BNO.");
  Serial.println(
      "UART aguardando conexao; motores parados ate validar IR e BNO.");
  Serial.printf("Sensores: IR=%s, BNO=%s, LDR=%s, ultrassonicos=%s\n",
                USAR_IR ? "ligado" : "desligado",
                USAR_BNO ? "ligado" : "desligado",
                USAR_LDR ? "ligado" : "desligado",
                USAR_ULTRASSONICOS ? "ligados" : "desligados");
  Serial.printf("ESP-NOW: %s\n", USAR_ESPNOW ? "ligado" : "desligado");
  Serial.println(USAR_LDR
      ? "LDR habilitado: classificacao por limiares fixos e histerese."
      : "LDR desativado.");
}

// Consome telemetria UART, atualiza a IHM e despacha exatamente a estrategia
// selecionada. Pacotes antigos sao ignorados; a perda de telemetria inicia
// reconexao continua e mantem os motores parados ate os fluxos serem validados.
void loop() {
  processarUART();
  atualizarLedUART();
  imprimirDiagnosticoUART();
  IHM_update();
  if (USAR_ESPNOW) {
    ESPNowLink::update();
  }
  const bool estrategiaAtaqueLocal = IHM_estrategiaAtaqueAtiva();
  // Ataque cede lugar a defesa enquanto o parceiro informa RoboDefOn = false;
  // timeout do vinculo nao conta como false, portanto o ataque segue normal.
  const bool defesaSubstituiAtaque =
      USAR_ESPNOW && estrategiaAtaqueLocal &&
      ESPNowLink::defesaParceiroDesligada();
  const bool estrategiaAtaqueAtiva =
      estrategiaAtaqueLocal && !defesaSubstituiAtaque;
  const bool estrategiaDefesaAtiva =
      IHM_estrategiaDefesaAtiva() || defesaSubstituiAtaque;
  const bool estrategiaDefesaExecutando = estrategiaDefesaAtiva;
  if (!estrategiaDefesaExecutando && defesaRumoDefinida) {
    defesaRumoDefinida = false;
    ultimaCorrecaoRumo = 0.0f;
    pidRumo.reset();
    ultimoCalculoPIDUs = micros();
  }
  if (!estrategiaDefesaExecutando) {
    avancoBolaDefesa.resetar();
    resetarControleLateralIR();
    resetarMediaUltrassomDefesa();
  }

  if (USAR_LDR) {
    ldrFixed.update();
    IHM_atualizarLDR(ldrFixed.regioesBrancas());
  }

  BNODataPacket pacoteBNO{};
  uint32_t recebidoBNOEmMs = 0;
  bool quadroBnoNovo = false;
  if (obterQuadroBNO(pacoteBNO, recebidoBNOEmMs) && USAR_BNO) {
    if (!sequenciaBNOIniciada ||
        (int16_t)(pacoteBNO.sequence - ultimaSequenciaBNO) > 0) {
      ultimaSequenciaBNO = pacoteBNO.sequence;
      sequenciaBNOIniciada = true;
      ultimoPacoteBNO = pacoteBNO;
      quadroBnoNovo = true;
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
  bool quadroUltraNovo = false;
  if (obterQuadroUltra(pacoteUltra, recebidoUltraEmMs)) {
    if (!sequenciaUltraIniciada ||
        (int16_t)(pacoteUltra.sequence - ultimaSequenciaUltra) > 0) {
      ultimaSequenciaUltra = pacoteUltra.sequence;
      sequenciaUltraIniciada = true;
      ultimoPacoteUltra = pacoteUltra;
      quadroUltraNovo = true;
      pacoteUltraInicialRecebido = true;
      ultimoRecebimentoUltraMs = recebidoUltraEmMs;
    }
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
      filtroProximidadeIR.atualizar(pacote.activity, IR_SENSOR_COUNT,
                                    ultimaBolaDetectada);
      ultimaDirecaoBola = pacote.directionTenthsDegrees / 10.0f;
      ultimoRecebimentoMs = recebidoEmMs;
      pacoteInicialRecebido = true;
    }
  }

  const bool bnoTelemetriaValida = USAR_BNO && pacoteBNOInicialRecebido &&
      (uint32_t)(agora - ultimoRecebimentoBNOMs) <= TIMEOUT_BNO_MS;
  const bool ultraTelemetriaValida = USAR_ULTRASSONICOS &&
      pacoteUltraInicialRecebido &&
      (uint32_t)(agora - ultimoRecebimentoUltraMs) <= TIMEOUT_ULTRA_MS;
  const bool irTelemetriaValida = USAR_IR && pacoteInicialRecebido &&
      (uint32_t)(agora - ultimoRecebimentoMs) <= TIMEOUT_PACOTE_MS;
  if (!irTelemetriaValida) filtroProximidadeIR.resetar();
  IHM_atualizarTelemetria(
      ultimoPacoteBNO.headingDegrees, ultimoPacoteBNO.gyroZDegreesPerSecond,
      bnoTelemetriaValida, ultimoPacoteUltra.distanceCm,
      ultimoPacoteUltra.validMask, ultraTelemetriaValida, ultimoPacote.activity,
      ultimaDirecaoBola, ultimaBolaDetectada, irTelemetriaValida,
      filtroProximidadeIR.estimativa());
  atualizarEstadoUART(irTelemetriaValida, bnoTelemetriaValida);
  atualizarLedUART();
  IHM_atualizarTesteKicker(!uartReconectando);

  float bateriaPercentual = 0.0f;
  const bool bateriaValida = IHM_obterBateria(bateriaPercentual);
  const Kicker::ModoAutomatico modoKicker = IHM_testeKickerFimDeCursoAtivo()
      ? Kicker::ModoAutomatico::FimDeCurso
      : IHM_testeKickerAutomaticoAtivo()
          ? Kicker::ModoAutomatico::ComUltrassom
          : estrategiaAtaqueAtiva
              ? Kicker::ModoAutomatico::AtaqueUltrassomDireito
              : estrategiaDefesaExecutando
                  ? Kicker::ModoAutomatico::Defesa
                  : Kicker::ModoAutomatico::Desativado;
  const ProximidadeIR::Estimativa &proximidadeBola =
      filtroProximidadeIR.estimativa();
  const bool bolaPerto = irTelemetriaValida && ultimaBolaDetectada &&
      proximidadeBola.valida && ProximidadeIR::estaPerto(proximidadeBola.indice);
  Kicker::atualizar(modoKicker, !uartReconectando,
                    bateriaValida, bateriaPercentual, ultraTelemetriaValida,
                    ultimoPacoteUltra.distanceCm, ultimoPacoteUltra.validMask,
                    bolaPerto);

  if (uartReconectando) {
    informarBloqueioMovimento(
        "aguardando reconexao UART e pacotes validos de todos os sensores.");
    motores.parar();
    return;
  }

  if (quadroNovo) {
    imprimirDadosIR(ultimoPacote);
  }

  if (IHM_testeTSSPAtivo()) {
    resetarControleLateralIR();
    pidRumo.reset();
    ultimaCorrecaoRumo = 0.0f;
    ultimoCalculoPIDUs = micros();
    motores.parar();
    return;
  }

  if (IHM_testeBNOMoveAtivo()) {
    resetarControleLateralIR();
    pidRumo.reset();
    ultimaCorrecaoRumo = 0.0f;
    ultimoCalculoPIDUs = micros();
    executarTesteBNOMove(bnoTelemetriaValida);
    return;
  }

  controleBnoMoveIniciado = false;
  rotacaoBnoMove = 0.0f;
  pidBnoMove.reset();
  if (estrategiaDefesaExecutando && USAR_BNO && bnoTelemetriaValida &&
      !defesaRumoDefinida) {
    rumoDefesaGraus = ultimoPacoteBNO.headingDegrees;
    defesaRumoDefinida = true;
    ultimaCorrecaoRumo = 0.0f;
    pidRumo.reset();
    ultimoCalculoPIDUs = micros();
    Serial.printf("Defesa: rumo de referencia capturado em %.1f graus.\n",
                  rumoDefesaGraus);
  }
  if (estrategiaAtaqueAtiva) {
    const bool fugaEmAndamento = fugaLinha.atualizar(USAR_LDR, agora);
    const bool paradaFrontal =
        fugaEmAndamento &&
        fugaLinha.origemCorrecao() == FugaLinha::OrigemCorrecao::ParadaFrontal;
    if (USAR_BNO && bnoTelemetriaValida) {
      ultimaCorrecaoRumo = calcularCorrecaoRumo(
          !paradaFrontal && (ultimaBolaDetectada || fugaEmAndamento),
          quadroBnoNovo,
          ultimoPacoteBNO.headingDegrees,
          ultimoPacoteBNO.gyroZDegreesPerSecond);
    } else if (!USAR_BNO) {
      ultimaCorrecaoRumo = 0.0f;
    }

    if (fugaLinha.expirada()) {
      if (quadroNovo) {
        Serial.printf("Fuga LDR EXPIRADA: C=%c E=%c D=%c I=%c S=%c\n",
                      fugaLinha.regiaoBranca(LDRFixed::Centro) ? 'B' : 'V',
                      fugaLinha.regiaoBranca(LDRFixed::Esquerda) ? 'B' : 'V',
                      fugaLinha.regiaoBranca(LDRFixed::Direita) ? 'B' : 'V',
                      fugaLinha.regiaoBranca(LDRFixed::Inferior) ? 'B' : 'V',
                      fugaLinha.regiaoBranca(LDRFixed::Superior) ? 'B' : 'V');
      }
      informarBloqueioMovimento(
          "linha continua detectada apos 1 s de fuga; aguardando linha limpa.");
      motores.parar();
      return;
    }
    if (fugaEmAndamento) {
      if (quadroNovo) {
        Serial.printf(
            "Fuga LDR: C=%c E=%c D=%c I=%c S=%c | vetor=(%.3f, %.3f) | "
            "origem=%s\n",
            fugaLinha.regiaoBranca(LDRFixed::Centro) ? 'B' : 'V',
            fugaLinha.regiaoBranca(LDRFixed::Esquerda) ? 'B' : 'V',
            fugaLinha.regiaoBranca(LDRFixed::Direita) ? 'B' : 'V',
            fugaLinha.regiaoBranca(LDRFixed::Inferior) ? 'B' : 'V',
            fugaLinha.regiaoBranca(LDRFixed::Superior) ? 'B' : 'V',
            fugaLinha.vetorX(), fugaLinha.vetorY(),
            descricaoOrigemFuga(fugaLinha.origemCorrecao()));
      }
      informarMovimentoLiberado();
      if (paradaFrontal) {
        motores.parar();
      } else {
        aplicarRodas(fugaLinha.vetorX(), fugaLinha.vetorY(),
                     ultimaCorrecaoRumo);
      }
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

    decidirMovimento(ultimaBolaDetectada, ultimaDirecaoBola,
                    ultimaCorrecaoRumo, quadroNovo);
    return;
  }

  if (estrategiaDefesaAtiva) {
    executarEstrategiaDefesa(quadroNovo, quadroBnoNovo,
                             quadroUltraNovo,
                             bnoTelemetriaValida, irTelemetriaValida,
                             ultraTelemetriaValida);
    fugaLinha.resetar();
    return;
  }

  fugaLinha.resetar();
  pidRumo.reset();
  ultimoCalculoPIDUs = micros();
  motores.parar();
}
