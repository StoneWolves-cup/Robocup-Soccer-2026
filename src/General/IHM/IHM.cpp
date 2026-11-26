#include <Wire.h>
#include <U8g2lib.h>
#include <BatteryEstimator.h>
#include <Kicker.h>
#include <KickerLogica.h>
#include <MotoresTeste.h>
#include <ConfigPinos.h>
#include <IRProtocol.h>
#include <ProximidadeIR.h>
#include <LDRFixed.h>
#include <PID.h>
#include <Preferences.h>
#include <ESPNowLink.h>
#include <math.h>

// =====================================================
// I2C
// =====================================================

constexpr uint8_t SDA_PIN = ConfigPinos::Body::IHM::OLED::SDA;
constexpr uint8_t SCK_PIN = ConfigPinos::Body::IHM::OLED::SCL;

// =====================================================
// =====================================================
// BOTOES
// =====================================================

constexpr uint8_t BOTAO_SELECIONAR = ConfigPinos::Body::IHM::Botoes::SELECIONAR;
constexpr uint8_t BOTAO_MEXER = ConfigPinos::Body::IHM::Botoes::MOVER;
constexpr uint8_t BOTAO_VOLTAR = ConfigPinos::Body::IHM::Botoes::VOLTAR;

// =====================================================
// =====================================================
// OLED
// =====================================================

U8G2_SSD1315_128X64_NONAME_F_HW_I2C oled(
  U8G2_R0,
  U8X8_PIN_NONE,
  SCK_PIN,
  SDA_PIN
);

// =====================================================
// =====================================================
// TELAS
// =====================================================

enum Tela {

  TELA_MENU,

  TELA_ESTRATEGIA,
  TELA_TELEMETRIA,
  TELA_BATERIA,
  TELA_CHECKLIST,

  TELA_ATAQUE,
  TELA_DEFESA,

  TELA_MENU_BNO,
  TELA_BNO_LEITURA,
  TELA_BNO_RESET,

  TELA_KICKER,
  TELA_TESTE_KICKER_AUTO,
  TELA_TESTE_KICKER_FIM_CURSO,
  TELA_ULTRA,
  TELA_INFRA,
  TELA_LDR,
  TELA_LDR_CENTRO,
  TELA_LDR_AJUSTE,
  TELA_LDR_CENTRO_AJUSTE,
  TELA_MAC,

  TELA_TESTE_BNO,
  TELA_TESTE_TSSP,
  TELA_INFO_TESTE
};

Tela tela = TELA_MENU;
BatteryEstimator estimadorBateria;
Motores motoresChecklist;
Omni omniChecklist;
uint32_t ultimaAtualizacaoBateriaMs = 0;

// =====================================================
// MENU PRINCIPAL
// =====================================================

int opcaoMenu = 0;
const int totalMenu = 4;

// 0 = ESTRATEGIA
// 1 = TELEMETRIA
// 2 = BATERIA
// 3 = CHECKLIST

// =====================================================
// MENU ESTRATEGIA
// =====================================================

int opcaoEstrategia = 0;
const int totalEstrategia = 2;
uint8_t perfilLDRAtual = 0;

// 0 = ATAQUE
// 1 = DEFESA

// =====================================================
// MENU TELEMETRIA
// =====================================================

int opcaoTelemetria = 0;
int opcaoKicker = 0;
const char *mensagemKicker = nullptr;
const int totalTelemetria = 8;
int opcaoMac = 0;
const uint8_t totalRegioesLDR = 4;
const char *const NOMES_LDR[totalRegioesLDR] = {"ESQ", "DIR", "INF", "FRT"};
uint16_t limiaresLDR[totalRegioesLDR] = {
    ConfigLDR::Ataque.esquerda, ConfigLDR::Ataque.direita,
    ConfigLDR::Ataque.inferior, ConfigLDR::Ataque.superior};
constexpr uint8_t TOTAL_SENSORES_CENTRO_LDR = 7;
uint16_t limiaresCentroLDR[TOTAL_SENSORES_CENTRO_LDR] = {
    ConfigLDR::Ataque.centro[0], ConfigLDR::Ataque.centro[1],
    ConfigLDR::Ataque.centro[2], ConfigLDR::Ataque.centro[3],
    ConfigLDR::Ataque.centro[4], ConfigLDR::Ataque.centro[5],
    ConfigLDR::Ataque.centro[6]};
int opcaoLDR = 0;
bool ajustandoLDR = false;
int opcaoCentroLDR = 0;
bool ajustandoCentroLDR = false;
LDRFixed *ldrThresholdTarget = nullptr;
Preferences preferenciasLDR;
bool preferenciasLDRDisponiveis = false;
bool erroLimiaresLDR = false;
bool erroPerfilLDR = false;

struct ConfiguracaoLimiaresLDR {
  uint16_t versao;
  uint16_t esquerda;
  uint16_t direita;
  uint16_t inferior;
  uint16_t superior;
};

// A versao 1 nao tinha o limiar superior; ainda e lida para nao perder a
// calibracao ja salva de ESQ/DIR/INF.
struct ConfiguracaoLimiaresLDRV1 {
  uint16_t versao;
  uint16_t esquerda;
  uint16_t direita;
  uint16_t inferior;
};

constexpr uint16_t VERSAO_CONFIG_LDR = 2;
constexpr uint16_t VERSAO_CONFIG_LDR_V1 = 1;
constexpr uint16_t LIMIAR_LDR_MAXIMO = 4095;

// 0 = BNO
// 1 = KICKER
// 2 = ULTRASSONICOS
// 3 = INFRA
// 4 = LDR
// 5 = MAC ADDRESS
// =====================================================
// MENU BNO
// =====================================================

int opcaoBNO = 0;
const int totalBNO = 3;

// 0 = LEITURA
// 1 = ZERO
// 2 = RESET SENSOR

constexpr float PASSO_KP_BNO_MOVE = 0.00004f;
constexpr float PASSO_KI_BNO_MOVE = 0.00000001f;
constexpr float PASSO_KD_BNO_MOVE = 0.00003f;
constexpr float MAX_KP_BNO_MOVE = 0.150f;
constexpr float MAX_KI_BNO_MOVE = 0.0001f;
constexpr float MAX_KD_BNO_MOVE = 0.004f;
constexpr uint16_t VERSAO_GANHOS_BNO_MOVE = 1;
constexpr uint32_t TEMPO_SEGURAR_VOLTAR_BNO_MS = 900;

struct GanhosBnoMove {
  uint16_t versao;
  float kp;
  float ki;
  float kd;
};

float kpBnoMove = PIDConfig::KP_PADRAO;
float kiBnoMove = PIDConfig::KI_PADRAO;
float kdBnoMove = PIDConfig::KD_PADRAO;
uint8_t opcaoBnoMove = 0;
bool bnoMoveExecutando = false;
bool bnoMoveErroNvs = false;
bool voltarBnoMovePressionado = false;
bool voltarBnoMoveLongo = false;
uint32_t inicioVoltarBnoMoveMs = 0;

// =====================================================
// MENU CHECKLIST
// =====================================================

int opcaoChecklist = 0;
const int totalChecklist = 6;
const int itensChecklistVisiveis = 4;

// BNO, IR, motores, solenoides, UART e LDR.
const char *const NOMES_CHECKLIST[totalChecklist] = {
    "BNO", "IR / TSSP", "MOTORES",
    "SOLENOIDES", "UART", "LDR"};

// =====================================================
// ESTADOS DOS BOTOES
// =====================================================

bool estadoAnteriorSelecionar = LOW;
bool estadoAnteriorMexer = LOW;
bool estadoAnteriorVoltar = LOW;

// =====================================================
// DEBOUNCE
// =====================================================

unsigned long ultimoSelecionar = 0;
unsigned long ultimoMexer = 0;
unsigned long ultimoVoltar = 0;

const unsigned long DEBOUNCE = 150;
constexpr unsigned long ATRASO_REPETICAO_LDR_MS = 400;
constexpr unsigned long INTERVALO_REPETICAO_LDR_MS = 150;

enum class RepeticaoAjusteLDR : uint8_t {
  Nenhuma,
  Aumentar,
  Diminuir
};

RepeticaoAjusteLDR repeticaoAjusteLDR = RepeticaoAjusteLDR::Nenhuma;
unsigned long inicioRepeticaoAjusteLDR = 0;
unsigned long ultimaRepeticaoAjusteLDR = 0;
bool alteracaoRepetidaLDRPendente = false;

// =====================================================
// DEBOUNCE REAL DOS BOTOES
// =====================================================
// Filtra o ruido mecanico do botao e so altera o estado
// depois que a leitura ficar estavel por alguns ms.
// Assim, um toque gera apenas um clique.

bool leituraBrutaSelecionar = LOW;
bool leituraBrutaMexer = LOW;
bool leituraBrutaVoltar = LOW;

bool estadoEstavelSelecionar = LOW;
bool estadoEstavelMexer = LOW;
bool estadoEstavelVoltar = LOW;

unsigned long ultimaMudancaBrutaSelecionar = 0;
unsigned long ultimaMudancaBrutaMexer = 0;
unsigned long ultimaMudancaBrutaVoltar = 0;

const unsigned long DEBOUNCE_ESTAVEL = 35;

bool lerBotaoEstavel(
  uint8_t pino,
  bool &leituraBruta,
  bool &estadoEstavel,
  unsigned long &ultimaMudanca
) {

  // Memoriza a ultima borda e so muda o estado logico depois que o sinal
  // permanece estavel; isso evita multiplos comandos causados pelo bounce.
  bool leituraAtual = digitalRead(pino);
  unsigned long agora = millis();

  if (leituraAtual != leituraBruta) {
    leituraBruta = leituraAtual;
    ultimaMudanca = agora;
  }

  if ((agora - ultimaMudanca) >= DEBOUNCE_ESTAVEL &&
      estadoEstavel != leituraBruta) {
    estadoEstavel = leituraBruta;
  }

  return estadoEstavel;
}

// =====================================================
// REFERENCIA DO BNO
// =====================================================

float referenciaGraus = 0.0;
float referenciaFrenteTras = 0.0;
float referenciaEsqDir = 0.0;

bool referenciaDefinida = false;

float telemetriaHeading = 0.0f;
float telemetriaGyroZ = 0.0f;
float telemetriaUltra[4] = {};
uint8_t telemetriaUltraMascara = 0;
uint8_t telemetriaAtividadeIR[IR_SENSOR_COUNT] = {};
float telemetriaDirecaoIR = 0.0f;
bool telemetriaBNOValida = false;
bool telemetriaUltraValida = false;
bool telemetriaIRValida = false;
bool telemetriaBolaDetectada = false;
ProximidadeIR::Estimativa telemetriaProximidadeIR;
bool telemetriaRegiaoBrancaLDR[LDRFixed::QuantidadeRegioes] = {};
bool testeKickerNoBody = false;
bool testeKickerComunicacaoValida = false;

// Recebe um snapshot do Body para a IHM nao acoplar a renderizacao aos drivers
// UART/sensores. Flags de validade distinguem "zero medido" de "sem pacote".
void IHM_atualizarTelemetria(float headingDegrees, float gyroZDegreesPerSecond,
                             bool bnoValido, const float distanciasUltra[4],
                             uint8_t mascaraUltra, bool ultraValido,
                             const uint8_t atividadesIR[IR_SENSOR_COUNT],
                             float direcaoIR, bool bolaDetectada, bool irValido,
                             const ProximidadeIR::Estimativa &proximidade) {
  telemetriaHeading = headingDegrees;
  telemetriaGyroZ = gyroZDegreesPerSecond;
  telemetriaBNOValida = bnoValido;
  for (uint8_t i = 0; i < 4; ++i) telemetriaUltra[i] = distanciasUltra[i];
  telemetriaUltraMascara = mascaraUltra;
  telemetriaUltraValida = ultraValido;
  for (uint8_t i = 0; i < IR_SENSOR_COUNT; ++i) {
    telemetriaAtividadeIR[i] = atividadesIR[i];
  }
  telemetriaDirecaoIR = direcaoIR;
  telemetriaBolaDetectada = bolaDetectada;
  telemetriaIRValida = irValido;
  telemetriaProximidadeIR = proximidade;
}

// Copia estados LDR para exibi-los sem reamostrar hardware dentro da IHM.
void IHM_atualizarLDR(const bool regioesBrancas[LDRFixed::QuantidadeRegioes]) {
  for (uint8_t i = 0; i < LDRFixed::QuantidadeRegioes; ++i) {
    telemetriaRegiaoBrancaLDR[i] = regioesBrancas[i];
  }
}

// O Body consulta estas funcoes no loop para decidir qual maquina de estados roda.
bool IHM_testeTSSPAtivo() {
  return tela == TELA_TESTE_TSSP;
}

bool IHM_testeBNOMoveAtivo() {
  return tela == TELA_TESTE_BNO;
}

bool IHM_testeBNOMoveExecutando() {
  return tela == TELA_TESTE_BNO && bnoMoveExecutando;
}

void IHM_pararTesteBNOMove() {
  bnoMoveExecutando = false;
}

void IHM_obterGanhosPID(float &kp, float &ki, float &kd) {
  kp = kpBnoMove;
  ki = kiBnoMove;
  kd = kdBnoMove;
}

bool IHM_estrategiaAtaqueAtiva() {
  return tela == TELA_ATAQUE;
}

bool IHM_estrategiaDefesaAtiva() {
  return tela == TELA_DEFESA;
}

bool IHM_testeKickerAutomaticoAtivo() {
  return tela == TELA_TESTE_KICKER_AUTO;
}

bool IHM_testeKickerFimDeCursoAtivo() {
  return tela == TELA_TESTE_KICKER_FIM_CURSO;
}

void IHM_atualizarTesteKicker(bool comunicacaoValida) {
  testeKickerNoBody = true;
  testeKickerComunicacaoValida = comunicacaoValida;
}

bool IHM_obterBateria(float &percentual) {
  percentual = estimadorBateria.getPercentage();
  return estimadorBateria.hasReading() &&
         isfinite(estimadorBateria.getVoltage()) && isfinite(percentual);
}

// =====================================================
// LOGO DO LOBO
// =====================================================

const uint8_t logoLobo[] PROGMEM = {

  0x20, 0x00, 0x00, 0x00, 0x00, 0x06, 0xF0, 0x01,
  0x00, 0x00, 0x80, 0x07, 0x30, 0x07, 0x00, 0x00,
  0x60, 0x0C, 0x30, 0x0C, 0x00, 0x00, 0x30, 0x0C,
  0x10, 0x1B, 0x00, 0x00, 0xCC, 0x0C, 0x10, 0x66,
  0x00, 0x00, 0x66, 0x0C, 0x10, 0xCE, 0x00, 0x80,
  0x79, 0x08, 0x10, 0x3C, 0x03, 0xC0, 0x3C, 0x08,
  0x10, 0x7C, 0x06, 0x60, 0x3E, 0x08, 0x10, 0xF8,
  0x0C, 0x90, 0x1F, 0x08, 0x98, 0xF8, 0x1B, 0xC8,
  0x8F, 0x09, 0x98, 0xF8, 0x33, 0xEC, 0x9F, 0x19,
  0x98, 0xF1, 0xE7, 0xE7, 0x8F, 0x19, 0x88, 0xE1,
  0x07, 0xE0, 0x87, 0x19, 0x88, 0xF1, 0x3F, 0xFC,
  0x8F, 0x19, 0x08, 0xC1, 0x3F, 0xFE, 0xC3, 0x18,
  0x10, 0x83, 0x7F, 0xFE, 0xC1, 0x08, 0x30, 0xC6,
  0x7F, 0xFE, 0x63, 0x04, 0x20, 0xF0, 0x7F, 0xFE,
  0x37, 0x06, 0x30, 0xF8, 0xFF, 0xFF, 0x3F, 0x0E,
  0x18, 0xFC, 0xFF, 0xFF, 0x1F, 0x18, 0x18, 0xFC,
  0xFF, 0xFF, 0x3F, 0x1C, 0x30, 0xFF, 0xFF, 0xFF,
  0xFF, 0x0C, 0x98, 0x7F, 0xFE, 0x3F, 0xFE, 0x19,
  0xCC, 0xFF, 0xF0, 0x0F, 0xFF, 0x37, 0xE2, 0xFF,
  0xE4, 0x27, 0xFF, 0x63, 0x0F, 0xFC, 0xE3, 0xC7,
  0x3F, 0xF0, 0x18, 0xFF, 0xCF, 0xF3, 0xFF, 0x1C,
  0x08, 0x3F, 0xEF, 0xF3, 0xFC, 0x10, 0x1C, 0x7F,
  0xEE, 0x37, 0xFE, 0x18, 0x70, 0xFC, 0xFC, 0x3F,
  0x3F, 0x0E, 0x10, 0xDC, 0xFD, 0x9F, 0x3F, 0x0C,
  0x20, 0x30, 0xF9, 0x9F, 0x07, 0x06, 0xC0, 0x00,
  0x38, 0xDC, 0x03, 0x03, 0x80, 0x01, 0x18, 0xCC,
  0x83, 0x01, 0x80, 0x03, 0x30, 0xEC, 0xC1, 0x00,
  0x00, 0x03, 0x60, 0xE6, 0xC0, 0x00, 0x00, 0x06,
  0x00, 0x70, 0x60, 0x00, 0x00, 0x0C, 0x80, 0x39,
  0x18, 0x00, 0x00, 0x30, 0x00, 0x18, 0x0C, 0x00,
  0x00, 0x60, 0x00, 0x0C, 0x06, 0x00, 0x00, 0xC0,
  0x00, 0x84, 0x01, 0x00, 0x00, 0x00, 0x03, 0xC0,
  0x00, 0x00, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00,
  0x00, 0x00, 0x0C, 0x18, 0x00, 0x00, 0x00, 0x00,
  0x30, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x60, 0x03,
  0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00
};

// =====================================================
// NORMALIZA 0 - 360
// =====================================================

// Normaliza angulos para exibicao e comparacoes de rumo sem valores negativos.
float normalizar360(float valor) {

  while (valor >= 360.0) {
    valor -= 360.0;
  }

  while (valor < 0.0) {
    valor += 360.0;
  }

  return valor;
}

// =====================================================
// CARREGAMENTO
// =====================================================

// Apresenta a inicializacao da interface antes de liberar o menu principal.
void telaCarregamento() {

  for (
    int progresso = 0;
    progresso <= 100;
    progresso += 10
  ) {

    oled.clearBuffer();

    oled.drawXBMP(
      40,
      0,
      48,
      48,
      logoLobo
    );

    oled.drawFrame(
      8,
      50,
      112,
      8
    );

    int largura =
      map(
        progresso,
        0,
        100,
        0,
        108
      );

    if (largura > 0) {

      oled.drawBox(
        10,
        52,
        largura,
        4
      );
    }

    char texto[8];

    sprintf(
      texto,
      "%d%%",
      progresso
    );

    oled.setFont(
      u8g2_font_5x8_tr
    );

    oled.drawStr(
      56,
      64,
      texto
    );

    oled.sendBuffer();

    delay(30);
  }

  //define o tempo de exibição do logo do lobo antes de ir para o menu principal
  delay(100);
}

// =====================================================
// MENU PRINCIPAL
// =====================================================

// Renderizadores mantem desenho isolado da logica de botoes e do firmware.
void mostrarMenu() {

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_ncenB10_tr
  );

  oled.drawStr(
    43,
    10,
    "MENU"
  );

  oled.drawHLine(
    8,
    14,
    112
  );

  const char* nomes[] = {
    "ESTRATEGIA",
    "TELEMETRIA",
    "BATERIA",
    "TESTES"
  };

  oled.setFont(
    u8g2_font_6x10_tr
  );

  for (
    int i = 0;
    i < totalMenu;
    i++
  ) {

    int y =
      27 + i * 10;

    if (i == opcaoMenu) {

      oled.drawStr(
        4,
        y,
        ">"
      );
    }

    oled.drawStr(
      14,
      y,
      nomes[i]
    );
  }

  oled.sendBuffer();
}

// =====================================================
// MENU ESTRATEGIA
// =====================================================

// Mostra as duas estrategias; a execucao ocorre no Body apos confirmacao.
void mostrarMenuEstrategia() {

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.drawStr(
    33,
    10,
    "ESTRATEGIA"
  );

  oled.drawHLine(
    8,
    14,
    112
  );

  const char* nomes[] = {
    "ATAQUE",
    "DEFESA"
  };

  for (
    int i = 0;
    i < totalEstrategia;
    i++
  ) {

    int y =
      31 + i * 14;

    if (
      i == opcaoEstrategia
    ) {

      oled.drawStr(
        8,
        y,
        ">"
      );
    }

    oled.drawStr(
      20,
      y,
      nomes[i]
    );
  }

  oled.sendBuffer();
}

// =====================================================
// MENU TELEMETRIA
// =====================================================

// Lista paginas de diagnostico alimentadas pelos snapshots UART mais recentes.
void mostrarMenuTelemetria() {

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.drawStr(
    29,
    9,
    "TELEMETRIA"
  );

  oled.drawHLine(
    8,
    13,
    112
  );

  const char* nomes[] = {
    "BNO",
    "KICKER",
    "ULTRASSONICOS",
    "INFRA",
    "LDR",
    "MAC ADDRESS",
    "KICKER AUTO",
    "KICKER FIM CURSO"
  };

  // A lista rola mantendo a opcao visivel.
  const int primeiroItem = opcaoTelemetria > 4 ? opcaoTelemetria - 4 : 0;
  for (
    int i = primeiroItem;
    i < totalTelemetria && i < primeiroItem + 5;
    i++
  ) {

    int y =
      23 + (i - primeiroItem) * 9;

    if (
      i == opcaoTelemetria
    ) {

      oled.drawStr(
        4,
        y,
        ">"
      );
    }

    oled.drawStr(
      14,
      y,
      nomes[i]
    );
  }

  oled.sendBuffer();
}

// =====================================================
// MENU BNO
// =====================================================

// Oferece leitura, definicao de zero relativo e verificacao de comunicacao BNO.
void mostrarMenuBNO() {

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.drawStr(
    50,
    10,
    "BNO"
  );

  oled.drawHLine(
    8,
    14,
    112
  );

  const char* nomes[] = {
    "LEITURA",
    "ZERO",
    "STATUS UART"
  };

  oled.setFont(
    u8g2_font_6x10_tr
  );

  for (
    int i = 0;
    i < totalBNO;
    i++
  ) {

    int y =
      29 + i * 11;

    if (
      i == opcaoBNO
    ) {

      oled.drawStr(
        7,
        y,
        ">"
      );
    }

    oled.drawStr(
      20,
      y,
      nomes[i]
    );
  }

  oled.sendBuffer();
}

// =====================================================
// MENU CHECKLIST
// =====================================================

// Desloca a janela visivel para manter o item selecionado dentro do OLED.
void mostrarMenuChecklist() {

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.drawStr(
    30,
    10,
    "TESTES"
  );

  oled.drawHLine(
    8,
    14,
    112
  );

  int primeiroItem = opcaoChecklist - itensChecklistVisiveis + 1;
  if (primeiroItem < 0) primeiroItem = 0;
  if (primeiroItem > totalChecklist - itensChecklistVisiveis) {
    primeiroItem = totalChecklist - itensChecklistVisiveis;
  }

  for (int linha = 0; linha < itensChecklistVisiveis; ++linha) {
    const int indice = primeiroItem + linha;
    const int y = 24 + linha * 9;
    if (indice == opcaoChecklist) oled.drawStr(4, y, ">");
    oled.drawStr(14, y, NOMES_CHECKLIST[indice]);
  }

  oled.setFont(
    u8g2_font_5x8_tr
  );

  oled.drawStr(4, 62, "MEXER=PROX SEL=ABRIR");

  oled.sendBuffer();
}

// =====================================================
// MOSTRAR BNO
// =====================================================

void mostrarUltra() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(45, 9, "ULTRA");
  oled.drawHLine(8, 13, 112);
  oled.setFont(u8g2_font_5x8_tr);
  if (telemetriaUltraValida) {
    for (uint8_t i = 0; i < 4; ++i) {
      const uint8_t y = 25 + i * 9;
      char label[8];
      snprintf(label, sizeof(label), "U%u:", i + 1);
      oled.drawStr(8, y, label);
      if (telemetriaUltraMascara & (1U << i)) {
        oled.setCursor(38, y);
        oled.print(telemetriaUltra[i], 1);
        oled.print(" cm");
      } else {
        oled.drawStr(38, y, "SEM ECO");
      }
    }
  } else {
    oled.drawStr(20, 35, "SEM DADOS UART");
  }
  oled.drawStr(4, 62, "VOLTAR = TELEMETRIA");
  oled.sendBuffer();
}

// Telemetria do kicker: MEXER alterna entre esquerdo/direito e SELECIONAR
// aciona o lado escolhido.
void mostrarKicker() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(40, 9, "KICKER");
  oled.drawHLine(8, 13, 112);
  const char *nomes[] = {"KICKER ESQUERDO", "KICKER DIREITO"};
  for (int i = 0; i < 2; ++i) {
    const int y = 28 + i * 11;
    if (i == opcaoKicker) oled.drawStr(4, y, ">");
    oled.drawStr(14, y, nomes[i]);
  }
  oled.setFont(u8g2_font_5x8_tr);
  if (mensagemKicker) oled.drawStr(4, 54, mensagemKicker);
  oled.drawStr(4, 62, "MEXER=LADO SEL=CHUTAR");
  oled.sendBuffer();
}

void mostrarTesteKickerAutomatico() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(4, 8, "TESTE KICKER AUTO");
  if (!testeKickerNoBody) {
    oled.drawStr(4, 30, "DISPONIVEL NO BODY");
  } else {
    oled.drawStr(4, 17, digitalRead(ConfigPinos::Body::Kicker::FIM_CURSO) == HIGH
                            ? "FIM: ACIONADO" : "FIM: LIVRE");
    const uint8_t indices[] = {
        ULTRA_INDEX_RIGHT, ULTRA_INDEX_FRONT, ULTRA_INDEX_LEFT};
    const char *nomes[] = {"DIR:", "FRT:", "ESQ:"};
    for (uint8_t i = 0; i < 3; ++i) {
      const uint8_t y = 26 + i * 9;
      oled.drawStr(4, y, nomes[i]);
      if (telemetriaUltraValida &&
          (telemetriaUltraMascara & (1U << indices[i])) &&
          KickerLogica::distanciaValida(telemetriaUltra[indices[i]])) {
        oled.setCursor(35, y);
        oled.print(telemetriaUltra[indices[i]], 1);
        oled.print(" cm");
      } else {
        oled.drawStr(35, y, "SEM DADOS");
      }
    }
    float percentual = 0.0f;
    const bool bateriaValida = IHM_obterBateria(percentual);
    const bool bateriaOk = KickerLogica::bateriaPermite(bateriaValida, percentual);
    oled.drawStr(4, 53, testeKickerComunicacaoValida ? "UART:OK" : "UART:OFF");
    oled.drawStr(65, 53, bateriaOk ? "BAT:OK" : "BAT:OFF");
  }
  oled.drawStr(4, 63, "VOLTAR = SAIR");
  oled.sendBuffer();
}

void mostrarTesteKickerFimDeCurso() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(4, 8, "TESTE KICKER FIM CURSO");
  if (!testeKickerNoBody) {
    oled.drawStr(4, 30, "DISPONIVEL NO BODY");
  } else {
    oled.drawStr(4, 23, digitalRead(ConfigPinos::Body::Kicker::FIM_CURSO) == HIGH
                            ? "FIM: ACIONADO" : "FIM: LIVRE");
    float percentual = 0.0f;
    const bool bateriaValida = IHM_obterBateria(percentual);
    const bool bateriaOk = KickerLogica::bateriaPermite(bateriaValida, percentual);
    oled.drawStr(4, 35, bateriaOk ? "BATERIA: OK" : "BATERIA: BLOQUEADA");
    oled.drawStr(4, 48, "DISPARO: AMBOS");
  }
  oled.drawStr(4, 63, "VOLTAR = SAIR");
  oled.sendBuffer();
}

bool lerDistanciaUltra(float &distanciaCm) {
  bool encontrou = false;
  for (uint8_t i = 0; i < 4; ++i) {
    if (!(telemetriaUltraMascara & (1U << i))) continue;
    if (!encontrou || telemetriaUltra[i] < distanciaCm) {
      distanciaCm = telemetriaUltra[i];
      encontrou = true;
    }
  }
  return telemetriaUltraValida && encontrou;
}

// Exibe valores somente quando a telemetria esta marcada valida, evitando
// apresentar valores antigos/zerados como se fossem leituras atuais.
void mostrarBNO() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(47, 9, "BNO REMOTO");
  oled.setFont(u8g2_font_5x8_tr);
  if (telemetriaBNOValida) {
    float rumoRelativo = telemetriaHeading;
    if (referenciaDefinida) {
      rumoRelativo = normalizar360(rumoRelativo - referenciaGraus);
    }
    oled.drawStr(3, 23, "ANG BNO:");
    oled.setFont(u8g2_font_6x10_tr);
    oled.setCursor(48, 23);
    oled.print(telemetriaHeading, 1);
    oled.setFont(u8g2_font_5x8_tr);
    oled.drawStr(92, 23, "graus");
    oled.drawStr(3, 37, "RUMO:");
    oled.setFont(u8g2_font_6x10_tr);
    oled.setCursor(43, 37);
    oled.print(rumoRelativo, 1);
    oled.setFont(u8g2_font_5x8_tr);
    oled.drawStr(3, 51, "GIRO Z:");
    oled.setFont(u8g2_font_6x10_tr);
    oled.setCursor(51, 51);
    oled.print(telemetriaGyroZ, 1);
    oled.setFont(u8g2_font_5x8_tr);
    oled.drawStr(90, 51, "graus/s");
  } else {
    oled.drawStr(15, 32, "SEM DADOS UART");
  }
  oled.drawStr(4, 62, "SEL=ZERO  VOLTAR");
  oled.sendBuffer();
}

// =====================================================
// ZERO BNO
// =====================================================

// Captura a leitura remota atual como referencia apenas se o pacote for valido.
void zerarBNO() {
  if (telemetriaBNOValida) {
    referenciaGraus = telemetriaHeading;
    referenciaDefinida = true;
  }
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(30, 20, "ZERO BNO REMOTO");
  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(10, 40, telemetriaBNOValida ? "REFERENCIA ATUALIZADA" : "SEM DADOS DO HEAD");
  oled.sendBuffer();
  delay(500);
  tela = TELA_BNO_LEITURA;
  mostrarBNO();
}

// =====================================================
// STATUS DO BNO RECEBIDO PELO HEAD
// =====================================================

// =====================================================
// TESTE DO BNO
// =====================================================

// Mantem ganhos em RAM se o NVS estiver indisponivel e sinaliza a falha na tela.
bool testarBNO() {
  return telemetriaBNOValida;
}

bool salvarGanhosBnoMove() {
  Preferences preferencias;
  if (!preferencias.begin("bno-move", false)) {
    Serial.println("ERRO: nao foi possivel abrir ganhos BNO-Move no NVS.");
    bnoMoveErroNvs = true;
    return false;
  }
  const GanhosBnoMove ganhos = {
      VERSAO_GANHOS_BNO_MOVE, kpBnoMove, kiBnoMove, kdBnoMove};
  const bool salvo =
      preferencias.putBytes("pid", &ganhos, sizeof(ganhos)) == sizeof(ganhos);
  preferencias.end();
  if (!salvo) {
    Serial.println("ERRO: falha ao salvar ganhos BNO-Move no NVS.");
  }
  bnoMoveErroNvs = !salvo;
  return salvo;
}

bool ganhosBnoMoveValidos(const GanhosBnoMove &ganhos) {
  return ganhos.versao == VERSAO_GANHOS_BNO_MOVE &&
         isfinite(ganhos.kp) && ganhos.kp >= 0.0f &&
         ganhos.kp <= MAX_KP_BNO_MOVE &&
         isfinite(ganhos.ki) && ganhos.ki >= 0.0f &&
         ganhos.ki <= MAX_KI_BNO_MOVE &&
         isfinite(ganhos.kd) && ganhos.kd >= 0.0f &&
         ganhos.kd <= MAX_KD_BNO_MOVE;
}

void carregarGanhosBnoMove() {
  Preferences preferencias;
  if (!preferencias.begin("bno-move", false)) {
    Serial.println("ERRO: nao foi possivel abrir ganhos BNO-Move no NVS.");
    bnoMoveErroNvs = true;
    return;
  }

  GanhosBnoMove ganhos{};
  const size_t tamanho = preferencias.getBytesLength("pid");
  if (tamanho == 0) {
    preferencias.end();
    Serial.println("NVS sem ganhos BNO-Move; usando valores padrao.");
    return;
  }
  const bool leu = tamanho == sizeof(ganhos) &&
      preferencias.getBytes("pid", &ganhos, sizeof(ganhos)) == sizeof(ganhos);
  preferencias.end();
  if (!leu) {
    Serial.println("ERRO: registro de ganhos BNO-Move invalido no NVS.");
    bnoMoveErroNvs = true;
    return;
  }
  if (!ganhosBnoMoveValidos(ganhos)) {
    Serial.println("ERRO: ganhos BNO-Move invalidos no NVS; usando padroes.");
    bnoMoveErroNvs = true;
    return;
  }

  kpBnoMove = ganhos.kp;
  kiBnoMove = ganhos.ki;
  kdBnoMove = ganhos.kd;
}

void ajustarGanhoBnoMove(float direcao) {
  float valorAnterior = 0.0f;
  switch (opcaoBnoMove) {
    case 1:
      valorAnterior = kpBnoMove;
      kpBnoMove = constrain(kpBnoMove + direcao * PASSO_KP_BNO_MOVE,
                            0.0f, MAX_KP_BNO_MOVE);
      break;
    case 2:
      valorAnterior = kiBnoMove;
      kiBnoMove = constrain(kiBnoMove + direcao * PASSO_KI_BNO_MOVE,
                            0.0f, MAX_KI_BNO_MOVE);
      break;
    case 3:
      valorAnterior = kdBnoMove;
      kdBnoMove = constrain(kdBnoMove + direcao * PASSO_KD_BNO_MOVE,
                            0.0f, MAX_KD_BNO_MOVE);
      break;
    default:
      return;
  }
  const float valorAtual = opcaoBnoMove == 1 ? kpBnoMove
      : opcaoBnoMove == 2 ? kiBnoMove : kdBnoMove;
  if (valorAtual != valorAnterior) salvarGanhosBnoMove();
}

void mostrarTesteBNOMove() {
  char linha[32];
  oled.clearBuffer();
  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(0, 8, "BNO-MOVE");

  snprintf(linha, sizeof(linha), "BNO:%s H=%+.1f",
           telemetriaBNOValida ? "OK" : "SEM DADOS", telemetriaHeading);
  oled.drawStr(0, 18, linha);
  snprintf(linha, sizeof(linha), "GZ=%+.1f deg/s%s",
           telemetriaGyroZ, bnoMoveErroNvs ? " NVS!" : "");
  oled.drawStr(0, 27, linha);

  oled.setFont(u8g2_font_4x6_tr);
  snprintf(linha, sizeof(linha), "%cRUN/PAUSE: %s",
           opcaoBnoMove == 0 ? '>' : ' ',
           bnoMoveExecutando ? "PAUSAR" : "INICIAR");
  oled.drawStr(0, 37, linha);
  snprintf(linha, sizeof(linha), "%cKp=%0.4f",
           opcaoBnoMove == 1 ? '>' : ' ', kpBnoMove);
  oled.drawStr(0, 44, linha);
  snprintf(linha, sizeof(linha), "%cKi=%0.7f",
           opcaoBnoMove == 2 ? '>' : ' ', kiBnoMove);
  oled.drawStr(0, 51, linha);
  snprintf(linha, sizeof(linha), "%cKd=%0.5f",
           opcaoBnoMove == 3 ? '>' : ' ', kdBnoMove);
  oled.drawStr(0, 58, linha);
  oled.drawStr(0, 63, "MEXER:ITEM SEL:+ VOLTAR:-/SEGURE:SAIR");
  oled.sendBuffer();
}

void abrirTesteBNOMove() {
  tela = TELA_TESTE_BNO;
  opcaoBnoMove = 0;
  bnoMoveExecutando = false;
  voltarBnoMovePressionado = false;
  voltarBnoMoveLongo = false;
  mostrarTesteBNOMove();
}

// =====================================================
// INFORMACOES DE TESTE
// =====================================================

// Informa qual ambiente de teste corresponde ao item selecionado e o estado
// UART conhecido, sem executar automaticamente um firmware de bancada.
void mostrarInfoTesteChecklist() {
  const char *codigoPrincipal = "";
  const char *codigoSecundario = "";
  const char *status = "TESTE SEPARADO";

  switch (opcaoChecklist) {
    case 1:
      codigoPrincipal = "tssp-16-test";
      status = telemetriaIRValida ? "IR UART: DADOS OK" : "IR: SEM DADOS UART";
      break;
    case 2:
      codigoPrincipal = "motores-test";
      codigoSecundario = "motor-unitario-test";
      break;
    case 3:
      codigoPrincipal = "SEM ENV PIO";
      codigoSecundario = "TEST_SOLENOIDES/*.ino";
      break;
    case 4:
      codigoPrincipal = "uart-receptor-test";
      codigoSecundario = "uart-emissor-test";
      status = (telemetriaBNOValida || telemetriaIRValida)
                   ? "UART: DADOS OK"
                   : "UART: SEM DADOS";
      break;
    case 5:
      codigoPrincipal = "ldr-test";
      codigoSecundario = "w/branco  g/verde";
      break;
    default:
      codigoPrincipal = "TESTE NAO DISPONIVEL";
      break;
  }

  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(30, 9, "TESTES");
  oled.drawHLine(8, 13, 112);
  oled.drawStr(4, 25, NOMES_CHECKLIST[opcaoChecklist]);

  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(4, 36, status);
  oled.drawStr(4, 47, codigoPrincipal);
  if (codigoSecundario[0] != '\0') {
    oled.drawStr(4, 56, codigoSecundario);
  }
  oled.drawStr(4, 63, "VOLTAR = TESTES");
  oled.sendBuffer();
}

// Mostra atividade dos 16 sensores recebida do Head para diagnostico do anel.
void mostrarTesteTSSP() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_4x6_tr);

  if (telemetriaIRValida) {
    oled.drawStr(
        4, 8, telemetriaBolaDetectada ? "IR/TSSP  BOLA: SIM"
                                     : "IR/TSSP  BOLA: NAO");

    for (uint8_t i = 0; i < IR_SENSOR_COUNT; ++i) {
      const uint8_t coluna = i % 4;
      const uint8_t linha = i / 4;
      oled.setCursor(2 + coluna * 32, 20 + linha * 10);
      if (i + 1 < 10) oled.print('0');
      oled.print(i + 1);
      oled.print(':');
      oled.print(telemetriaAtividadeIR[i]);
      oled.print('%');
    }
  } else {
    oled.drawStr(4, 8, "IR/TSSP  SEM DADOS UART");
  }

  oled.drawStr(4, 63, "VOLTAR=TESTES");
  oled.sendBuffer();
}

// Executa sequencia de translacao com potencia reduzida; garante parada ao fim
// para que a rotina de checklist nao deixe motores energizados.
void testarMotoresNaTela() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(30, 9, "TESTES");
  oled.drawHLine(8, 13, 112);
  oled.drawStr(25, 30, "TESTE MOTORES");
  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(20, 45, "FRENTE ESQ TRAS DIR");
  oled.sendBuffer();

  motoresChecklist.begin();
  motoresChecklist.parar();
  executarTesteTranslacaoMotores(motoresChecklist, omniChecklist);
  motoresChecklist.parar();

  tela = TELA_INFO_TESTE;
  mostrarInfoTesteChecklist();
}

// =====================================================
// EM BREVE
// =====================================================

// Pagina informativa para itens da telemetria ainda sem funcionalidade ligada.
void mostrarEmBreve(
  const char* titulo
) {

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_ncenB10_tr
  );

  int largura =
    oled.getStrWidth(
      titulo
    );

  oled.drawStr(
    (128 - largura) / 2,
    16,
    titulo
  );

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.drawStr(
    34,
    37,
    "EM BREVE..."
  );

  oled.setFont(
    u8g2_font_5x8_tr
  );

  oled.drawStr(
    27,
    61,
    "VOLTAR = MENU"
  );

  oled.sendBuffer();
}

// Confirma na tela o modo selecionado; o controle efetivo roda no loop do Body.
void mostrarEstrategiaAtiva(const char *nome) {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(28, 12, "ESTRATEGIA");
  oled.drawHLine(8, 16, 112);
  oled.drawStr(36, 32, nome);
  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(18, 48, "ROBO EM OPERACAO");
  oled.drawStr(27, 61, "VOLTAR = MENU");
  oled.sendBuffer();
}

// =====================================================
// MOSTRAR RESULTADO DO RESET
// =====================================================

// Mantem o resultado visivel por um segundo para leitura do operador.
void mostrarResultadoReset(
  bool sucesso
) {

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.drawStr(
    38,
    12,
    "BNO UART"
  );

  if (sucesso) {

    oled.setFont(
      u8g2_font_ncenB10_tr
    );

    oled.drawStr(
      45,
      37,
      "OK!"
    );

    oled.setFont(
      u8g2_font_5x8_tr
    );

    oled.drawStr(
      24,
      52,
      "DADOS RECEBIDOS"
    );

  } else {

    oled.setFont(
      u8g2_font_ncenB10_tr
    );

    oled.drawStr(
      43,
      37,
      "ERRO!"
    );

    oled.setFont(
      u8g2_font_5x8_tr
    );

    oled.drawStr(
      20,
      52,
      "BNO NAO RESPONDE"
    );
  }

  oled.sendBuffer();

  delay(1000);
}

// Mantem hardware e interface sincronizados com os valores do perfil selecionado.
void aplicarLimiaresLDRNoFirmware() {
  if (ldrThresholdTarget == nullptr) {
    return;
  }
  const ConfigLDR::Limiares &padrao =
      perfilLDRAtual == 0 ? ConfigLDR::Ataque : ConfigLDR::Defesa;
  ldrThresholdTarget->setThresholds(
      limiaresLDR[0], limiaresLDR[1], limiaresLDR[2], limiaresLDR[3]);
  ldrThresholdTarget->setCenterSensorThresholds(
      limiaresCentroLDR, padrao.histereseCentro);
}

bool ajustarLimiarLDR(int variacao) {
  if (opcaoLDR < 2 || opcaoLDR >= totalRegioesLDR + 2) {
    return false;
  }

  const int regiao = opcaoLDR - 2;
  const int novoValor = static_cast<int>(limiaresLDR[regiao]) + variacao;
  const int valorLimitado = novoValor < 0
                                ? 0
                                : novoValor > LIMIAR_LDR_MAXIMO
                                      ? LIMIAR_LDR_MAXIMO
                                      : novoValor;
  if (valorLimitado == limiaresLDR[regiao]) {
    return false;
  }

  limiaresLDR[regiao] = static_cast<uint16_t>(valorLimitado);
  aplicarLimiaresLDRNoFirmware();
  return true;
}

bool ajustarLimiarCentroLDR(int variacao) {
  if (opcaoCentroLDR < 0 ||
      opcaoCentroLDR >= TOTAL_SENSORES_CENTRO_LDR) {
    return false;
  }

  const int novoValor =
      static_cast<int>(limiaresCentroLDR[opcaoCentroLDR]) + variacao;
  const int valorLimitado = novoValor < 0
                                ? 0
                                : novoValor > LIMIAR_LDR_MAXIMO
                                      ? LIMIAR_LDR_MAXIMO
                                      : novoValor;
  if (valorLimitado == limiaresCentroLDR[opcaoCentroLDR]) {
    return false;
  }

  limiaresCentroLDR[opcaoCentroLDR] =
      static_cast<uint16_t>(valorLimitado);
  aplicarLimiaresLDRNoFirmware();
  return true;
}

// Separa persistencia Attack/Defense para que a calibracao de uma nao sobrescreva a outra.
const char *namespaceLimiaresLDR() {
  return perfilLDRAtual == 0 ? "ldr-config" : "ldr-defense";
}

// Persiste snapshot com versao; retorna falha explicitamente para a IHM poder
// informar que o ajuste existe apenas em RAM e pode se perder ao reiniciar.
bool salvarLimiaresLDR() {
  if (!preferenciasLDRDisponiveis) {
    Serial.println("ERRO: preferencias LDR indisponiveis; limiares nao salvos.");
    erroLimiaresLDR = true;
    return false;
  }

  const ConfiguracaoLimiaresLDR config = {
      VERSAO_CONFIG_LDR, limiaresLDR[0], limiaresLDR[1], limiaresLDR[2],
      limiaresLDR[3]};
  if (preferenciasLDR.putBytes("thresholds", &config, sizeof(config)) !=
      sizeof(config)) {
    Serial.println("ERRO: falha ao salvar limiares LDR no NVS.");
    erroLimiaresLDR = true;
    return false;
  }
  if (preferenciasLDR.putBytes("center", limiaresCentroLDR,
                               sizeof(limiaresCentroLDR)) !=
      sizeof(limiaresCentroLDR)) {
    Serial.println("ERRO: falha ao salvar limiares centrais LDR no NVS.");
    erroLimiaresLDR = true;
    return false;
  }
  erroLimiaresLDR = false;
  return true;
}

// Carrega somente registro com tamanho, versao e faixa validos; em caso contrario
// conserva os padroes do perfil, em vez de aplicar dados corrompidos.
void carregarLimiaresLDR() {
  if (!preferenciasLDR.begin(namespaceLimiaresLDR(), false)) {
    Serial.println("ERRO: nao foi possivel abrir preferencias LDR no NVS.");
    erroLimiaresLDR = true;
    return;
  }
  preferenciasLDRDisponiveis = true;

  ConfiguracaoLimiaresLDR config{};
  ConfiguracaoLimiaresLDRV1 configV1{};
  const size_t tamanhoSalvo = preferenciasLDR.getBytesLength("thresholds");
  if (tamanhoSalvo == sizeof(config) &&
      preferenciasLDR.getBytes("thresholds", &config, sizeof(config)) ==
          sizeof(config) &&
      config.versao == VERSAO_CONFIG_LDR &&
      config.esquerda <= LIMIAR_LDR_MAXIMO &&
      config.direita <= LIMIAR_LDR_MAXIMO &&
      config.inferior <= LIMIAR_LDR_MAXIMO &&
      config.superior <= LIMIAR_LDR_MAXIMO) {
    limiaresLDR[0] = config.esquerda;
    limiaresLDR[1] = config.direita;
    limiaresLDR[2] = config.inferior;
    limiaresLDR[3] = config.superior;
  } else if (tamanhoSalvo == sizeof(configV1) &&
             preferenciasLDR.getBytes("thresholds", &configV1,
                                      sizeof(configV1)) == sizeof(configV1) &&
             configV1.versao == VERSAO_CONFIG_LDR_V1 &&
             configV1.esquerda <= LIMIAR_LDR_MAXIMO &&
             configV1.direita <= LIMIAR_LDR_MAXIMO &&
             configV1.inferior <= LIMIAR_LDR_MAXIMO) {
    limiaresLDR[0] = configV1.esquerda;
    limiaresLDR[1] = configV1.direita;
    limiaresLDR[2] = configV1.inferior;
    Serial.printf("NVS LDR v1 migrado; FRT usa o padrao %u.\n", limiaresLDR[3]);
  } else {
    Serial.printf("NVS sem limiares LDR validos; usando padroes %u/%u/%u/%u.\n",
                  limiaresLDR[0], limiaresLDR[1], limiaresLDR[2],
                  limiaresLDR[3]);
  }

  const size_t tamanhoCentroSalvo =
      preferenciasLDR.getBytesLength("center");
  if (tamanhoCentroSalvo == sizeof(limiaresCentroLDR)) {
    uint16_t centroSalvo[TOTAL_SENSORES_CENTRO_LDR];
    bool centroValido =
        preferenciasLDR.getBytes("center", centroSalvo, sizeof(centroSalvo)) ==
        sizeof(centroSalvo);
    for (uint8_t i = 0; i < TOTAL_SENSORES_CENTRO_LDR; ++i) {
      centroValido = centroValido &&
          centroSalvo[i] <= LIMIAR_LDR_MAXIMO;
    }
    if (centroValido) {
      for (uint8_t i = 0; i < TOTAL_SENSORES_CENTRO_LDR; ++i) {
        limiaresCentroLDR[i] = centroSalvo[i];
      }
    } else {
      Serial.println(
          "ERRO: limiares centrais LDR invalidos no NVS; usando padroes.");
      erroLimiaresLDR = true;
    }
  } else if (tamanhoCentroSalvo != 0) {
    Serial.println(
        "ERRO: tamanho dos limiares centrais LDR invalido no NVS.");
    erroLimiaresLDR = true;
  }
}

// Recupera a ultima estrategia selecionada para aplicar seus limiares no boot.
void carregarPerfilLDR() {
  Preferences preferenciasPerfil;
  if (!preferenciasPerfil.begin("ldr-perfil", true)) {
    // O namespace ainda nao existe apos apagar a flash.
    if (!preferenciasPerfil.begin("ldr-perfil", false)) {
      Serial.println("ERRO: nao foi possivel abrir perfil LDR no NVS.");
      erroPerfilLDR = true;
      return;
    }
  }
  const uint8_t perfil = preferenciasPerfil.getUChar("ativo", 0);
  preferenciasPerfil.end();
  if (perfil > 1) {
    Serial.println("ERRO: perfil LDR invalido no NVS; usando limiar 1.");
    erroPerfilLDR = true;
    return;
  }
  perfilLDRAtual = perfil;
}

// Inicializa os limiares perifericos com o perfil antes de sobrepor ajustes NVS.
void definirPadroesLDR() {
  const ConfigLDR::Limiares &padrao =
      perfilLDRAtual == 0 ? ConfigLDR::Ataque : ConfigLDR::Defesa;
  limiaresLDR[0] = padrao.esquerda;
  limiaresLDR[1] = padrao.direita;
  limiaresLDR[2] = padrao.inferior;
  limiaresLDR[3] = padrao.superior;
  for (uint8_t i = 0; i < TOTAL_SENSORES_CENTRO_LDR; ++i) {
    limiaresCentroLDR[i] = padrao.centro[i];
  }
}

// Troca de perfil atomica do ponto de vista da aplicacao: fecha NVS anterior,
// carrega novo namespace, aplica limiares e reinicializa classificacao do sensor.
void selecionarPerfilLDR(uint8_t perfil) {
  if (preferenciasLDRDisponiveis) {
    preferenciasLDR.end();
    preferenciasLDRDisponiveis = false;
  }
  perfilLDRAtual = perfil;
  definirPadroesLDR();
  if (!preferenciasLDR.begin(namespaceLimiaresLDR(), false)) {
    Serial.println("ERRO: nao foi possivel abrir preferencias LDR no NVS.");
  } else {
    preferenciasLDRDisponiveis = true;
  }
  aplicarLimiaresLDRNoFirmware();
  if (ldrThresholdTarget != nullptr) {
    ldrThresholdTarget->resetClassification();
  }
  salvarLimiaresLDR();
  Preferences preferenciasPerfil;
  if (!preferenciasPerfil.begin("ldr-perfil", false)) {
    Serial.println("ERRO: nao foi possivel salvar perfil LDR no NVS.");
    erroPerfilLDR = true;
    return;
  }
  const bool perfilSalvo = preferenciasPerfil.putUChar("ativo", perfil) == 1;
  preferenciasPerfil.end();
  if (!perfilSalvo) {
    Serial.println("ERRO: falha ao salvar perfil LDR no NVS.");
  }
  erroPerfilLDR = !perfilSalvo;
}

// Injeta o driver criado pelo Body; a IHM nao assume propriedade do objeto.
void IHM_conectarLDR(LDRFixed *ldr) {
  ldrThresholdTarget = ldr;
}

// Mostra medias, limiares ativos e estados sem iniciar uma nova leitura ADC.
void mostrarLDR() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(4, 9, "LDR: M=MEDIA L=LIMIAR");
  oled.drawHLine(8, 13, 112);

  oled.drawStr(4, 22, "CENTRO");
  oled.setCursor(48, 22);
  oled.print(telemetriaRegiaoBrancaLDR[LDRFixed::Centro] ? "BRANCO" : "VERDE");

  const LDRFixed::Regiao regioes[] = {
      LDRFixed::Esquerda, LDRFixed::Direita, LDRFixed::Inferior,
      LDRFixed::Superior};
  const char *const nomes[] = {"ESQ", "DIR", "INF", "FRT"};
  for (uint8_t i = 0; i < 4; ++i) {
    const uint8_t y = 30 + i * 8;
    oled.drawStr(4, y, nomes[i]);
    oled.setCursor(28, y);
    if (ldrThresholdTarget != nullptr) {
      oled.print("M");
      oled.print(ldrThresholdTarget->getAverage(regioes[i]));
      oled.print(" L");
      oled.print(ldrThresholdTarget->getThreshold(regioes[i]));
    } else {
      oled.print("SEM DADOS");
    }
    oled.setCursor(91, y);
    oled.print(telemetriaRegiaoBrancaLDR[regioes[i]] ? "BRANCO" : "VERDE");
  }

  oled.drawStr(4, 63, "SEL=CENTRO  VOLTAR=SAIR");
  oled.sendBuffer();
}

void mostrarLDRCentro() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(4, 9, telemetriaRegiaoBrancaLDR[LDRFixed::Centro]
                         ? "CENTRO: BRANCO"
                         : "CENTRO: VERDE");
  oled.drawHLine(8, 13, 112);
  oled.setFont(u8g2_font_4x6_tr);

  for (uint8_t i = 0; i < 7; ++i) {
    const uint8_t y = 19 + i * 6;
    oled.setCursor(4, y);
    oled.print("C");
    oled.print(i + 1);
    oled.print(" M");
    if (ldrThresholdTarget != nullptr) {
      oled.print(ldrThresholdTarget->getCenterSensorReading(i));
      oled.print(" L");
      oled.print(ldrThresholdTarget->getCenterSensorThreshold(i));
    } else {
      oled.print("SEM DADOS");
    }
  }

  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(4, 63, "VOLTAR=RESUMO");
  oled.sendBuffer();
}

// Exibe MAC local, MACs online e o estado do vinculo ESP-NOW com o outro robo.
void formatarMac(char *saida, const uint8_t mac[6]) {
  snprintf(saida, 18, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2],
           mac[3], mac[4], mac[5]);
}

void mostrarMac() {
  char texto[18];
  uint8_t local[6];
  ESPNowLink::macLocal(local);
  formatarMac(texto, local);

  oled.clearBuffer();
  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(4, 8, "MAC ADDRESS LOCAL");
  oled.drawStr(4, 17, texto);
  oled.drawHLine(4, 20, 120);

  uint8_t macs[ESPNowLink::MAX_CANDIDATOS][6];
  const uint8_t total =
      ESPNowLink::candidatosOnline(macs, ESPNowLink::MAX_CANDIDATOS);
  if (total > 0 && opcaoMac >= total) opcaoMac = total - 1;

  if (ESPNowLink::possuiVinculo()) {
    uint8_t parceiro[6];
    ESPNowLink::macVinculo(parceiro);
    formatarMac(texto, parceiro);
    if (ESPNowLink::conectado()) {
      oled.drawStr(4, 30, "CONECTADO:");
      oled.drawStr(4, 39, texto);
      oled.drawStr(4, 48, ESPNowLink::defesaParceiroDesligada()
                              ? "RoboDefOn: FALSE" : "RoboDefOn: TRUE");
    } else {
      oled.drawStr(4, 30, "CONEXAO SENDO DEFINIDA:");
      oled.drawStr(4, 39, texto);
    }
    oled.drawStr(4, 62, "SELECIONAR = ESQUECER");
  } else if (total == 0) {
    oled.drawStr(4, 34, "NENHUM MAC ONLINE");
  } else if (total == 1) {
    formatarMac(texto, macs[0]);
    oled.drawStr(4, 30, "1 MAC ADDRESS");
    oled.drawStr(4, 39, "CONEXAO AUTOMATICA");
    oled.drawStr(4, 48, texto);
  } else {
    const int primeiro = opcaoMac > 2 ? opcaoMac - 2 : 0;
    for (int linha = 0; linha < 3 && primeiro + linha < total; ++linha) {
      const int item = primeiro + linha;
      const uint8_t y = 30 + linha * 9;
      formatarMac(texto, macs[item]);
      if (item == opcaoMac) oled.drawStr(2, y, ">");
      oled.drawStr(10, y, texto);
    }
    oled.drawStr(4, 62, "MEXER=PROX SEL=ESCOLHER");
  }
  oled.sendBuffer();
}

// Exibe selecao de perfil e limiares perifericos; erros de NVS ficam visiveis
// para que o operador saiba se o ajuste sobrevive a um reinicio.
void mostrarLDRAjuste() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(4, 9, "LDR TESTE");
  oled.setCursor(88, 9);
  oled.print(perfilLDRAtual == 0 ? "L1" : "L2");
  oled.drawHLine(8, 13, 112);
  oled.setFont(u8g2_font_5x8_tr);

  constexpr int TOTAL_OPCOES_LDR = totalRegioesLDR + 3;
  const int primeiroItem = opcaoLDR > 3 ? opcaoLDR - 3 : 0;
  for (int linha = 0; linha < 4; ++linha) {
    const int item = primeiroItem + linha;
    if (item >= TOTAL_OPCOES_LDR) break;
    const uint8_t y = 22 + linha * 9;
    if (item == opcaoLDR) {
      oled.drawStr(4, y, ">");
      if (ajustandoLDR) {
        oled.drawStr(58, y, "*");
      }
    }
    if (item < 2) {
      oled.drawStr(16, y, item == 0 ? "LIMIAR 1" : "LIMIAR 2");
    } else if (item < totalRegioesLDR + 2) {
      oled.drawStr(16, y, NOMES_LDR[item - 2]);
      oled.setCursor(65, y);
      oled.print(limiaresLDR[item - 2]);
    } else {
      oled.drawStr(16, y, "CENTRO");
    }
  }

  oled.drawStr(4, 62, (erroLimiaresLDR || erroPerfilLDR) ? "ERRO NO NVS" :
               ajustandoLDR ? "SEL +10 MEXER -10" : "MEXER=PROX SEL=ESCOLHER");
  oled.sendBuffer();
}

void mostrarLDRAjusteCentro() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(4, 9, "LIMIARES CENTRO");
  oled.setCursor(95, 9);
  oled.print(perfilLDRAtual == 0 ? "L1" : "L2");
  oled.drawHLine(8, 13, 112);
  oled.setFont(u8g2_font_5x8_tr);

  const int primeiroItem = opcaoCentroLDR > 3 ? opcaoCentroLDR - 3 : 0;
  for (int linha = 0; linha < 4; ++linha) {
    const int sensor = primeiroItem + linha;
    if (sensor >= TOTAL_SENSORES_CENTRO_LDR) break;
    const uint8_t y = 23 + linha * 9;
    if (sensor == opcaoCentroLDR) {
      oled.drawStr(4, y, ">");
      if (ajustandoCentroLDR) oled.drawStr(17, y, "*");
    }
    oled.setCursor(25, y);
    oled.print("S");
    oled.print(sensor + 1);
    oled.print(" L");
    oled.print(limiaresCentroLDR[sensor]);
    if (ldrThresholdTarget != nullptr) {
      oled.print(" M");
      oled.print(ldrThresholdTarget->getCenterSensorReading(sensor));
    }
  }

  oled.drawStr(4, 62, (erroLimiaresLDR || erroPerfilLDR) ? "ERRO NO NVS" :
               ajustandoCentroLDR ? "SEL +10 MEXER -10" :
                                    "MEXER=PROX SEL=AJUSTAR");
  oled.sendBuffer();
}

// Exibe direcao IR somente quando pacote recente e bola detectada sao validos.
void mostrarInfra() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(45, 9, "INFRA IR");
  oled.setFont(u8g2_font_5x8_tr);
  if (!telemetriaIRValida) {
    oled.drawStr(18, 36, "SEM DADOS UART");
  } else if (!telemetriaBolaDetectada) {
    oled.drawStr(12, 36, "BOLA NAO DETECTADA");
  } else {
    oled.setCursor(4, 23);
    oled.print("ANGULO: ");
    oled.print(telemetriaDirecaoIR, 1);
    oled.setCursor(4, 34);
    oled.print("PROX: ");
    if (telemetriaProximidadeIR.valida) {
      oled.print(ProximidadeIR::descricao(telemetriaProximidadeIR.indice));
      oled.print(' ');
      oled.print(telemetriaProximidadeIR.indice * 100.0f, 0);
      oled.print('%');
      oled.setCursor(4, 45);
      oled.print("MAX:");
      oled.print(telemetriaProximidadeIR.atividadeMaxima);
      oled.print("% ATIVOS:");
      oled.print(telemetriaProximidadeIR.sensoresAtivos);
    } else {
      oled.print("SEM ESTIMATIVA");
    }
  }
  oled.drawStr(4, 63, "VOLTAR = TELEMETRIA");
  oled.sendBuffer();
}

// Apresenta a tensao medida e o alerta de bateria baixa.
void mostrarBateria() {
  const float voltage = estimadorBateria.getVoltage();
  const bool readingValid = estimadorBateria.hasReading() && isfinite(voltage);

  oled.clearBuffer();
  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(4, 9, "BATERIA");
  oled.drawStr(94, 9, "VOLTAR");
  oled.drawHLine(4, 12, 120);

  if (readingValid) {
    oled.setCursor(4, 21);
    oled.print("Tensao: ");
    oled.print(voltage, 2);
    oled.print("V");

    oled.setCursor(4, 30);
    oled.print("Bateria: ");
    oled.print(estimadorBateria.getPercentage(), 0);
    oled.print("%");

    oled.drawStr(4, 40, estimadorBateria.isLow()
                            ? "ALERTA: BATERIA BAIXA!"
                            : "BATERIA OK");
  } else {
    oled.drawStr(16, 32, "LEITURA INVALIDA");
    oled.drawStr(12, 44, "VERIFIQUE BATERIA");
  }
  oled.sendBuffer();
}

// Inicializa botoes, estimador, OLED e perfis persistidos antes do primeiro menu.
void IHM_begin() {
  pinMode(BOTAO_SELECIONAR, INPUT_PULLDOWN);
  pinMode(BOTAO_MEXER, INPUT_PULLDOWN);
  pinMode(BOTAO_VOLTAR, INPUT_PULLDOWN);
  estimadorBateria.begin();
  ultimaAtualizacaoBateriaMs = millis();
  Wire.begin(SDA_PIN, SCK_PIN);
  oled.setI2CAddress(0x3C << 1);
  oled.begin();
  carregarPerfilLDR();
  definirPadroesLDR();
  carregarLimiaresLDR();
  carregarGanhosBnoMove();
  aplicarLimiaresLDRNoFirmware();
  telaCarregamento();
  mostrarMenu();
}

// Faz uma iteracao cooperativa da interface: debounce, atualizacao de bateria,
// navegacao da maquina de telas e renderizacao periodica das paginas dinamicas.
void IHM_update() {
  // Atualiza todos os botoes em cada iteracao para nao perder cliques durante
  // tarefas de renderizacao; o segundo debounce limita repeticao por pressao.
  const bool selecionar = lerBotaoEstavel(
      BOTAO_SELECIONAR, leituraBrutaSelecionar, estadoEstavelSelecionar,
      ultimaMudancaBrutaSelecionar);
  const bool mexer = lerBotaoEstavel(
      BOTAO_MEXER, leituraBrutaMexer, estadoEstavelMexer,
      ultimaMudancaBrutaMexer);
  const bool voltar = lerBotaoEstavel(
      BOTAO_VOLTAR, leituraBrutaVoltar, estadoEstavelVoltar,
      ultimaMudancaBrutaVoltar);
  const unsigned long agora = millis();
  if (agora - ultimaAtualizacaoBateriaMs >= UPDATE_PERIOD_MS) {
    estimadorBateria.update(agora - ultimaAtualizacaoBateriaMs);
    ultimaAtualizacaoBateriaMs = agora;
  }
  const bool clicouSelecionar = estadoAnteriorSelecionar == LOW && selecionar == HIGH &&
                                agora - ultimoSelecionar > DEBOUNCE;
  const bool clicouMexer = estadoAnteriorMexer == LOW && mexer == HIGH &&
                           agora - ultimoMexer > DEBOUNCE;
  const bool clicouVoltar = estadoAnteriorVoltar == LOW && voltar == HIGH &&
                            agora - ultimoVoltar > DEBOUNCE;
  const bool pressionouVoltar = estadoAnteriorVoltar == LOW && voltar == HIGH;
  const bool soltouVoltar = estadoAnteriorVoltar == HIGH && voltar == LOW;

  // MEXER percorre opcoes ou ajusta valores; a mesma tecla muda de papel
  // conforme o estado da tela para manter a operacao com tres botoes.
  if (clicouMexer) {
    ultimoMexer = agora;
    if (tela == TELA_MENU) {
      opcaoMenu = (opcaoMenu + 1) % totalMenu;
      mostrarMenu();
    } else if (tela == TELA_ESTRATEGIA) {
      opcaoEstrategia = (opcaoEstrategia + 1) % totalEstrategia;
      mostrarMenuEstrategia();
    } else if (tela == TELA_TELEMETRIA) {
      opcaoTelemetria = (opcaoTelemetria + 1) % totalTelemetria;
      mostrarMenuTelemetria();
    } else if (tela == TELA_KICKER) {
      opcaoKicker = (opcaoKicker + 1) % 2;
      mensagemKicker = nullptr;
      mostrarKicker();
    } else if (tela == TELA_MENU_BNO) {
      opcaoBNO = (opcaoBNO + 1) % totalBNO;
      mostrarMenuBNO();
    } else if (tela == TELA_CHECKLIST) {
      opcaoChecklist = (opcaoChecklist + 1) % totalChecklist;
      mostrarMenuChecklist();
    } else if (tela == TELA_TESTE_BNO) {
      opcaoBnoMove = (opcaoBnoMove + 1) % 4;
      mostrarTesteBNOMove();
    } else if (tela == TELA_LDR_AJUSTE) {
      if (ajustandoLDR) {
        if (ajustarLimiarLDR(-10)) {
          salvarLimiaresLDR();
        }
        repeticaoAjusteLDR = RepeticaoAjusteLDR::Diminuir;
        inicioRepeticaoAjusteLDR = agora;
        ultimaRepeticaoAjusteLDR = agora;
        alteracaoRepetidaLDRPendente = false;
      } else {
        opcaoLDR = (opcaoLDR + 1) % (totalRegioesLDR + 3);
      }
      mostrarLDRAjuste();
    } else if (tela == TELA_LDR_CENTRO_AJUSTE) {
      if (ajustandoCentroLDR) {
        if (ajustarLimiarCentroLDR(-10)) {
          salvarLimiaresLDR();
        }
        repeticaoAjusteLDR = RepeticaoAjusteLDR::Diminuir;
        inicioRepeticaoAjusteLDR = agora;
        ultimaRepeticaoAjusteLDR = agora;
        alteracaoRepetidaLDRPendente = false;
      } else {
        opcaoCentroLDR =
            (opcaoCentroLDR + 1) % TOTAL_SENSORES_CENTRO_LDR;
      }
      mostrarLDRAjusteCentro();
    } else if (tela == TELA_INFO_TESTE) {
      mostrarInfoTesteChecklist();
    } else if (tela == TELA_MAC) {
      opcaoMac++;
      mostrarMac();
    }
  }

  // Cada confirmacao e despachada pela tela atual, tornando explicita a maquina
  // de estados da interface e evitando que um clique tenha efeitos em outra pagina.
  if (clicouSelecionar) {
    ultimoSelecionar = agora;
    switch (tela) {
      case TELA_MENU:
        if (opcaoMenu == 0) {
          tela = TELA_ESTRATEGIA;
          opcaoEstrategia = 0;
          mostrarMenuEstrategia();
        } else if (opcaoMenu == 1) {
          tela = TELA_TELEMETRIA;
          opcaoTelemetria = 0;
          mostrarMenuTelemetria();
        } else if (opcaoMenu == 3) {
          tela = TELA_CHECKLIST;
          opcaoChecklist = 0;
          mostrarMenuChecklist();
        } else {
          tela = TELA_BATERIA;
          mostrarBateria();
        }
        break;
      case TELA_ESTRATEGIA:
        tela = opcaoEstrategia == 0 ? TELA_ATAQUE : TELA_DEFESA;
        ESPNowLink::iniciarBusca();
        if (tela == TELA_ATAQUE) {
          mostrarEstrategiaAtiva("ATAQUE ATIVO");
        } else {
          mostrarEstrategiaAtiva("DEFESA ATIVA");
        }
        break;
      case TELA_TELEMETRIA:
        switch (opcaoTelemetria) {
          case 0:
            tela = TELA_MENU_BNO;
            opcaoBNO = 0;
            mostrarMenuBNO();
            break;
          case 1:
            tela = TELA_KICKER;
            opcaoKicker = 0;
            mensagemKicker = nullptr;
            mostrarKicker();
            break;
          case 2:
            tela = TELA_ULTRA;
            mostrarUltra();
            break;
          case 3:
            tela = TELA_INFRA;
            mostrarInfra();
            break;
          case 4:
            tela = TELA_LDR;
            mostrarLDR();
            break;
          case 5:
            tela = TELA_MAC;
            opcaoMac = 0;
            ESPNowLink::iniciarBusca();
            mostrarMac();
            break;
          case 6:
            tela = TELA_TESTE_KICKER_AUTO;
            mostrarTesteKickerAutomatico();
            break;
          case 7:
            tela = TELA_TESTE_KICKER_FIM_CURSO;
            mostrarTesteKickerFimDeCurso();
            break;
        }
        break;
      case TELA_MAC:
        if (ESPNowLink::possuiVinculo()) {
          ESPNowLink::esquecerVinculo();
        } else {
          uint8_t macs[ESPNowLink::MAX_CANDIDATOS][6];
          const uint8_t total =
              ESPNowLink::candidatosOnline(macs, ESPNowLink::MAX_CANDIDATOS);
          if (total > 0) {
            ESPNowLink::selecionarMac(macs[opcaoMac < total ? opcaoMac : 0]);
          }
        }
        mostrarMac();
        break;
      case TELA_KICKER:
        mensagemKicker = Kicker::dispararManual(opcaoKicker == 1)
            ? "DISPARADO"
            : "AGUARDE (COOLDOWN)";
        mostrarKicker();
        break;
      case TELA_MENU_BNO:
        if (opcaoBNO == 0) {
          tela = TELA_BNO_LEITURA;
          mostrarBNO();
        } else if (opcaoBNO == 1) {
          zerarBNO();
        } else {
          tela = TELA_BNO_RESET;
          mostrarResultadoReset(testarBNO());
          if (telemetriaBNOValida) {
            tela = TELA_BNO_LEITURA;
            mostrarBNO();
          } else {
            tela = TELA_MENU_BNO;
            mostrarMenuBNO();
          }
        }
        break;
      case TELA_BNO_LEITURA:
        zerarBNO();
        break;
      case TELA_LDR:
        tela = TELA_LDR_CENTRO;
        mostrarLDRCentro();
        break;
      case TELA_LDR_CENTRO:
        tela = TELA_LDR;
        mostrarLDR();
        break;
      case TELA_LDR_AJUSTE:
        if (ajustandoLDR) {
          if (ajustarLimiarLDR(10)) {
            salvarLimiaresLDR();
          }
          repeticaoAjusteLDR = RepeticaoAjusteLDR::Aumentar;
          inicioRepeticaoAjusteLDR = agora;
          ultimaRepeticaoAjusteLDR = agora;
          alteracaoRepetidaLDRPendente = false;
        } else if (opcaoLDR < 2) {
          selecionarPerfilLDR(static_cast<uint8_t>(opcaoLDR));
        } else if (opcaoLDR < totalRegioesLDR + 2) {
          ajustandoLDR = true;
        } else {
          tela = TELA_LDR_CENTRO_AJUSTE;
          opcaoCentroLDR = 0;
          ajustandoCentroLDR = false;
        }
        if (tela == TELA_LDR_CENTRO_AJUSTE) {
          mostrarLDRAjusteCentro();
        } else {
          mostrarLDRAjuste();
        }
        break;
      case TELA_LDR_CENTRO_AJUSTE:
        if (ajustandoCentroLDR) {
          if (ajustarLimiarCentroLDR(10)) {
            salvarLimiaresLDR();
          }
          repeticaoAjusteLDR = RepeticaoAjusteLDR::Aumentar;
          inicioRepeticaoAjusteLDR = agora;
          ultimaRepeticaoAjusteLDR = agora;
          alteracaoRepetidaLDRPendente = false;
        } else {
          ajustandoCentroLDR = true;
        }
        mostrarLDRAjusteCentro();
        break;
      case TELA_CHECKLIST:
        if (opcaoChecklist == 0) {
          abrirTesteBNOMove();
        } else if (opcaoChecklist == 1) {
          tela = TELA_TESTE_TSSP;
          mostrarTesteTSSP();
        } else if (opcaoChecklist == 2) {
          testarMotoresNaTela();
        } else if (opcaoChecklist == 5) {
          tela = TELA_LDR_AJUSTE;
          opcaoLDR = 0;
          ajustandoLDR = false;
          ajustandoCentroLDR = false;
          mostrarLDRAjuste();
        } else {
          tela = TELA_INFO_TESTE;
          mostrarInfoTesteChecklist();
        }
        break;
      case TELA_TESTE_BNO:
        if (opcaoBnoMove == 0) {
          if (!bnoMoveExecutando && !telemetriaBNOValida) {
            Serial.println(
                "BNO-Move nao iniciado: aguardando telemetria BNO recente.");
          } else {
            bnoMoveExecutando = !bnoMoveExecutando;
          }
        } else {
          ajustarGanhoBnoMove(1.0f);
        }
        mostrarTesteBNOMove();
        break;
      default:
        break;
    }
  }

  // VOLTAR encerra o contexto atual e restaura a tela pai; telas de estrategia
  // retornam ao menu, o que tambem faz o Body interromper a movimentacao.
  if (tela == TELA_TESTE_BNO) {
    if (pressionouVoltar) {
      ultimoVoltar = agora;
      voltarBnoMovePressionado = true;
      voltarBnoMoveLongo = false;
      inicioVoltarBnoMoveMs = agora;
    }
    if (voltarBnoMovePressionado && voltar &&
        !voltarBnoMoveLongo &&
        (uint32_t)(agora - inicioVoltarBnoMoveMs) >=
            TEMPO_SEGURAR_VOLTAR_BNO_MS) {
      voltarBnoMoveLongo = true;
      voltarBnoMovePressionado = false;
      bnoMoveExecutando = false;
      tela = TELA_CHECKLIST;
      mostrarMenuChecklist();
    } else if (soltouVoltar && voltarBnoMovePressionado) {
      voltarBnoMovePressionado = false;
      ajustarGanhoBnoMove(-1.0f);
      mostrarTesteBNOMove();
    }
  }

  if (clicouVoltar && tela != TELA_TESTE_BNO) {
    ultimoVoltar = agora;
    if (tela == TELA_MENU) {
      mostrarMenu();
    } else if (tela == TELA_MENU_BNO) {
      tela = TELA_TELEMETRIA;
      mostrarMenuTelemetria();
    } else if (tela == TELA_BNO_LEITURA) {
      tela = TELA_MENU_BNO;
      mostrarMenuBNO();
    } else if (tela == TELA_TELEMETRIA || tela == TELA_CHECKLIST) {
      tela = TELA_MENU;
      mostrarMenu();
    } else if (tela == TELA_ESTRATEGIA || tela == TELA_ATAQUE ||
               tela == TELA_DEFESA) {
      tela = TELA_MENU;
      mostrarMenu();
    } else if (tela == TELA_LDR_CENTRO) {
      tela = TELA_LDR;
      mostrarLDR();
    } else if (tela == TELA_ULTRA || tela == TELA_INFRA || tela == TELA_LDR ||
               tela == TELA_KICKER || tela == TELA_TESTE_KICKER_AUTO ||
               tela == TELA_TESTE_KICKER_FIM_CURSO || tela == TELA_MAC) {
      ajustandoLDR = false;
      tela = TELA_TELEMETRIA;
      mostrarMenuTelemetria();
    } else if (tela == TELA_LDR_CENTRO_AJUSTE && ajustandoCentroLDR) {
      ajustandoCentroLDR = false;
      mostrarLDRAjusteCentro();
    } else if (tela == TELA_LDR_CENTRO_AJUSTE) {
      ajustandoCentroLDR = false;
      tela = TELA_LDR_AJUSTE;
      mostrarLDRAjuste();
    } else if (tela == TELA_LDR_AJUSTE) {
      ajustandoLDR = false;
      tela = TELA_CHECKLIST;
      mostrarMenuChecklist();
    } else if (tela == TELA_TESTE_TSSP) {
      tela = TELA_CHECKLIST;
      mostrarMenuChecklist();
    } else if (tela == TELA_INFO_TESTE) {
      tela = TELA_CHECKLIST;
      mostrarMenuChecklist();
    } else {
      tela = TELA_MENU;
      mostrarMenu();
    }
  }

  if (((tela == TELA_LDR_AJUSTE && ajustandoLDR) ||
       (tela == TELA_LDR_CENTRO_AJUSTE && ajustandoCentroLDR)) &&
      repeticaoAjusteLDR != RepeticaoAjusteLDR::Nenhuma) {
    const bool botaoPressionado =
        repeticaoAjusteLDR == RepeticaoAjusteLDR::Aumentar ? selecionar : mexer;
    if (!botaoPressionado) {
      if (alteracaoRepetidaLDRPendente) {
        salvarLimiaresLDR();
      }
      repeticaoAjusteLDR = RepeticaoAjusteLDR::Nenhuma;
      alteracaoRepetidaLDRPendente = false;
    } else if (agora - inicioRepeticaoAjusteLDR >=
                   ATRASO_REPETICAO_LDR_MS &&
               agora - ultimaRepeticaoAjusteLDR >=
                   INTERVALO_REPETICAO_LDR_MS) {
      ultimaRepeticaoAjusteLDR = agora;
      const int variacao =
          repeticaoAjusteLDR == RepeticaoAjusteLDR::Aumentar ? 10 : -10;
      const bool alterouLimiar =
          tela == TELA_LDR_AJUSTE
              ? ajustarLimiarLDR(variacao)
              : ajustarLimiarCentroLDR(variacao);
      if (alterouLimiar) {
        alteracaoRepetidaLDRPendente = true;
        if (tela == TELA_LDR_AJUSTE) {
          mostrarLDRAjuste();
        } else {
          mostrarLDRAjusteCentro();
        }
      }
    }
  } else {
    if (alteracaoRepetidaLDRPendente) {
      salvarLimiaresLDR();
    }
    repeticaoAjusteLDR = RepeticaoAjusteLDR::Nenhuma;
    alteracaoRepetidaLDRPendente = false;
  }

  static unsigned long ultimaAtualizacao = 0;
  // Telemetria e bateria sao redesenhadas periodicamente sem bloquear a leitura
  // de botoes nem depender de uma interrupcao de tela.
  if (agora - ultimaAtualizacao >= 180) {
    ultimaAtualizacao = agora;
    if (tela == TELA_BNO_LEITURA) mostrarBNO();
    else if (tela == TELA_ULTRA) mostrarUltra();
    else if (tela == TELA_TESTE_KICKER_AUTO) mostrarTesteKickerAutomatico();
    else if (tela == TELA_TESTE_KICKER_FIM_CURSO) mostrarTesteKickerFimDeCurso();
    else if (tela == TELA_INFRA) mostrarInfra();
    else if (tela == TELA_LDR) mostrarLDR();
    else if (tela == TELA_LDR_CENTRO) mostrarLDRCentro();
    else if (tela == TELA_LDR_AJUSTE) mostrarLDRAjuste();
    else if (tela == TELA_LDR_CENTRO_AJUSTE) mostrarLDRAjusteCentro();
    else if (tela == TELA_BATERIA) mostrarBateria();
    else if (tela == TELA_TESTE_BNO) mostrarTesteBNOMove();
    else if (tela == TELA_TESTE_TSSP) mostrarTesteTSSP();
    else if (tela == TELA_INFO_TESTE) mostrarInfoTesteChecklist();
    else if (tela == TELA_MAC) mostrarMac();
  }

  // RoboDefOn acompanha a tela: so e true na defesa; ao voltar dela vira false.
  ESPNowLink::definirEstadoLocal(
      tela == TELA_ATAQUE ? ESPNowLink::Papel::Ataque
      : tela == TELA_DEFESA ? ESPNowLink::Papel::Defesa
                            : ESPNowLink::Papel::Nenhum,
      tela == TELA_DEFESA);

  estadoAnteriorSelecionar = selecionar;
  estadoAnteriorMexer = mexer;
  estadoAnteriorVoltar = voltar;
}
