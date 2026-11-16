#include <Wire.h>
#include <U8g2lib.h>
#include <BatteryEstimator.h>
#include <MotoresTeste.h>
#include <ConfigPinos.h>
#include <IRProtocol.h>
#include <LDRFixed.h>
#include <Preferences.h>
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
  TELA_ULTRA,
  TELA_INFRA,
  TELA_LDR,
  TELA_LDR_AJUSTE,

  TELA_TESTE_BNO,
  TELA_TESTE_ULTRA,
  TELA_TESTE_TSSP,
  TELA_INFO_TESTE
};

Tela tela = TELA_MENU;
BatteryEstimator estimadorBateria;
Motores motoresChecklist;
Omni omniChecklist;
uint32_t ultimaAtualizacaoBateriaMs = 0;
uint32_t ultimoLogBateriaMs = 0;

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
const int totalTelemetria = 5;
const uint8_t totalRegioesLDR = 3;
const char *const NOMES_LDR[totalRegioesLDR] = {"ESQ", "DIR", "INF"};
uint16_t limiaresLDR[totalRegioesLDR] = {
    ConfigLDR::Ataque.esquerda, ConfigLDR::Ataque.direita,
    ConfigLDR::Ataque.inferior};
int opcaoLDR = 0;
bool ajustandoLDR = false;
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
};

constexpr uint16_t VERSAO_CONFIG_LDR = 1;
constexpr uint16_t LIMIAR_LDR_MAXIMO = 4095;

// 0 = BNO
// 1 = KICKER
// 2 = ULTRA
// 3 = INFRA
// 4 = LDR

// =====================================================
// MENU BNO
// =====================================================

int opcaoBNO = 0;
const int totalBNO = 3;

// 0 = LEITURA
// 1 = ZERO
// 2 = RESET SENSOR

constexpr float KP_BNO_MOVE_INICIAL = 0.0044f;
constexpr float KI_BNO_MOVE_INICIAL = 0.0000022f;
constexpr float KD_BNO_MOVE_INICIAL = 0.0001f;
constexpr float PASSO_KP_BNO_MOVE = 0.0002f;
constexpr float PASSO_KI_BNO_MOVE = 0.0000005f;
constexpr float PASSO_KD_BNO_MOVE = 0.0001f;
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

float kpBnoMove = KP_BNO_MOVE_INICIAL;
float kiBnoMove = KI_BNO_MOVE_INICIAL;
float kdBnoMove = KD_BNO_MOVE_INICIAL;
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
const int totalChecklist = 7;
const int itensChecklistVisiveis = 4;

// BNO, IR, ultrassonicos, motores, solenoides, UART e LDR.
const char *const NOMES_CHECKLIST[totalChecklist] = {
    "BNO", "IR / TSSP", "ULTRASSONICOS", "MOTORES",
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
bool telemetriaRegiaoBrancaLDR[4] = {};

// Recebe um snapshot do Body para a IHM nao acoplar a renderizacao aos drivers
// UART/sensores. Flags de validade distinguem "zero medido" de "sem pacote".
void IHM_atualizarTelemetria(float headingDegrees, float gyroZDegreesPerSecond,
                             bool bnoValido, const float distanciasUltra[4],
                             uint8_t mascaraUltra, bool ultraValido,
                             const uint8_t atividadesIR[IR_SENSOR_COUNT],
                             float direcaoIR, bool bolaDetectada, bool irValido) {
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
}

// Copia estados LDR para exibi-los sem reamostrar hardware dentro da IHM.
void IHM_atualizarLDR(const bool regioesBrancas[4]) {
  for (uint8_t i = 0; i < 4; ++i) {
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

void IHM_obterGanhosTesteBNOMove(float &kp, float &ki, float &kd) {
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
    "ULTRA",
    "INFRA",
    "LDR"
  };

  for (
    int i = 0;
    i < totalTelemetria;
    i++
  ) {

    int y =
      23 + i * 9;

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
  const bool leu =
      preferencias.getBytesLength("pid") == sizeof(ganhos) &&
      preferencias.getBytes("pid", &ganhos, sizeof(ganhos)) == sizeof(ganhos);
  preferencias.end();
  if (!leu) {
    Serial.println("NVS sem ganhos BNO-Move; usando valores padrao.");
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
  switch (opcaoBnoMove) {
    case 1:
      kpBnoMove = constrain(kpBnoMove + direcao * PASSO_KP_BNO_MOVE,
                            0.0f, MAX_KP_BNO_MOVE);
      break;
    case 2:
      kiBnoMove = constrain(kiBnoMove + direcao * PASSO_KI_BNO_MOVE,
                            0.0f, MAX_KI_BNO_MOVE);
      break;
    case 3:
      kdBnoMove = constrain(kdBnoMove + direcao * PASSO_KD_BNO_MOVE,
                            0.0f, MAX_KD_BNO_MOVE);
      break;
    default:
      return;
  }
  salvarGanhosBnoMove();
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
  snprintf(linha, sizeof(linha), "%cKd=%0.4f",
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
// HC-SR04 / ULTRA
// =====================================================

// Seleciona o menor eco valido para o checklist; mascara e frescor evitam usar
// ecos ausentes ou snapshots vencidos como distancia de obstaculo.
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

// Exibe cada canal separadamente para diferenciar ausencia de eco de distancia.
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

// Apresenta diagnostico de ultra e distancia minima sem acionar movimento.
void testarUltraNaTela() {

  tela = TELA_TESTE_ULTRA;

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.drawStr(
    30,
    10,
    "TESTES"
  );

  oled.drawStr(
    47,
    25,
    "ULTRA"
  );

  oled.setFont(
    u8g2_font_5x8_tr
  );

  oled.drawStr(
    25,
    43,
    "TESTANDO..."
  );

  oled.sendBuffer();

  delay(400);

  float distancia = 0.0f;
  bool resultado =
    lerDistanciaUltra(distancia);

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.drawStr(
    30,
    9,
    "TESTES"
  );

  oled.drawStr(
    49,
    20,
    "ULTRA"
  );

  oled.setFont(
    u8g2_font_5x8_tr
  );

  if (resultado) {

    oled.drawStr(
      5,
      33,
      "UART: OK"
    );

    oled.drawStr(
      5,
      43,
      "SENSOR: OK"
    );

    oled.drawStr(
      5,
      53,
      "PIO: ultrassonicos-test"
    );

    oled.setFont(
      u8g2_font_ncenB10_tr
    );

    oled.drawStr(
      82,
      44,
      "OK!"
    );

  } else {

    oled.drawStr(
      5,
      33,
      "SEM DADOS UART"
    );

    oled.drawStr(
      5,
      43,
      "AGUARDANDO LEITURA"
    );

    oled.drawStr(5, 53, "PIO: ultrassonicos-test");
  }

  if (resultado) {

    oled.setFont(
      u8g2_font_5x8_tr
    );

    oled.setCursor(
      5,
      63
    );

    oled.print(
      distancia,
      1
    );

    oled.print(
      " cm"
    );
  }

  oled.sendBuffer();
}

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
    case 3:
      codigoPrincipal = "motores-test";
      codigoSecundario = "motor-unitario-test";
      break;
    case 4:
      codigoPrincipal = "SEM ENV PIO";
      codigoSecundario = "TEST_SOLENOIDES/*.ino";
      break;
    case 5:
      codigoPrincipal = "uart-receptor-test";
      codigoSecundario = "uart-emissor-test";
      status = (telemetriaBNOValida || telemetriaUltraValida ||
                telemetriaIRValida)
                   ? "UART: DADOS OK"
                   : "UART: SEM DADOS";
      break;
    case 6:
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
      limiaresLDR[0], limiaresLDR[1], limiaresLDR[2]);
  ldrThresholdTarget->setCenterSensorThresholds(
      padrao.centro, padrao.histereseCentro);
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
      VERSAO_CONFIG_LDR, limiaresLDR[0], limiaresLDR[1], limiaresLDR[2]};
  if (preferenciasLDR.putBytes("thresholds", &config, sizeof(config)) !=
      sizeof(config)) {
    Serial.println("ERRO: falha ao salvar limiares LDR no NVS.");
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
  if (preferenciasLDR.getBytesLength("thresholds") == sizeof(config) &&
      preferenciasLDR.getBytes("thresholds", &config, sizeof(config)) ==
          sizeof(config) &&
      config.versao == VERSAO_CONFIG_LDR &&
      config.esquerda <= LIMIAR_LDR_MAXIMO &&
      config.direita <= LIMIAR_LDR_MAXIMO &&
      config.inferior <= LIMIAR_LDR_MAXIMO) {
    limiaresLDR[0] = config.esquerda;
    limiaresLDR[1] = config.direita;
    limiaresLDR[2] = config.inferior;
  } else {
    Serial.printf("NVS sem limiares LDR validos; usando padroes %u/%u/%u.\n",
                  limiaresLDR[0], limiaresLDR[1], limiaresLDR[2]);
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
      LDRFixed::Esquerda, LDRFixed::Direita, LDRFixed::Inferior};
  const char *const nomes[] = {"ESQ", "DIR", "INF"};
  for (uint8_t i = 0; i < 3; ++i) {
    const uint8_t y = 32 + i * 10;
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

  oled.drawStr(4, 63, "VOLTAR = TELEMETRIA");
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

  const int primeiroItem = opcaoLDR > 3 ? opcaoLDR - 3 : 0;
  for (int linha = 0; linha < 4; ++linha) {
    const int item = primeiroItem + linha;
    const uint8_t y = 22 + linha * 9;
    if (item == opcaoLDR) {
      oled.drawStr(4, y, ">");
      if (ajustandoLDR) {
        oled.drawStr(58, y, "*");
      }
    }
    if (item < 2) {
      oled.drawStr(16, y, item == 0 ? "LIMIAR 1" : "LIMIAR 2");
    } else {
      oled.drawStr(16, y, NOMES_LDR[item - 2]);
      oled.setCursor(65, y);
      oled.print(limiaresLDR[item - 2]);
    }
  }

  oled.drawStr(4, 62, (erroLimiaresLDR || erroPerfilLDR) ? "ERRO NO NVS" :
               ajustandoLDR ? "SEL +5  MEXER -5" : "MEXER=PROX SEL=ESCOLHER");
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
    oled.setCursor(20, 38);
    oled.print("ANGULO: ");
    oled.print(telemetriaDirecaoIR, 1);
    oled.print(" graus");
  }
  oled.drawStr(4, 63, "VOLTAR = TELEMETRIA");
  oled.sendBuffer();
}

// Apresenta SOC e leituras brutas/filtradas; valores nao finitos ou implausiveis
// sao substituidos por mensagem de falha, nao por um percentual fabricado.
void mostrarBateria() {
  const float voltage = estimadorBateria.getFilteredVoltage();
  const float percentage = estimadorBateria.getPercentCombined();
  const float rawVoltage = estimadorBateria.getRawVoltage();
  const float openCircuitVoltage = estimadorBateria.getOpenCircuitVoltage();
  const float current = estimadorBateria.getCurrentA();
  const float voltageSoc = estimadorBateria.getPercentVoltageBased();
  const float coulombSoc = estimadorBateria.getPercentCoulomb();
  const float consumedMah = estimadorBateria.getConsumedMah();
  const bool readingValid = isfinite(voltage) && voltage > 0.1f &&
                            isfinite(percentage) &&
                            percentage >= 0.0f && percentage <= 100.0f;

  oled.clearBuffer();
  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(4, 9, "BATERIA");
  oled.drawStr(94, 9, "VOLTAR");
  oled.drawHLine(4, 12, 120);

  if (readingValid) {
    oled.setCursor(4, 21);
    oled.print("SOC ");
    oled.print(percentage, 1);
    oled.print("% V ");
    oled.print(voltage, 2);
    oled.print("V");

    oled.setCursor(4, 29);
    oled.print("Raw ");
    oled.print(rawVoltage, 2);
    oled.print(" Filt ");
    oled.print(voltage, 2);
    oled.print("V");

    oled.setCursor(4, 37);
    oled.print("Voc ");
    oled.print(openCircuitVoltage, 2);
    oled.print("V I ");
    oled.print(current, 2);
    oled.print("A");

    oled.setCursor(4, 45);
    oled.print("SOCv ");
    oled.print(voltageSoc, 0);
    oled.print("% SOCq ");
    oled.print(coulombSoc, 0);
    oled.print("%");

    oled.setCursor(4, 53);
    oled.print("Consumido ");
    oled.print(consumedMah, 1);
    oled.print("mAh");

    oled.drawFrame(4, 56, 120, 8);
    const uint8_t barWidth = static_cast<uint8_t>(percentage * 1.16f);
    if (barWidth > 0) oled.drawBox(6, 58, barWidth, 4);
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
  ultimoLogBateriaMs = ultimaAtualizacaoBateriaMs;
  Serial.println();
  Serial.println("=== Estimador de bateria iniciado ===");
  Serial.printf("Tensao inicial: %.3f V | SOC inicial: %.1f %%\n",
                estimadorBateria.getFilteredVoltage(),
                estimadorBateria.getPercentCombined());
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
  if (agora - ultimoLogBateriaMs >= LOG_PERIOD_MS) {
    ultimoLogBateriaMs = agora;

    Serial.printf(
        "Vraw=%.3fV  Vfilt=%.3fV  Voc=%.3fV  I=%.3fA  "
        "SOCv=%.1f%%  SOCq=%.1f%%  SOCcomb=%.1f%%  consumido=%.1fmAh\n",
        estimadorBateria.getRawVoltage(),
        estimadorBateria.getFilteredVoltage(),
        estimadorBateria.getOpenCircuitVoltage(),
        estimadorBateria.getCurrentA(),
        estimadorBateria.getPercentVoltageBased(),
        estimadorBateria.getPercentCoulomb(),
        estimadorBateria.getPercentCombined(),
        estimadorBateria.getConsumedMah());
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
        const int regiao = opcaoLDR - 2;
        const int novoValor = static_cast<int>(limiaresLDR[regiao]) - 5;
        limiaresLDR[regiao] = novoValor > 0 ? static_cast<uint16_t>(novoValor) : 0;
        aplicarLimiaresLDRNoFirmware();
        salvarLimiaresLDR();
      } else {
        opcaoLDR = (opcaoLDR + 1) % (totalRegioesLDR + 2);
      }
      mostrarLDRAjuste();
    } else if (tela == TELA_INFO_TESTE) {
      mostrarInfoTesteChecklist();
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
            mostrarEmBreve("KICKER");
            break;
          case 2:
            tela = TELA_ULTRA;
            mostrarUltra();
            break;
          case 3:
            tela = TELA_INFRA;
            mostrarInfra();
            break;
          default:
            tela = TELA_LDR;
            mostrarLDR();
            break;
        }
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
      case TELA_LDR_AJUSTE:
        if (ajustandoLDR) {
          const int regiao = opcaoLDR - 2;
          const int novoValor = static_cast<int>(limiaresLDR[regiao]) + 5;
          if (novoValor <= LIMIAR_LDR_MAXIMO) {
            limiaresLDR[regiao] = static_cast<uint16_t>(novoValor);
            aplicarLimiaresLDRNoFirmware();
            salvarLimiaresLDR();
          }
        } else if (opcaoLDR < 2) {
          selecionarPerfilLDR(static_cast<uint8_t>(opcaoLDR));
        } else {
          ajustandoLDR = true;
        }
        mostrarLDRAjuste();
        break;
      case TELA_CHECKLIST:
        if (opcaoChecklist == 0) {
          abrirTesteBNOMove();
        } else if (opcaoChecklist == 1) {
          tela = TELA_TESTE_TSSP;
          mostrarTesteTSSP();
        } else if (opcaoChecklist == 2) {
          testarUltraNaTela();
        } else if (opcaoChecklist == 3) {
          testarMotoresNaTela();
        } else if (opcaoChecklist == 6) {
          tela = TELA_LDR_AJUSTE;
          opcaoLDR = 0;
          ajustandoLDR = false;
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
      case TELA_TESTE_ULTRA:
        testarUltraNaTela();
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
    } else if (tela == TELA_ULTRA || tela == TELA_INFRA || tela == TELA_LDR) {
      ajustandoLDR = false;
      tela = TELA_TELEMETRIA;
      mostrarMenuTelemetria();
    } else if (tela == TELA_LDR_AJUSTE) {
      ajustandoLDR = false;
      tela = TELA_CHECKLIST;
      mostrarMenuChecklist();
    } else if (tela == TELA_TESTE_ULTRA) {
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

  static unsigned long ultimaAtualizacao = 0;
  // Telemetria e bateria sao redesenhadas periodicamente sem bloquear a leitura
  // de botoes nem depender de uma interrupcao de tela.
  if (agora - ultimaAtualizacao >= 180) {
    ultimaAtualizacao = agora;
    if (tela == TELA_BNO_LEITURA) mostrarBNO();
    else if (tela == TELA_ULTRA) mostrarUltra();
    else if (tela == TELA_INFRA) mostrarInfra();
    else if (tela == TELA_LDR) mostrarLDR();
    else if (tela == TELA_LDR_AJUSTE) mostrarLDRAjuste();
    else if (tela == TELA_BATERIA) mostrarBateria();
    else if (tela == TELA_TESTE_BNO) mostrarTesteBNOMove();
    else if (tela == TELA_TESTE_TSSP) mostrarTesteTSSP();
    else if (tela == TELA_INFO_TESTE) mostrarInfoTesteChecklist();
  }

  estadoAnteriorSelecionar = selecionar;
  estadoAnteriorMexer = mexer;
  estadoAnteriorVoltar = voltar;
}
