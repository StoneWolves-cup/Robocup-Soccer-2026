#include <Wire.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <BatteryEstimator.h>
#include <MotoresTeste.h>
#include <ConfigPinos.h>
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

  TELA_TESTE_BNO,
  TELA_TESTE_ULTRA,
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

// 0 = ATAQUE
// 1 = DEFESA

// =====================================================
// MENU TELEMETRIA
// =====================================================

int opcaoTelemetria = 0;
const int totalTelemetria = 5;

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
float telemetriaDirecaoIR = 0.0f;
bool telemetriaBNOValida = false;
bool telemetriaUltraValida = false;
bool telemetriaIRValida = false;
bool telemetriaBolaDetectada = false;
bool telemetriaRegiaoBrancaLDR[4] = {};

void IHM_atualizarTelemetria(float headingDegrees, float gyroZDegreesPerSecond,
                             bool bnoValido, const float distanciasUltra[4],
                             uint8_t mascaraUltra, bool ultraValido,
                             float direcaoIR, bool bolaDetectada, bool irValido) {
  telemetriaHeading = headingDegrees;
  telemetriaGyroZ = gyroZDegreesPerSecond;
  telemetriaBNOValida = bnoValido;
  for (uint8_t i = 0; i < 4; ++i) telemetriaUltra[i] = distanciasUltra[i];
  telemetriaUltraMascara = mascaraUltra;
  telemetriaUltraValida = ultraValido;
  telemetriaDirecaoIR = direcaoIR;
  telemetriaBolaDetectada = bolaDetectada;
  telemetriaIRValida = irValido;
}

void IHM_atualizarLDR(const bool regioesBrancas[4]) {
  for (uint8_t i = 0; i < 4; ++i) {
    telemetriaRegiaoBrancaLDR[i] = regioesBrancas[i];
  }
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

  delay(150);
}

// =====================================================
// MENU PRINCIPAL
// =====================================================

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
    "CHECKLIST"
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

void mostrarMenuChecklist() {

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.drawStr(
    30,
    10,
    "CHECKLIST"
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

void mostrarBNO() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(47, 9, "BNO REMOTO");
  oled.setFont(u8g2_font_5x8_tr);
  if (telemetriaBNOValida) {
    float heading = telemetriaHeading;
    if (referenciaDefinida) heading = normalizar360(heading - referenciaGraus);
    oled.drawStr(3, 24, "RUMO:");
    oled.setFont(u8g2_font_6x10_tr);
    oled.setCursor(43, 24);
    oled.print(heading, 1);
    oled.setFont(u8g2_font_5x8_tr);
    oled.drawStr(3, 40, "GIRO Z:");
    oled.setFont(u8g2_font_6x10_tr);
    oled.setCursor(51, 40);
    oled.print(telemetriaGyroZ, 1);
    oled.setFont(u8g2_font_5x8_tr);
    oled.drawStr(90, 40, "graus/s");
  } else {
    oled.drawStr(15, 32, "SEM DADOS UART");
  }
  oled.drawStr(4, 62, "SEL=ZERO  VOLTAR");
  oled.sendBuffer();
}

// =====================================================
// ZERO BNO
// =====================================================

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

bool verificarBNO() {
  return telemetriaBNOValida;
}

// =====================================================
// TESTE DO BNO
// =====================================================

bool testarBNO() {
  return telemetriaBNOValida;
}

// =====================================================
// TESTE BNO NA TELA
// =====================================================

void testarBNONaTela() {

  tela =
    TELA_TESTE_BNO;

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.drawStr(
    30,
    10,
    "CHECKLIST"
  );

  oled.drawStr(
    53,
    25,
    "BNO"
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

  delay(500);

  bool resultado =
    testarBNO();

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.drawStr(
    30,
    9,
    "CHECKLIST"
  );

  oled.drawStr(
    53,
    20,
    "BNO"
  );

  if (resultado) {

    oled.setFont(
      u8g2_font_5x8_tr
    );

    oled.drawStr(
      5,
      32,
      "UART: OK"
    );

    oled.drawStr(
      5,
      42,
      "PACOTE: OK"
    );

    oled.drawStr(
      5,
      52,
      "PIO: bno-test"
    );

    oled.setFont(
      u8g2_font_ncenB10_tr
    );

    oled.drawStr(
      76,
      44,
      "OK!"
    );

  } else {

    oled.setFont(
      u8g2_font_5x8_tr
    );

    oled.drawStr(
      5,
      32,
      "SEM DADOS DO HEAD"
    );

    oled.drawStr(5, 43, "PIO: bno-test");
  }

  oled.setFont(
    u8g2_font_5x8_tr
  );

  oled.drawStr(
    17,
    62,
    "VOLTAR"
  );

  oled.sendBuffer();
}

// =====================================================
// HC-SR04 / ULTRA
// =====================================================

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

bool testarUltra() {
  float distancia = 0.0f;
  return lerDistanciaUltra(distancia);
}

void testarUltraNaTela() {

  tela = TELA_TESTE_ULTRA;

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.drawStr(
    30,
    10,
    "CHECKLIST"
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
    "CHECKLIST"
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
  oled.drawStr(30, 9, "CHECKLIST");
  oled.drawHLine(8, 13, 112);
  oled.drawStr(4, 25, NOMES_CHECKLIST[opcaoChecklist]);

  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(4, 36, status);
  oled.drawStr(4, 47, codigoPrincipal);
  if (codigoSecundario[0] != '\0') {
    oled.drawStr(4, 56, codigoSecundario);
  }
  oled.drawStr(4, 63, "VOLTAR = CHECKLIST");
  oled.sendBuffer();
}

void testarMotoresNaTela() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(30, 9, "CHECKLIST");
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

// =====================================================
// MOSTRAR RESULTADO DO RESET
// =====================================================

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

void mostrarLDR() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(54, 9, "LDR");
  oled.drawHLine(8, 13, 112);
  oled.setFont(u8g2_font_5x8_tr);

  const char *nomes[4] = {"CENTRO", "ESQ", "DIR", "INF"};
  for (uint8_t i = 0; i < 4; ++i) {
    const uint8_t y = 24 + i * 10;
    oled.drawStr(6, y, nomes[i]);
    oled.setCursor(62, y);
    oled.print(telemetriaRegiaoBrancaLDR[i] ? "BRANCO" : "VERDE");
  }

  oled.drawStr(4, 63, "VOLTAR = TELEMETRIA");
  oled.sendBuffer();
}

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

void mostrarBateria() {
  const float voltage = estimadorBateria.getFilteredVoltage();
  const float percentage = estimadorBateria.getPercentCombined();
  const bool readingValid = isfinite(voltage) && voltage > 0.1f &&
                            isfinite(percentage) &&
                            percentage >= 0.0f && percentage <= 100.0f;

  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(38, 9, "BATERIA");
  oled.drawHLine(8, 13, 112);

  oled.setFont(u8g2_font_5x8_tr);
  if (readingValid) {
    oled.setCursor(12, 27);
    oled.print("CARGA: ");
    oled.print(percentage, 1);
    oled.print("%");

    oled.setCursor(12, 38);
    oled.print("TENSAO: ");
    oled.print(voltage, 2);
    oled.print(" V");

    oled.drawFrame(8, 44, 112, 10);
    const uint8_t barWidth = static_cast<uint8_t>(percentage * 1.08f);
    if (barWidth > 0) oled.drawBox(10, 46, barWidth, 6);
  } else {
    oled.drawStr(16, 29, "LEITURA INVALIDA");
    oled.drawStr(12, 41, "VERIFIQUE BATERIA");
  }

  oled.drawStr(4, 63, "VOLTAR = MENU");
  oled.sendBuffer();
}

void IHM_begin() {
  pinMode(BOTAO_SELECIONAR, INPUT_PULLDOWN);
  pinMode(BOTAO_MEXER, INPUT_PULLDOWN);
  pinMode(BOTAO_VOLTAR, INPUT_PULLDOWN);
  estimadorBateria.begin();
  ultimaAtualizacaoBateriaMs = millis();
  Wire.begin(SDA_PIN, SCK_PIN);
  oled.setI2CAddress(0x3C << 1);
  oled.begin();
  telaCarregamento();
  mostrarMenu();
}

void IHM_update() {
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
    } else if (tela == TELA_INFO_TESTE) {
      mostrarInfoTesteChecklist();
    }
  }

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
        mostrarEmBreve(opcaoEstrategia == 0 ? "ATAQUE" : "DEFESA");
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
          mostrarResultadoReset(verificarBNO());
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
      case TELA_CHECKLIST:
        if (opcaoChecklist == 0) {
          testarBNONaTela();
        } else if (opcaoChecklist == 2) {
          testarUltraNaTela();
        } else if (opcaoChecklist == 3) {
          testarMotoresNaTela();
        } else {
          tela = TELA_INFO_TESTE;
          mostrarInfoTesteChecklist();
        }
        break;
      case TELA_TESTE_BNO:
        testarBNONaTela();
        break;
      case TELA_TESTE_ULTRA:
        testarUltraNaTela();
        break;
      default:
        break;
    }
  }

  if (clicouVoltar) {
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
    } else if (tela == TELA_ULTRA || tela == TELA_INFRA || tela == TELA_LDR) {
      tela = TELA_TELEMETRIA;
      mostrarMenuTelemetria();
    } else if (tela == TELA_TESTE_BNO || tela == TELA_TESTE_ULTRA) {
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
  if (agora - ultimaAtualizacao >= 180) {
    ultimaAtualizacao = agora;
    if (tela == TELA_BNO_LEITURA) mostrarBNO();
    else if (tela == TELA_ULTRA) mostrarUltra();
    else if (tela == TELA_INFRA) mostrarInfra();
    else if (tela == TELA_LDR) mostrarLDR();
    else if (tela == TELA_BATERIA) mostrarBateria();
    else if (tela == TELA_INFO_TESTE) mostrarInfoTesteChecklist();
  }

  estadoAnteriorSelecionar = selecionar;
  estadoAnteriorMexer = mexer;
  estadoAnteriorVoltar = voltar;
}
