#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// =====================================================
// I2C
// =====================================================

#define SDA_PIN 21
#define SCL_PIN 22

// =====================================================
// BNO055 RESET
// =====================================================

#define BNO_RST 25

// =====================================================
// BOTOES
// =====================================================

#define BOTAO_SELECIONAR 18
#define BOTAO_MEXER      19
#define BOTAO_VOLTAR     23

// =====================================================
// HC-SR04 / ULTRA
// =====================================================

#define ULTRA_TRIG 27
#define ULTRA_ECHO 26

// =====================================================
// OLED
// =====================================================

U8G2_SSD1315_128X64_NONAME_F_HW_I2C oled(
  U8G2_R0,
  U8X8_PIN_NONE,
  SCL_PIN,
  SDA_PIN
);

// =====================================================
// BNO055
// =====================================================

Adafruit_BNO055 bno(55, 0x28, &Wire);

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
  TELA_LDR,

  TELA_TESTE_BNO,
  TELA_TESTE_ULTRA
};

Tela tela = TELA_MENU;

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
const int totalChecklist = 2;

// 0 = BNO
// 1 = ULTRA

// =====================================================
// ESTADOS DOS BOTOES
// =====================================================

bool estadoAnteriorSelecionar = HIGH;
bool estadoAnteriorMexer = HIGH;
bool estadoAnteriorVoltar = HIGH;

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

bool leituraBrutaSelecionar = HIGH;
bool leituraBrutaMexer = HIGH;
bool leituraBrutaVoltar = HIGH;

bool estadoEstavelSelecionar = HIGH;
bool estadoEstavelMexer = HIGH;
bool estadoEstavelVoltar = HIGH;

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
    progresso += 2
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

    delay(120);
  }

  delay(500);
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
    "RESET SENSOR"
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

  const char* nomes[] = {
    "BNO",
    "ULTRA"
  };

  for (int i = 0; i < totalChecklist; i++) {

    int y = 31 + i * 13;

    if (i == opcaoChecklist) {

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

  oled.setFont(
    u8g2_font_5x8_tr
  );

  oled.drawStr(
    4,
    62,
    "MEXER=TROCA  SEL=TESTE"
  );

  oled.sendBuffer();
}

// =====================================================
// MOSTRAR BNO
// =====================================================

void mostrarBNO() {

  imu::Vector<3> euler =
    bno.getVector(
      Adafruit_BNO055::VECTOR_EULER
    );

  float graus =
    euler.x();

  float frenteTras =
    euler.z();

  float esquerdaDireita =
    euler.y();

  if (referenciaDefinida) {

    graus =
      normalizar360(
        graus - referenciaGraus
      );

    frenteTras =
      normalizar360(
        frenteTras -
        referenciaFrenteTras
      );

    esquerdaDireita =
      normalizar360(
        esquerdaDireita -
        referenciaEsqDir
      );
  }

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.drawStr(
    47,
    9,
    "BNO"
  );

  // GRAUS

  oled.setFont(
    u8g2_font_5x8_tr
  );

  oled.drawStr(
    2,
    21,
    "GRAUS:"
  );

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.setCursor(
    42,
    21
  );

  oled.print(
    graus,
    1
  );

  // FRENTE/TRAS

  oled.setFont(
    u8g2_font_5x8_tr
  );

  oled.drawStr(
    2,
    36,
    "FRENTE/TRAS:"
  );

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.setCursor(
    76,
    36
  );

  oled.print(
    frenteTras,
    1
  );

  // ESQ/DIR

  oled.setFont(
    u8g2_font_5x8_tr
  );

  oled.drawStr(
    2,
    50,
    "ESQ/DIR:"
  );

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.setCursor(
    52,
    50
  );

  oled.print(
    esquerdaDireita,
    1
  );

  oled.setFont(
    u8g2_font_5x8_tr
  );

  oled.drawStr(
    5,
    62,
    "SEL=ZERO  VOLTAR"
  );

  oled.sendBuffer();
}

// =====================================================
// ZERO BNO
// =====================================================

void zerarBNO() {

  imu::Vector<3> euler =
    bno.getVector(
      Adafruit_BNO055::VECTOR_EULER
    );

  referenciaGraus =
    euler.x();

  referenciaFrenteTras =
    euler.z();

  referenciaEsqDir =
    euler.y();

  referenciaDefinida =
    true;

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.drawStr(
    42,
    24,
    "BNO ZERO"
  );

  oled.setFont(
    u8g2_font_5x8_tr
  );

  oled.drawStr(
    19,
    39,
    "POSICAO INICIAL"
  );

  oled.drawStr(
    38,
    51,
    "SALVA!"
  );

  oled.sendBuffer();

  delay(900);

  tela =
    TELA_BNO_LEITURA;

  mostrarBNO();
}

// =====================================================
// RESET FISICO DO BNO
// =====================================================

bool resetarBNOSensor() {

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.drawStr(
    30,
    24,
    "RESET BNO"
  );

  oled.setFont(
    u8g2_font_5x8_tr
  );

  oled.drawStr(
    26,
    40,
    "REINICIANDO..."
  );

  oled.sendBuffer();

  // RST LOW
  digitalWrite(
    BNO_RST,
    LOW
  );

  delay(50);

  // RST HIGH
  digitalWrite(
    BNO_RST,
    HIGH
  );

  // Espera o BNO reiniciar
  delay(700);

  // Inicializa novamente
  if (!bno.begin()) {

    return false;
  }

  bno.setExtCrystalUse(true);

  delay(500);

  return true;
}

// =====================================================
// TESTE DO BNO
// =====================================================

bool testarBNO() {

  // ---------------------------------------------------
  // I2C
  // ---------------------------------------------------

  Wire.beginTransmission(0x28);

  if (
    Wire.endTransmission() != 0
  ) {

    return false;
  }

  // ---------------------------------------------------
  // CHIP ID
  // ---------------------------------------------------

  Wire.beginTransmission(0x28);

  Wire.write(0x00);

  if (
    Wire.endTransmission(false) != 0
  ) {

    return false;
  }

  if (
    Wire.requestFrom(0x28, 1) != 1
  ) {

    return false;
  }

  uint8_t chipID =
    Wire.read();

  if (
    chipID != 0xA0
  ) {

    return false;
  }

  // ---------------------------------------------------
  // DADOS
  // ---------------------------------------------------

  for (
    int i = 0;
    i < 5;
    i++
  ) {

    imu::Vector<3> euler =
      bno.getVector(
        Adafruit_BNO055::VECTOR_EULER
      );

    if (
      isnan(euler.x()) ||
      isnan(euler.y()) ||
      isnan(euler.z())
    ) {

      return false;
    }

    delay(50);
  }

  return true;
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
      "I2C: OK"
    );

    oled.drawStr(
      5,
      42,
      "CHIP: OK"
    );

    oled.drawStr(
      5,
      52,
      "DADOS: OK"
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
      "I2C/CHIP/DADOS:"
    );

    oled.setFont(
      u8g2_font_ncenB10_tr
    );

    oled.drawStr(
      48,
      49,
      "ERRO!"
    );
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

  digitalWrite(
    ULTRA_TRIG,
    LOW
  );

  delayMicroseconds(3);

  digitalWrite(
    ULTRA_TRIG,
    HIGH
  );

  delayMicroseconds(10);

  digitalWrite(
    ULTRA_TRIG,
    LOW
  );

  unsigned long duracao = pulseIn(
    ULTRA_ECHO,
    HIGH,
    25000
  );

  if (duracao == 0) {
    return false;
  }

  distanciaCm =
    (duracao * 0.0343f) / 2.0f;

  // Evita mostrar leituras absurdas.
  if (distanciaCm < 2.0f || distanciaCm > 430.0f) {
    return false;
  }

  return true;
}

void mostrarUltra() {

  float distancia = 0.0f;

  bool leituraOK =
    lerDistanciaUltra(distancia);

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.drawStr(
    45,
    9,
    "ULTRA"
  );

  oled.drawHLine(
    8,
    13,
    112
  );

  oled.setFont(
    u8g2_font_5x8_tr
  );

  oled.drawStr(
    4,
    25,
    "DISTANCIA:"
  );

  if (leituraOK) {

    oled.setFont(
      u8g2_font_9x15_tr
    );

    oled.setCursor(
      67,
      27
    );

    oled.print(
      distancia,
      1
    );

    oled.setFont(
      u8g2_font_5x8_tr
    );

    oled.drawStr(
      107,
      27,
      "cm"
    );

    // Barra visual: 2 cm = cheia / 100 cm = vazia.
    // Serve apenas como indicador visual de proximidade.
    int barra = map(
      constrain((int)distancia, 2, 100),
      100,
      2,
      0,
      112
    );

    oled.drawFrame(
      8,
      34,
      112,
      9
    );

    if (barra > 0) {
      oled.drawBox(
        10,
        36,
        barra,
        5
      );
    }

    oled.setFont(
      u8g2_font_5x8_tr
    );

    oled.drawStr(
      4,
      52,
      "STATUS: OBJETO DETECTADO"
    );

  } else {

    oled.setFont(
      u8g2_font_6x10_tr
    );

    oled.drawStr(
      56,
      27,
      "---"
    );

    oled.setFont(
      u8g2_font_5x8_tr
    );

    oled.drawStr(
      4,
      41,
      "SEM ECO"
    );

    oled.drawStr(
      4,
      52,
      "AGUARDANDO OBJETO"
    );
  }

  oled.setFont(
    u8g2_font_5x8_tr
  );

  oled.drawStr(
    4,
    62,
    "VOLTAR = TELEMETRIA"
  );

  oled.sendBuffer();
}

bool testarUltra() {

  float distancia = 0.0f;

  return lerDistanciaUltra(
    distancia
  );
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
      "TRIG: OK"
    );

    oled.drawStr(
      5,
      43,
      "ECHO: OK"
    );

    oled.drawStr(
      5,
      53,
      "LEITURA: OK"
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
      "ECHO: SEM RESPOSTA"
    );

    oled.drawStr(
      5,
      43,
      "COLOQUE UM OBJETO"
    );

    oled.setFont(
      u8g2_font_ncenB10_tr
    );

    oled.drawStr(
      48,
      58,
      "ERRO!"
    );
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
    "BNO RESET"
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
      "SENSOR REINICIADO"
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

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  // BOTOES

  pinMode(
    BOTAO_SELECIONAR,
    INPUT_PULLUP
  );

  pinMode(
    BOTAO_MEXER,
    INPUT_PULLUP
  );

  pinMode(
    BOTAO_VOLTAR,
    INPUT_PULLUP
  );

  // HC-SR04

  pinMode(
    ULTRA_TRIG,
    OUTPUT
  );

  pinMode(
    ULTRA_ECHO,
    INPUT
  );

  digitalWrite(
    ULTRA_TRIG,
    LOW
  );

  // RST BNO

  pinMode(
    BNO_RST,
    OUTPUT
  );

  digitalWrite(
    BNO_RST,
    HIGH
  );

  // I2C

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  // OLED

  oled.setI2CAddress(
    0x3C << 1
  );

  oled.begin();

  // CARREGAMENTO

  telaCarregamento();

  // BNO

  if (!bno.begin()) {

    oled.clearBuffer();

    oled.setFont(
      u8g2_font_ncenB10_tr
    );

    oled.drawStr(
      20,
      25,
      "BNO055 ERRO!"
    );

    oled.setFont(
      u8g2_font_6x10_tr
    );

    oled.drawStr(
      18,
      43,
      "Sensor nao encontrado"
    );

    oled.sendBuffer();

    while (1) {
      delay(100);
    }
  }

  Serial.println(
    "BNO055 encontrado!"
  );

  bno.setExtCrystalUse(true);

  delay(500);

  mostrarMenu();
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  bool selecionar = lerBotaoEstavel(
    BOTAO_SELECIONAR,
    leituraBrutaSelecionar,
    estadoEstavelSelecionar,
    ultimaMudancaBrutaSelecionar
  );

  bool mexer = lerBotaoEstavel(
    BOTAO_MEXER,
    leituraBrutaMexer,
    estadoEstavelMexer,
    ultimaMudancaBrutaMexer
  );

  bool voltar = lerBotaoEstavel(
    BOTAO_VOLTAR,
    leituraBrutaVoltar,
    estadoEstavelVoltar,
    ultimaMudancaBrutaVoltar
  );

  // ===================================================
  // MENU PRINCIPAL
  // ===================================================

  if (
    tela == TELA_MENU
  ) {

    if (
      estadoAnteriorMexer == HIGH &&
      mexer == LOW &&
      millis() - ultimoMexer > DEBOUNCE
    ) {

      ultimoMexer =
        millis();

      opcaoMenu++;

      if (
        opcaoMenu >= totalMenu
      ) {

        opcaoMenu = 0;
      }

      mostrarMenu();
    }

    if (
      estadoAnteriorSelecionar == HIGH &&
      selecionar == LOW &&
      millis() - ultimoSelecionar > DEBOUNCE
    ) {

      ultimoSelecionar =
        millis();

      switch (
        opcaoMenu
      ) {

        case 0:

          tela =
            TELA_ESTRATEGIA;

          opcaoEstrategia = 0;

          mostrarMenuEstrategia();

          break;

        case 1:

          tela =
            TELA_TELEMETRIA;

          opcaoTelemetria = 0;

          mostrarMenuTelemetria();

          break;

        case 2:

          tela =
            TELA_BATERIA;

          mostrarEmBreve(
            "BATERIA"
          );

          break;

        case 3:

          tela =
            TELA_CHECKLIST;

          opcaoChecklist = 0;

          mostrarMenuChecklist();

          break;
      }
    }
  }

  // ===================================================
  // ESTRATEGIA
  // ===================================================

  else if (
    tela == TELA_ESTRATEGIA
  ) {

    if (
      estadoAnteriorMexer == HIGH &&
      mexer == LOW &&
      millis() - ultimoMexer > DEBOUNCE
    ) {

      ultimoMexer =
        millis();

      opcaoEstrategia++;

      if (
        opcaoEstrategia >=
        totalEstrategia
      ) {

        opcaoEstrategia = 0;
      }

      mostrarMenuEstrategia();
    }

    if (
      estadoAnteriorSelecionar == HIGH &&
      selecionar == LOW &&
      millis() - ultimoSelecionar > DEBOUNCE
    ) {

      ultimoSelecionar =
        millis();

      if (
        opcaoEstrategia == 0
      ) {

        tela =
          TELA_ATAQUE;

        mostrarEmBreve(
          "ATAQUE"
        );

      } else {

        tela =
          TELA_DEFESA;

        mostrarEmBreve(
          "DEFESA"
        );
      }
    }

    if (
      estadoAnteriorVoltar == HIGH &&
      voltar == LOW &&
      millis() - ultimoVoltar > DEBOUNCE
    ) {

      ultimoVoltar =
        millis();

      tela =
        TELA_MENU;

      mostrarMenu();
    }
  }

  // ===================================================
  // TELEMETRIA
  // ===================================================

  else if (
    tela == TELA_TELEMETRIA
  ) {

    if (
      estadoAnteriorMexer == HIGH &&
      mexer == LOW &&
      millis() - ultimoMexer > DEBOUNCE
    ) {

      ultimoMexer =
        millis();

      opcaoTelemetria++;

      if (
        opcaoTelemetria >=
        totalTelemetria
      ) {

        opcaoTelemetria = 0;
      }

      mostrarMenuTelemetria();
    }

    if (
      estadoAnteriorSelecionar == HIGH &&
      selecionar == LOW &&
      millis() - ultimoSelecionar > DEBOUNCE
    ) {

      ultimoSelecionar =
        millis();

      switch (
        opcaoTelemetria
      ) {

        case 0:

          tela =
            TELA_MENU_BNO;

          opcaoBNO = 0;

          mostrarMenuBNO();

          break;

        case 1:

          tela =
            TELA_KICKER;

          mostrarEmBreve(
            "KICKER"
          );

          break;

        case 2:

          tela =
            TELA_ULTRA;

          mostrarUltra();

          break;

        case 3:

          tela =
            TELA_TELEMETRIA;

          mostrarEmBreve(
            "INFRA"
          );

          break;

        case 4:

          tela =
            TELA_LDR;

          mostrarEmBreve(
            "LDR"
          );

          break;
      }
    }

    if (
      estadoAnteriorVoltar == HIGH &&
      voltar == LOW &&
      millis() - ultimoVoltar > DEBOUNCE
    ) {

      ultimoVoltar =
        millis();

      tela =
        TELA_MENU;

      mostrarMenu();
    }
  }

  // ===================================================
  // MENU DO BNO
  // ===================================================

  else if (
    tela == TELA_MENU_BNO
  ) {

    // MEXER

    if (
      estadoAnteriorMexer == HIGH &&
      mexer == LOW &&
      millis() - ultimoMexer > DEBOUNCE
    ) {

      ultimoMexer =
        millis();

      opcaoBNO++;

      if (
        opcaoBNO >= totalBNO
      ) {

        opcaoBNO = 0;
      }

      mostrarMenuBNO();
    }

    // SELECT

    if (
      estadoAnteriorSelecionar == HIGH &&
      selecionar == LOW &&
      millis() - ultimoSelecionar > DEBOUNCE
    ) {

      ultimoSelecionar =
        millis();

      switch (
        opcaoBNO
      ) {

        // ---------------------------------------------
        // LEITURA
        // ---------------------------------------------

        case 0:

          tela =
            TELA_BNO_LEITURA;

          mostrarBNO();

          break;

        // ---------------------------------------------
        // ZERO
        // ---------------------------------------------

        case 1:

          zerarBNO();

          break;

        // ---------------------------------------------
        // RESET SENSOR
        // ---------------------------------------------

        case 2:

          tela =
            TELA_BNO_RESET;

          {

            bool sucesso =
              resetarBNOSensor();

            mostrarResultadoReset(
              sucesso
            );

            if (sucesso) {

              tela =
                TELA_BNO_LEITURA;

              mostrarBNO();

            } else {

              tela =
                TELA_MENU_BNO;

              mostrarMenuBNO();
            }
          }

          break;
      }
    }

    // VOLTAR

    if (
      estadoAnteriorVoltar == HIGH &&
      voltar == LOW &&
      millis() - ultimoVoltar > DEBOUNCE
    ) {

      ultimoVoltar =
        millis();

      tela =
        TELA_TELEMETRIA;

      mostrarMenuTelemetria();
    }
  }

  // ===================================================
  // LEITURA BNO
  // ===================================================

  else if (
    tela == TELA_BNO_LEITURA
  ) {

    // SELECT = ZERO

    if (
      estadoAnteriorSelecionar == HIGH &&
      selecionar == LOW &&
      millis() - ultimoSelecionar > DEBOUNCE
    ) {

      ultimoSelecionar =
        millis();

      zerarBNO();
    }

    // VOLTAR = MENU BNO

    if (
      estadoAnteriorVoltar == HIGH &&
      voltar == LOW &&
      millis() - ultimoVoltar > DEBOUNCE
    ) {

      ultimoVoltar =
        millis();

      tela =
        TELA_MENU_BNO;

      mostrarMenuBNO();
    }

    // Atualização
    static unsigned long ultimaAtualizacaoBNO = 0;

    if (
      millis() -
      ultimaAtualizacaoBNO >= 100
    ) {

      ultimaAtualizacaoBNO =
        millis();

      mostrarBNO();
    }
  }

  // ===================================================
  // TELEMETRIA ULTRA
  // ===================================================

  else if (
    tela == TELA_ULTRA
  ) {

    static unsigned long ultimaAtualizacaoUltra = 0;

    if (
      millis() - ultimaAtualizacaoUltra >= 180
    ) {

      ultimaAtualizacaoUltra =
        millis();

      mostrarUltra();
    }

    if (
      estadoAnteriorVoltar == HIGH &&
      voltar == LOW &&
      millis() - ultimoVoltar > DEBOUNCE
    ) {

      ultimoVoltar =
        millis();

      tela =
        TELA_TELEMETRIA;

      mostrarMenuTelemetria();
    }
  }

  // ===================================================
  // CHECKLIST
  // ===================================================

  else if (
    tela == TELA_CHECKLIST
  ) {

    // MEXER = alterna entre BNO e ULTRA

    if (
      estadoAnteriorMexer == HIGH &&
      mexer == LOW &&
      millis() - ultimoMexer > DEBOUNCE
    ) {

      ultimoMexer =
        millis();

      opcaoChecklist++;

      if (
        opcaoChecklist >= totalChecklist
      ) {

        opcaoChecklist = 0;
      }

      mostrarMenuChecklist();
    }

    // SELECT

    if (
      estadoAnteriorSelecionar == HIGH &&
      selecionar == LOW &&
      millis() - ultimoSelecionar > DEBOUNCE
    ) {

      ultimoSelecionar =
        millis();

      if (
        opcaoChecklist == 0
      ) {

        testarBNONaTela();

      } else if (
        opcaoChecklist == 1
      ) {

        testarUltraNaTela();
      }
    }

    // VOLTAR

    if (
      estadoAnteriorVoltar == HIGH &&
      voltar == LOW &&
      millis() - ultimoVoltar > DEBOUNCE
    ) {

      ultimoVoltar =
        millis();

      tela =
        TELA_MENU;

      mostrarMenu();
    }
  }

  // ===================================================
  // RESULTADO CHECKLIST
  // ===================================================

  else if (
    tela == TELA_TESTE_BNO
  ) {

    if (
      estadoAnteriorSelecionar == HIGH &&
      selecionar == LOW &&
      millis() - ultimoSelecionar > DEBOUNCE
    ) {

      ultimoSelecionar =
        millis();

      testarBNONaTela();
    }

    if (
      estadoAnteriorVoltar == HIGH &&
      voltar == LOW &&
      millis() - ultimoVoltar > DEBOUNCE
    ) {

      ultimoVoltar =
        millis();

      tela =
        TELA_CHECKLIST;

      mostrarMenuChecklist();
    }
  }

  // ===================================================
  // RESULTADO CHECKLIST ULTRA
  // ===================================================

  else if (
    tela == TELA_TESTE_ULTRA
  ) {

    if (
      estadoAnteriorSelecionar == HIGH &&
      selecionar == LOW &&
      millis() - ultimoSelecionar > DEBOUNCE
    ) {

      ultimoSelecionar =
        millis();

      testarUltraNaTela();
    }

    if (
      estadoAnteriorVoltar == HIGH &&
      voltar == LOW &&
      millis() - ultimoVoltar > DEBOUNCE
    ) {

      ultimoVoltar =
        millis();

      tela =
        TELA_CHECKLIST;

      mostrarMenuChecklist();
    }
  }

  // ===================================================
  // OUTRAS TELAS
  // ===================================================

  else {

    if (
      estadoAnteriorVoltar == HIGH &&
      voltar == LOW &&
      millis() - ultimoVoltar > DEBOUNCE
    ) {

      ultimoVoltar =
        millis();

      tela =
        TELA_MENU;

      mostrarMenu();
    }
  }

  // ===================================================
  // ESTADOS ANTERIORES
  // =====================================================

  estadoAnteriorSelecionar =
    selecionar;

  estadoAnteriorMexer =
    mexer;

  estadoAnteriorVoltar =
    voltar;
}