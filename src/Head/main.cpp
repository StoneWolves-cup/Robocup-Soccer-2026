#include <Arduino.h>
#include <math.h>
#include <IRProtocol.h>
#include <BNO055.h>

namespace {
constexpr uint8_t NUM_SENSORES = IR_SENSOR_COUNT;
constexpr uint32_t JANELA_US = 5000;
constexpr uint32_t SERIAL_MS = 100;
constexpr uint32_t INTERVALO_ENVIO_MS = 20;
constexpr uint32_t INTERVALO_BNO_MS = 10;
constexpr float ALPHA = 0.25f;
constexpr float LIMIAR_ATIVIDADE = 2.0f;
constexpr float CONFIANCA_MIN = 0.15f;
constexpr uint32_t UART_BAUD = 115200;
constexpr int UART_RX_PIN = 18;
constexpr int UART_TX_PIN = 17;
constexpr uint8_t BNO_SDA_PIN = 8;
constexpr uint8_t BNO_SCL_PIN = 9;

// Mesma ordem de pinos, angulos e aquisicao usada no teste TSSP de 16 sensores.
const uint8_t pinos[NUM_SENSORES] = {
  21, 47, 48, 38, 39, 40, 41,
  42, 16, 15, 7, 6, 5, 4, 1, 2
};
const float angulosSensores[NUM_SENSORES] = {
  0.0f, 22.5f, 45.0f, 67.5f, 90.0f, 112.5f, 135.0f, 157.5f,
  180.0f, 202.5f, 225.0f, 247.5f, 270.0f, 292.5f, 315.0f, 337.5f
};

uint32_t totalAmostras = 0;
uint32_t lowCount[NUM_SENSORES] = {};
float atividade[NUM_SENSORES] = {};
float peso[NUM_SENSORES] = {};
float vetorXFiltrado = 0.0f;
float vetorYFiltrado = 0.0f;
float somaPesosFiltrada = 0.0f;
bool filtroIniciado = false;
uint32_t inicioJanela = 0;
uint32_t ultimaSerial = 0;
uint32_t ultimoEnvio = 0;
uint16_t sequenciaIR = 0;
uint16_t sequenciaBNO = 0;
float direcaoGraus = 0.0f;
float confianca = 0.0f;
bool bolaDetectada = false;
BNO055 sensorBNO;
bool bnoDisponivel = false;
uint32_t ultimaAtualizacaoBNO = 0;

// Copiada do TEST_TSSP/TSSP_16_test.cpp para manter o mesmo calculo vetorial.
void calcularDirecao() {
  float x = 0.0f;
  float y = 0.0f;
  float somaPesos = 0.0f;

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    peso[i] = fmaxf(0.0f, atividade[i] - LIMIAR_ATIVIDADE);
    const float rad = angulosSensores[i] * DEG_TO_RAD;
    x += peso[i] * cosf(rad);
    y += peso[i] * sinf(rad);
    somaPesos += peso[i];
  }

  if (!filtroIniciado) {
    vetorXFiltrado = x;
    vetorYFiltrado = y;
    somaPesosFiltrada = somaPesos;
    filtroIniciado = true;
  } else {
    vetorXFiltrado = ALPHA * x + (1.0f - ALPHA) * vetorXFiltrado;
    vetorYFiltrado = ALPHA * y + (1.0f - ALPHA) * vetorYFiltrado;
    somaPesosFiltrada =
        ALPHA * somaPesos + (1.0f - ALPHA) * somaPesosFiltrada;
  }

  if (somaPesos <= 0.0f || somaPesosFiltrada <= 0.0f) {
    bolaDetectada = false;
    confianca = 0.0f;
    return;
  }

  const float moduloFiltrado = sqrtf(
      vetorXFiltrado * vetorXFiltrado + vetorYFiltrado * vetorYFiltrado);
  confianca = moduloFiltrado / somaPesosFiltrada;
  if (confianca < CONFIANCA_MIN) {
    bolaDetectada = false;
    return;
  }

  direcaoGraus = atan2f(vetorYFiltrado, vetorXFiltrado) * RAD_TO_DEG;
  if (direcaoGraus < 0.0f) direcaoGraus += 360.0f;
  if (direcaoGraus > 180.0f) direcaoGraus -= 360.0f;
  bolaDetectada = true;
}

void fecharJanela() {
  if (totalAmostras == 0) return;

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    atividade[i] = 100.0f * lowCount[i] / totalAmostras;
    lowCount[i] = 0;
  }

  totalAmostras = 0;
  calcularDirecao();
}

void transmitirIR() {
  IRActivityPacket pacote{};
  pacote.magic = IR_PACKET_MAGIC;
  pacote.version = IR_PACKET_VERSION;
  pacote.sequence = sequenciaIR++;
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    pacote.activity[i] = static_cast<uint8_t>(atividade[i] + 0.5f);
  }
  pacote.directionTenthsDegrees = bolaDetectada
      ? static_cast<int16_t>(lroundf(direcaoGraus * 10.0f)) : 0;
  int confiancaPercentual = static_cast<int>(lroundf(confianca * 100.0f));
  if (confiancaPercentual < 0) confiancaPercentual = 0;
  if (confiancaPercentual > 100) confiancaPercentual = 100;
  pacote.confidencePercent = static_cast<uint8_t>(confiancaPercentual);
  pacote.ballDetected = bolaDetectada ? 1 : 0;
  pacote.checksum = irPacketChecksum(pacote);
  Serial1.write(reinterpret_cast<const uint8_t *>(&pacote), sizeof(pacote));
}

void transmitirBNO() {
  if (!bnoDisponivel) return;

  BNODataPacket pacote{};
  pacote.magic = BNO_PACKET_MAGIC;
  pacote.version = BNO_PACKET_VERSION;
  pacote.sequence = sequenciaBNO++;
  pacote.headingDegrees = sensorBNO.getHeading();
  pacote.gyroZDegreesPerSecond = sensorBNO.getGyroZ();
  pacote.checksum = bnoPacketChecksum(pacote);
  Serial1.write(reinterpret_cast<const uint8_t *>(&pacote), sizeof(pacote));
}

void imprimirDados() {
  Serial.print("LOW%: ");
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    Serial.print(atividade[i], 1);
    if (i < NUM_SENSORES - 1) Serial.print(',');
  }

  Serial.print(" | Bola: ");
  Serial.print(bolaDetectada ? "SIM" : "NAO");
  if (bolaDetectada) {
    Serial.print(" | Direcao: ");
    Serial.print(direcaoGraus, 1);
    Serial.print(" graus");
  }
  Serial.print(" | Confianca: ");
  Serial.println(confianca, 2);
}
} // namespace

void setup() {
  Serial.begin(115200);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  bnoDisponivel = sensorBNO.begin(BNO_SDA_PIN, BNO_SCL_PIN);
  if (!bnoDisponivel) {
    Serial.println("ERRO: BNO055 nao encontrado em SDA=8/SCL=9; BODY deve manter os motores parados.");
  }

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    if (pinos[i] >= 34 && pinos[i] <= 39) pinMode(pinos[i], INPUT);
    else pinMode(pinos[i], INPUT_PULLUP);
  }

  inicioJanela = micros();
  Serial.println("Coroa TSSP4038 - 16 sensores");
  Serial.println("Head UART1: 115200 baud, 8N1, RX=18 TX=17; TSSP e telemetria BNO055 ativos.");
}

void loop() {
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    if (digitalRead(pinos[i]) == LOW) lowCount[i]++;
  }
  totalAmostras++;

  const uint32_t agoraUs = micros();
  if ((uint32_t)(agoraUs - inicioJanela) >= JANELA_US) {
    fecharJanela();
    inicioJanela = agoraUs;
  }

  const uint32_t agoraMs = millis();
  if (bnoDisponivel &&
      (uint32_t)(agoraMs - ultimaAtualizacaoBNO) >= INTERVALO_BNO_MS) {
    ultimaAtualizacaoBNO = agoraMs;
    sensorBNO.update();
  }
  if ((uint32_t)(agoraMs - ultimoEnvio) >= INTERVALO_ENVIO_MS) {
    ultimoEnvio = agoraMs;
    transmitirIR();
    transmitirBNO();
  }
  if ((uint32_t)(agoraMs - ultimaSerial) >= SERIAL_MS) {
    ultimaSerial = agoraMs;
    imprimirDados();
  }
}
