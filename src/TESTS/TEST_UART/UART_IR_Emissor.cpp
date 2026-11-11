#include <Arduino.h>
#include <math.h>
#include <IRProtocol.h>

namespace {
constexpr uint8_t NUM_SENSORES = IR_SENSOR_COUNT;
constexpr uint32_t JANELA_US = 5000;
constexpr uint32_t SERIAL_MS = 100;
constexpr uint32_t INTERVALO_ENVIO_MS = 20;
constexpr float ALPHA = 0.25f;
constexpr float LIMIAR_ATIVIDADE = 2.0f;
constexpr float CONFIANCA_MIN = 0.15f;
constexpr uint32_t UART_BAUD = 460800;
constexpr int UART_RX_PIN = 18;
constexpr int UART_TX_PIN = 17;

const uint8_t pinos[NUM_SENSORES] = {
  21, 47, 48, 38, 39, 40, 41, 42,
  16, 15, 7, 6, 5, 4, 1, 2
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
float direcaoGraus = 0.0f;
float confianca = 0.0f;
bool bolaDetectada = false;
uint32_t ultimoDiagnostico = 0;
uint32_t pacotesEnviados = 0;

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
  const size_t bytesEnviados =
      Serial1.write(reinterpret_cast<const uint8_t *>(&pacote), sizeof(pacote));
  if (bytesEnviados == sizeof(pacote)) pacotesEnviados++;
}

void imprimirDados() {
  Serial.print("Bola: ");
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
  Serial.begin(UART_BAUD);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    if (pinos[i] >= 34 && pinos[i] <= 39) pinMode(pinos[i], INPUT);
    else pinMode(pinos[i], INPUT_PULLUP);
  }

  inicioJanela = micros();
  Serial.println("Leitura TSSP de 16 sensores com envio de direcao pela UART1.");
  Serial.println("Conecte TX GPIO17 ao RX GPIO18 do receptor e una os GNDs.");
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
  if ((uint32_t)(agoraMs - ultimoEnvio) >= INTERVALO_ENVIO_MS) {
    ultimoEnvio = agoraMs;
    transmitirIR();
  }
  if ((uint32_t)(agoraMs - ultimaSerial) >= SERIAL_MS) {
    ultimaSerial = agoraMs;
    imprimirDados();
  }
  if ((uint32_t)(agoraMs - ultimoDiagnostico) >= 1000) {
    ultimoDiagnostico = agoraMs;
    Serial.print("Pacotes IR enviados pela UART1: ");
    Serial.println(pacotesEnviados);
    pacotesEnviados = 0;
  }
}
