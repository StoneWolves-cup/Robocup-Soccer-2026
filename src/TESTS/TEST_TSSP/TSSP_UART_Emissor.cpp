#include <Arduino.h>
#include <ConfigPinos.h>
#include <IRProtocol.h>
#include <TSSP.h>
#include <math.h>

namespace {
constexpr uint8_t NUM_SENSORES = IR_SENSOR_COUNT;
constexpr uint32_t SERIAL_MS = 100;
constexpr uint32_t INTERVALO_ENVIO_MS = 20;
constexpr uint32_t UART_BAUD = 460800;
constexpr int UART_RX_PIN = ConfigPinos::Geral::UART1::RX;
constexpr int UART_TX_PIN = ConfigPinos::Geral::UART1::TX;
constexpr float ALPHA = 0.25f;
constexpr float LIMIAR_ATIVIDADE = 2.0f;
constexpr float CONFIANCA_MIN = 0.15f;

const float angulosSensores[NUM_SENSORES] = {
  0.0f, 22.5f, 45.0f, 67.5f, 90.0f, 112.5f, 135.0f, 157.5f,
  180.0f, 202.5f, 225.0f, 247.5f, 270.0f, 292.5f, 315.0f, 337.5f
};

TSSP sensorTSSP;
float atividade[NUM_SENSORES] = {};
float vetorXFiltrado = 0.0f;
float vetorYFiltrado = 0.0f;
float somaPesosFiltrada = 0.0f;
bool filtroIniciado = false;
float direcaoGraus = 0.0f;
float confianca = 0.0f;
bool bolaDetectada = false;
uint16_t sequenciaIR = 0;
uint32_t ultimoEnvio = 0;
uint32_t ultimaSerial = 0;

void calcularDirecao() {
  float x = 0.0f;
  float y = 0.0f;
  float somaPesos = 0.0f;

  for (uint8_t i = 0; i < NUM_SENSORES; ++i) {
    const float peso = fmaxf(0.0f, atividade[i] - LIMIAR_ATIVIDADE);
    const float rad = angulosSensores[i] * DEG_TO_RAD;
    x += peso * cosf(rad);
    y += peso * sinf(rad);
    somaPesos += peso;
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

void atualizarAtividade() {
  for (uint8_t i = 0; i < NUM_SENSORES; ++i) {
    atividade[i] = sensorTSSP.atividadePercentual(i);
  }
  calcularDirecao();
}

void transmitirIR() {
  IRActivityPacket pacote{};
  pacote.magic = IR_PACKET_MAGIC;
  pacote.version = IR_PACKET_VERSION;
  pacote.sequence = sequenciaIR++;

  for (uint8_t i = 0; i < NUM_SENSORES; ++i) {
    pacote.activity[i] = static_cast<uint8_t>(atividade[i] + 0.5f);
  }

  pacote.directionTenthsDegrees = bolaDetectada
      ? static_cast<int16_t>(lroundf(direcaoGraus * 10.0f))
      : 0;
  const int confiancaPercentual =
      static_cast<int>(lroundf(confianca * 100.0f));
  pacote.confidencePercent = static_cast<uint8_t>(
      constrain(confiancaPercentual, 0, 100));
  pacote.ballDetected = bolaDetectada ? 1 : 0;
  pacote.checksum = irPacketChecksum(pacote);

  const size_t bytesEnviados =
      Serial1.write(reinterpret_cast<const uint8_t *>(&pacote), sizeof(pacote));
  if (bytesEnviados != sizeof(pacote)) {
    Serial.println("ERRO: quadro TSSP incompleto na UART1.");
  }
}

void imprimirDiagnostico() {
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
  Serial.begin(115200);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  sensorTSSP.begin();

  Serial.println("Emissor de seguimento TSSP iniciado.");
  Serial.println(
      "UART1: TX GPIO17 -> RX GPIO18 do receptor; conecte o GND comum.");
}

void loop() {
  if (sensorTSSP.atualizar()) atualizarAtividade();

  const uint32_t agora = millis();
  if ((uint32_t)(agora - ultimoEnvio) >= INTERVALO_ENVIO_MS) {
    ultimoEnvio = agora;
    transmitirIR();
  }
  if ((uint32_t)(agora - ultimaSerial) >= SERIAL_MS) {
    ultimaSerial = agora;
    imprimirDiagnostico();
  }
}
