#include <Arduino.h>
#include <math.h>

// ==================================================
// COROA DE 16 SENSORES TSSP4038
// ESP32 DevKit V1
// GPIOs provisórios: confira conflitos antes de usar
// ==================================================

constexpr uint8_t NUM_SENSORES = 16;

const uint8_t pinos[NUM_SENSORES] = {
  34, 23, 22, 21, 19, 18, 17, 16,
  4, 14, 13, 27, 26, 25, 33, 32
};

// Ângulos físicos em graus. Sensor 0 aponta para a frente;
// valores positivos apontam para a esquerda.
const float angulosSensores[NUM_SENSORES] = {
  0.0f, 22.5f, 45.0f, 67.5f, 90.0f, 112.5f, 135.0f, 157.5f,
  180.0f, 202.5f, 225.0f, 247.5f, 270.0f, 292.5f, 315.0f, 337.5f
};

constexpr uint32_t JANELA_US = 5000;
constexpr uint32_t SERIAL_MS = 100;

// 0 = filtro muito lento; 1 = sem filtro
constexpr float ALPHA = 0.25f;

// Ajuste experimentalmente
constexpr float LIMIAR_ATIVIDADE = 2.0f;
constexpr float CONFIANCA_MIN = 0.15f;

uint32_t totalAmostras = 0;
uint32_t lowCount[NUM_SENSORES] = {};
float atividade[NUM_SENSORES] = {};
float peso[NUM_SENSORES] = {};

float vetorXFiltrado = 0.0f;
float vetorYFiltrado = 0.0f;
float somaPesosFiltrada = 0.0f;
bool filtroIniciado = false;

uint32_t inicioJanela;
uint32_t ultimaSerial = 0;

float direcaoGraus = 0.0f;
float confianca = 0.0f;
bool bolaDetectada = false;

// Normaliza um ângulo para [-180, 180)
float normalizarAngulo(float angulo) {
  while (angulo >= 180.0f) angulo -= 360.0f;
  while (angulo < -180.0f) angulo += 360.0f;
  return angulo;
}

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

  // Filtra os componentes do vetor e a soma dos pesos.
  if (!filtroIniciado) {
    vetorXFiltrado = x;
    vetorYFiltrado = y;
    somaPesosFiltrada = somaPesos;
    filtroIniciado = true;
  } else {
    vetorXFiltrado =
      ALPHA * x + (1.0f - ALPHA) * vetorXFiltrado;
    vetorYFiltrado =
      ALPHA * y + (1.0f - ALPHA) * vetorYFiltrado;
    somaPesosFiltrada =
      ALPHA * somaPesos + (1.0f - ALPHA) * somaPesosFiltrada;
  }

  if (somaPesos <= 0.0f || somaPesosFiltrada <= 0.0f) {
    bolaDetectada = false;
    confianca = 0.0f;
    return;
  }

  const float moduloFiltrado = sqrtf(
    vetorXFiltrado * vetorXFiltrado +
    vetorYFiltrado * vetorYFiltrado
  );

  confianca = moduloFiltrado / somaPesosFiltrada;

  if (confianca < CONFIANCA_MIN) {
    bolaDetectada = false;
    return;
  }

  direcaoGraus = atan2f(vetorYFiltrado, vetorXFiltrado) * RAD_TO_DEG;

  if (direcaoGraus < 0.0f) {
    direcaoGraus += 360.0f;
  }

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

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    // GPIOs 34 a 39 não possuem pull-up interno.
    if (pinos[i] >= 34 && pinos[i] <= 39) {
      pinMode(pinos[i], INPUT);
    } else {
      pinMode(pinos[i], INPUT_PULLUP);
    }
  }

  inicioJanela = micros();

  Serial.println("Coroa TSSP4038 - 16 sensores");
  Serial.println("Verifique os GPIOs e os pull-ups externos.");
}

void loop() {
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    if (digitalRead(pinos[i]) == LOW) {
      lowCount[i]++;
    }
  }
  totalAmostras++;

  const uint32_t agoraUs = micros();

  if ((uint32_t)(agoraUs - inicioJanela) >= JANELA_US) {
    fecharJanela();
    inicioJanela = agoraUs;
  }

  const uint32_t agoraMs = millis();

  if ((uint32_t)(agoraMs - ultimaSerial) >= SERIAL_MS) {
    ultimaSerial = agoraMs;
    imprimirDados();
  }
}