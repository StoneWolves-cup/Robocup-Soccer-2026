
#include <Arduino.h>
/*const int FRENTE = 13;
const int TRAS = 19;
const int ESQUERDA = 25;
const int DIREITA = 23;*/

const uint8_t PINO_TSSP1 = 13;
const uint8_t PINO_TSSP2 = 19;

const uint32_t JANELA_US = 5000;
const uint32_t INTERVALO_SERIAL_MS = 100;

uint32_t inicioJanela = 0;
uint32_t ultimaSerial = 0;

uint32_t totalLeituras1 = 0;
uint32_t leiturasLow1 = 0;

uint32_t totalLeituras2 = 0;
uint32_t leiturasLow2 = 0;

float atividade1 = 0.0f;
float atividade2 = 0.0f;

void setup() {
  Serial.begin(115200);

  pinMode(PINO_TSSP1, INPUT_PULLUP);
  pinMode(PINO_TSSP2, INPUT_PULLUP);

  inicioJanela = micros();
  ultimaSerial = millis();

  Serial.println("Teste de 2 sensores TSSP4038");
  Serial.println("Aguardando sinal infravermelho...");
}

void loop() {
  uint32_t agoraUs = micros();
  uint32_t agoraMs = millis();

  if ((uint32_t)(agoraUs - inicioJanela) >= JANELA_US) {
    if (totalLeituras1 > 0) {
      atividade1 = 100.0f * leiturasLow1 / totalLeituras1;
    }

    if (totalLeituras2 > 0) {
      atividade2 = 100.0f * leiturasLow2 / totalLeituras2;
    }

    totalLeituras1 = 0;
    leiturasLow1 = 0;

    totalLeituras2 = 0;
    leiturasLow2 = 0;

    inicioJanela = agoraUs;
  }

  int estado1 = digitalRead(PINO_TSSP1);
  int estado2 = digitalRead(PINO_TSSP2);

  totalLeituras1++;
  totalLeituras2++;

  if (estado1 == LOW) {
    leiturasLow1++;
  }

  if (estado2 == LOW) {
    leiturasLow2++;
  }

  if ((uint32_t)(agoraMs - ultimaSerial) >=
      INTERVALO_SERIAL_MS) {

    ultimaSerial = agoraMs;

    Serial.print("Sensor 1  | Atividade LOW: ");
    Serial.print(atividade1, 1);
    Serial.print("% | Estado: ");
    Serial.print(estado1 == LOW ? "LOW" : "HIGH");
    Serial.print(" | Deteccao: ");
    Serial.println(atividade1 > 5.0f
                   ? "SINAL DETECTADO"
                   : "SEM SINAL");

    Serial.print("Sensor 2 ( | Atividade LOW: ");
    Serial.print(atividade2, 1);
    Serial.print("% | Estado: ");
    Serial.print(estado2 == LOW ? "LOW" : "HIGH");
    Serial.print(" | Deteccao: ");
    Serial.println(atividade2 > 5.0f
                   ? "SINAL DETECTADO"
                   : "SEM SINAL");

    Serial.println("------------------------------");
  }
}
