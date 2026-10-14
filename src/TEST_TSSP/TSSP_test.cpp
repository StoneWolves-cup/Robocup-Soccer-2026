
#include <Arduino.h>
#include <PID.h>

#define PINO_TSSP 19

// ===== CONFIGURACAO DO PID =====
float Kp = 1.0f;
float Ki = 0.05f;
float Kd = 0.0f;

const float SAIDA_MIN = -100.0f;
const float SAIDA_MAX = 100.0f;
const float LIMITE_INTEGRAL = 50.0f;

PID pid(Kp, Ki, Kd, SAIDA_MIN, SAIDA_MAX);

// ===== AMOSTRAGEM =====
const uint32_t JANELA_US = 5000;
const uint32_t PERIODO_PID_MS = 20;

const uint8_t QUADROS = 12;
const float ALPHA = 0.2f;

// ===== DETECCAO =====
float SETPOINT = 30.0f;
float LIMIAR_LIGAR = 3.0f;
float LIMIAR_DESLIGAR = 1.5f;

const uint8_t CONFIRMA_LIGAR = 3;
const uint8_t CONFIRMA_DESLIGAR = 5;

float buffer[QUADROS] = {0};
uint8_t indice = 0;
uint8_t quantidade = 0;

float atividadeFiltrada = 0.0f;
float saidaPID = 0.0f;

bool bolaDetectada = false;
bool primeiraLeitura = true;

uint8_t contadorLigado = 0;
uint8_t contadorDesligado = 0;

uint32_t inicioJanela;
uint32_t ultimaAtualizacao;

uint32_t totalAmostras = 0;
uint32_t amostrasLow = 0;

float calcularMedia() {
  float soma = 0.0f;

  for (uint8_t i = 0; i < quantidade; i++) {
    soma += buffer[i];
  }

  return quantidade > 0 ? soma / quantidade : 0.0f;
}

void setup() {
  Serial.begin(115200);
  pinMode(PINO_TSSP, INPUT);

  pid.setIntegralLimit(LIMITE_INTEGRAL);

  inicioJanela = micros();
  ultimaAtualizacao = millis();

  Serial.println("TSSP4038 + PID.h iniciado");
}

void loop() {
  // Leitura continua do sensor, sem delay().
  if (digitalRead(PINO_TSSP) == LOW) {
    amostrasLow++;
  }
  totalAmostras++;

  uint32_t agoraUs = micros();

  // Calcula atividade a cada janela de 5 ms.
  if ((uint32_t)(agoraUs - inicioJanela) >= JANELA_US) {
    float atividade = 0.0f;

    if (totalAmostras > 0) {
      atividade =
          100.0f * amostrasLow / totalAmostras;
    }

    if (primeiraLeitura) {
      atividadeFiltrada = atividade;
      primeiraLeitura = false;
    } else {
      atividadeFiltrada =
          ALPHA * atividade +
          (1.0f - ALPHA) * atividadeFiltrada;
    }

    // Armazena atividade filtrada no buffer circular.
    buffer[indice] = atividadeFiltrada;
    indice = (indice + 1) % QUADROS;

    if (quantidade < QUADROS) {
      quantidade++;
    }

    amostrasLow = 0;
    totalAmostras = 0;
    inicioJanela = agoraUs;
  }

  uint32_t agoraMs = millis();

  // Atualiza o PID em intervalo fixo.
  if ((uint32_t)(agoraMs - ultimaAtualizacao)
      >= PERIODO_PID_MS) {

    float dt =
        (agoraMs - ultimaAtualizacao) / 1000.0f;

    ultimaAtualizacao = agoraMs;

    float atividadeMedia = calcularMedia();
    float erro = SETPOINT - atividadeMedia;

    // Sem giroscópio: velocidade angular igual a zero.
    saidaPID = pid.calculate(erro, 0.0f, dt);

    // Histerese com confirmacao temporal.
    if (!bolaDetectada) {
      contadorDesligado = 0;

      if (atividadeMedia >= LIMIAR_LIGAR) {
        if (contadorLigado < CONFIRMA_LIGAR) {
          contadorLigado++;
        }
      } else {
        contadorLigado = 0;
      }

      if (contadorLigado >= CONFIRMA_LIGAR) {
        bolaDetectada = true;
        contadorLigado = 0;
        pid.reset();
      }
    } else {
      contadorLigado = 0;

      if (atividadeMedia <= LIMIAR_DESLIGAR) {
        if (contadorDesligado < CONFIRMA_DESLIGAR) {
          contadorDesligado++;
        }
      } else {
        contadorDesligado = 0;
      }

      if (contadorDesligado >= CONFIRMA_DESLIGAR) {
        bolaDetectada = false;
        contadorDesligado = 0;
        pid.reset();
      }
    }

    Serial.print("Atividade: ");
    Serial.print(atividadeMedia, 2);

    Serial.print("% | Erro: ");
    Serial.print(erro, 2);

    Serial.print(" | PID: ");
    Serial.print(saidaPID, 2);

    Serial.print(" | Bola: ");
    Serial.println(
        bolaDetectada ? "DETECTADA" : "AUSENTE"
    );
  }
}
