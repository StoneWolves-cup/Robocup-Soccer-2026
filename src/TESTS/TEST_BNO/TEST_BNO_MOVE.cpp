#include <Arduino.h>
#include <ConfigPinos.h>
#include <math.h>
#include <stdlib.h>
#include <Motores.h>
#include <Omni.h>
#include <PID.h>

namespace {
constexpr int UART_RX_PIN = ConfigPinos::Geral::UART1::RX;
constexpr int UART_TX_PIN = ConfigPinos::Geral::UART1::TX;
constexpr uint32_t TIMEOUT_EMISSOR_MS = 300;
constexpr uint32_t INTERVALO_TELEMETRIA_MS = 100;

constexpr float KP_INICIAL = 0.0044f;
constexpr float KI_INICIAL = 0.0000022f;
constexpr float KD_INICIAL = 0.0001f;
constexpr float LIMITE_INTEGRAL = 300.0f;
constexpr float ZONA_MORTA_GRAUS = 0.70f;
constexpr float ROTACAO_MAXIMA = 0.86f;
constexpr float PASSO_ROTACAO = 0.006f;
constexpr float PASSO_KP = 0.0002f;
constexpr float PASSO_KI = 0.0000005f;
constexpr float PASSO_KD = 0.0001f;
constexpr size_t TAMANHO_LINHA = 16;

Motores motores;
Omni omni;
PID pidRumo(KP_INICIAL, KI_INICIAL, KD_INICIAL,
            -ROTACAO_MAXIMA, ROTACAO_MAXIMA);

float heading = 0.0f;
float gyroEstimado = 0.0f;
float correcaoAtual = 0.0f;
float kpRumo = KP_INICIAL;
float kiRumo = KI_INICIAL;
float kdRumo = KD_INICIAL;
char linhaRecebida[TAMANHO_LINHA] = {};
size_t tamanhoLinha = 0;
uint32_t ultimaRecepcaoMs = 0;
uint32_t ultimaLeituraUs = 0;
uint32_t ultimaTelemetriaMs = 0;
bool linhaMuitoLonga = false;
bool headingDisponivel = false;

float erroAngular(float desejado, float atual) {
  float erro = desejado - atual;
  while (erro > 180.0f) erro -= 360.0f;
  while (erro < -180.0f) erro += 360.0f;
  return erro;
}

void pararMovimento() {
  correcaoAtual = 0.0f;
  motores.parar();
}

void imprimirGanhos() {
  Serial.printf("Kp=%.4f Ki=%.7f Kd=%.4f\n", kpRumo, kiRumo, kdRumo);
}

void ajustarGanhos(char comando) {
  switch (comando) {
    case 'q':
    case 'Q':
      kpRumo = fminf(kpRumo + PASSO_KP, 0.150f);
      break;
    case 'a':
    case 'A':
      kpRumo = fmaxf(kpRumo - PASSO_KP, 0.0f);
      break;
    case 'w':
    case 'W':
      kiRumo = fminf(kiRumo + PASSO_KI, 0.0001f);
      break;
    case 's':
    case 'S':
      kiRumo = fmaxf(kiRumo - PASSO_KI, 0.0f);
      break;
    case 'e':
    case 'E':
      kdRumo = fminf(kdRumo + PASSO_KD, 0.004f);
      break;
    case 'd':
    case 'D':
      kdRumo = fmaxf(kdRumo - PASSO_KD, 0.0f);
      break;
    default:
      return;
  }

  pidRumo.setGains(kpRumo, kiRumo, kdRumo);
  imprimirGanhos();
}

void processarSerial() {
  while (Serial.available() > 0) {
    const char comando = static_cast<char>(Serial.read());
    if (comando == '\r' || comando == '\n') continue;

    ajustarGanhos(comando);
  }
}

bool converterHeading(const char *linha, float &valor) {
  char *fim = nullptr;
  valor = strtof(linha, &fim);
  if (fim == linha || !isfinite(valor)) return false;
  while (*fim == ' ' || *fim == '\t') ++fim;
  return *fim == '\0' && valor >= -180.0f && valor <= 180.0f;
}

void processarLinhaHeading() {
  float novoHeading = 0.0f;
  if (!converterHeading(linhaRecebida, novoHeading)) {
    Serial.printf("Angulo UART invalido: '%s'\n", linhaRecebida);
    return;
  }

  const uint32_t agoraUs = micros();
  const uint32_t decorridoUs = agoraUs - ultimaLeituraUs;
  ultimaLeituraUs = agoraUs;
  const float dt = constrain(decorridoUs / 1000000.0f, 0.001f, 0.3f);

  if (headingDisponivel) {
    gyroEstimado = erroAngular(novoHeading, heading) / dt;
  } else {
    headingDisponivel = true;
    gyroEstimado = 0.0f;
    pidRumo.reset();
  }

  heading = novoHeading;
  ultimaRecepcaoMs = millis();

  float erro = erroAngular(0.0f, heading);
  if (fabsf(erro) <= ZONA_MORTA_GRAUS) erro = 0.0f;

  float alvo = pidRumo.calculate(erro, gyroEstimado, dt);
  alvo = constrain(alvo, -ROTACAO_MAXIMA, ROTACAO_MAXIMA);
  correcaoAtual += constrain(alvo - correcaoAtual,
                             -PASSO_ROTACAO, PASSO_ROTACAO);

  const ComandosMotores rodas = omni.calcular(0.0f, 0.0f, correcaoAtual);
  motores.definir(rodas.m1, rodas.m2, rodas.m3, rodas.m4);

  const uint32_t agoraMs = millis();
  if ((uint32_t)(agoraMs - ultimaTelemetriaMs) >=
      INTERVALO_TELEMETRIA_MS) {
    ultimaTelemetriaMs = agoraMs;
    Serial.printf("rumo=%.2f erro=%.2f gyro=%.1f rot=%.3f Kp=%.4f Ki=%.7f Kd=%.4f rodas=%.3f,%.3f,%.3f,%.3f\n",
                  heading, erro, gyroEstimado, correcaoAtual, kpRumo, kiRumo,
                  kdRumo,
                  rodas.m1, rodas.m2, rodas.m3, rodas.m4);
  }
}

void processarUART() {
  while (Serial1.available() > 0) {
    const char caractere = static_cast<char>(Serial1.read());
    if (caractere == '\r') continue;
    if (caractere == '\n') {
      if (linhaMuitoLonga) {
        Serial.println("Linha UART do angulo excedeu o tamanho permitido.");
      } else if (tamanhoLinha > 0) {
        linhaRecebida[tamanhoLinha] = '\0';
        processarLinhaHeading();
      }
      tamanhoLinha = 0;
      linhaMuitoLonga = false;
      continue;
    }

    if (tamanhoLinha + 1 < TAMANHO_LINHA) {
      linhaRecebida[tamanhoLinha++] = caractere;
    } else {
      linhaMuitoLonga = true;
    }
  }
}
} // namespace

void setup() {
  Serial.begin(460800);
  Serial1.begin(460800, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  pidRumo.setIntegralLimit(LIMITE_INTEGRAL);
  motores.begin();
  motores.parar();

  Serial.println("=== Receptor UART: PID de rumo e motores ===");
  Serial.println("Recebe somente o angulo do emissor; alvo inicial = 0 graus.");
  Serial.println("Serial: q/a Kp, w/s Ki, e/d Kd.");
  imprimirGanhos();
}

void loop() {
  processarSerial();
  processarUART();

  if (headingDisponivel &&
      (uint32_t)(millis() - ultimaRecepcaoMs) > TIMEOUT_EMISSOR_MS) {
    pararMovimento();
    pidRumo.reset();
    headingDisponivel = false;
    Serial.println("Sem angulo recente do emissor; motores parados.");
  }
}
