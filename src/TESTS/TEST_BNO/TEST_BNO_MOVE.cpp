#include <Arduino.h>
#include <math.h>
#include <BNO055.h>
#include <Motores.h>
#include <Omni.h>

namespace {
constexpr uint32_t PERIODO_CONTROLE_US = 10000; // 100 Hz
constexpr uint32_t INTERVALO_TELEMETRIA_MS = 100;

// Primeira etapa de ajuste: ganhos conservadores, iguais ao controlador atual.
constexpr float KP_INICIAL = 0.0015f;
constexpr float KD_INICIAL = 0.0020f;
constexpr float ZONA_MORTA_GRAUS = 2.0f;
// Autoriza o controlador a usar toda a escala do driver: 1.0 = 255 PWM.
constexpr float ROTACAO_MAXIMA = 1.0f;
constexpr float PASSO_ROTACAO = 0.006f;
constexpr float PASSO_AJUSTE_GANHO = 0.0001f;

BNO055 bno;
Motores motores;
Omni omni;

float correcaoAtual = 0.0f;
float kpRumo = KP_INICIAL;
float kdGiro = KD_INICIAL;
float sentidoCorrecao = 1.0f;
bool controleAtivo = true;
uint32_t ultimaLeituraUs = 0;
uint32_t ultimaTelemetriaMs = 0;

float erroAngular(float desejado, float atual) {
  float erro = desejado - atual;
  while (erro > 180.0f) erro -= 360.0f;
  while (erro < -180.0f) erro += 360.0f;
  return erro;
}

void pararComando() {
  correcaoAtual = 0.0f;
  motores.parar();
}

void atualizarControle() {
  const uint32_t agoraUs = micros();
  if ((uint32_t)(agoraUs - ultimaLeituraUs) < PERIODO_CONTROLE_US) return;
  ultimaLeituraUs = agoraUs;

  bno.update();
  const float heading = bno.getHeading();
  const float erro = erroAngular(0.0f, heading);
  const float erroControle = fabsf(erro) <= ZONA_MORTA_GRAUS ? 0.0f : erro;

  // PD de rumo: correge o erro e freia a rotacao medida pelo giroscopio.
  float alvo = 0.0f;
  if (controleAtivo) {
    alvo = sentidoCorrecao *
        (erroControle * kpRumo - bno.getGyroZ() * kdGiro);
  } else {
    correcaoAtual = 0.0f;
  }
  alvo = constrain(alvo, -ROTACAO_MAXIMA, ROTACAO_MAXIMA);

  const float passo = constrain(alvo - correcaoAtual,
                                -PASSO_ROTACAO, PASSO_ROTACAO);
  correcaoAtual += passo;

  // x=0 e y=0: apenas gira para corrigir o rumo, sem translacao.
  const ComandosMotores rodas = omni.calcular(0.0f, 0.0f, correcaoAtual);
  motores.definir(rodas.m1, rodas.m2, rodas.m3, rodas.m4);

  const uint32_t agoraMs = millis();
  if ((uint32_t)(agoraMs - ultimaTelemetriaMs) >= INTERVALO_TELEMETRIA_MS) {
    ultimaTelemetriaMs = agoraMs;
    Serial.printf("rumo=%.2f erro=%.2f gyroZ=%.2f rot=%.3f Kp=%.4f Kd=%.4f sentido=%.0f ativo=%u rodas=%.3f,%.3f,%.3f,%.3f\n",
                  heading, erro, bno.getGyroZ(), correcaoAtual,
                  kpRumo, kdGiro, sentidoCorrecao, controleAtivo ? 1 : 0,
                  rodas.m1, rodas.m2, rodas.m3, rodas.m4);
  }
}
} // namespace

void setup() {
  Serial.begin(115200);
  motores.begin();
  motores.parar();
  delay(300);

  Serial.println("=== TESTE BNO: somente correcao de rumo ===");
  Serial.println("O robo nao vai perseguir a bola nem transladar.");
  Serial.println("Mantenha-o no rumo inicial; envie 'c' para definir o zero novamente.");
  Serial.println("Serial: +/- ajusta Kp; [/] ajusta Kd; r inverte o sentido; p pausa/retoma.");

  if (!bno.begin()) {
    Serial.println("ERRO: BNO055 nao encontrado; motores permanecem parados.");
    while (true) {
      motores.parar();
      delay(1000);
    }
  }

  bno.calibrateNorth();
  ultimaLeituraUs = micros();
  Serial.println("Referencia definida: rumo atual = 0 graus.");
}

void loop() {
  if (Serial.available()) {
    const char comando = Serial.read();
    if (comando == 'c' || comando == 'C') {
      pararComando();
      bno.update();
      bno.calibrateNorth();
      ultimaLeituraUs = micros();
      Serial.println("Referencia recalibrada: rumo atual = 0 graus.");
    } else if (comando == '+') {
      kpRumo = fminf(kpRumo + PASSO_AJUSTE_GANHO, 0.020f);
      Serial.printf("Kp=%.4f\n", kpRumo);
    } else if (comando == '-') {
      kpRumo = fmaxf(kpRumo - PASSO_AJUSTE_GANHO, 0.0f);
      Serial.printf("Kp=%.4f\n", kpRumo);
    } else if (comando == ']') {
      kdGiro = fminf(kdGiro + PASSO_AJUSTE_GANHO, 0.010f);
      Serial.printf("Kd=%.4f\n", kdGiro);
    } else if (comando == '[') {
      kdGiro = fmaxf(kdGiro - PASSO_AJUSTE_GANHO, 0.0f);
      Serial.printf("Kd=%.4f\n", kdGiro);
    } else if (comando == 'r' || comando == 'R') {
      pararComando();
      sentidoCorrecao = -sentidoCorrecao;
      Serial.printf("Sentido da correcao=%+.0f\n", sentidoCorrecao);
    } else if (comando == 'p' || comando == 'P') {
      controleAtivo = !controleAtivo;
      pararComando();
      Serial.println(controleAtivo ? "Correcao retomada." : "Correcao pausada; motores parados.");
    }
  }

  atualizarControle();
}
