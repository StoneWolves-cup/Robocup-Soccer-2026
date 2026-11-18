#ifndef MOVIMENTO_DEFESA_H
#define MOVIMENTO_DEFESA_H

#include <Angulo.h>
#include <math.h>

namespace MovimentoDefesa {
constexpr float DISTANCIA_ALVO_CM = 40.0f;
constexpr float ZONA_MORTA_DISTANCIA_CM = 6.0f;
constexpr float GANHO_DISTANCIA = 0.12f;
constexpr float VELOCIDADE_MAXIMA_DISTANCIA = 0.80f;
constexpr float VELOCIDADE_LATERAL_MINIMA = 0.65f;
constexpr float VELOCIDADE_LATERAL_MAXIMA = 0.90f;
constexpr float ANGULO_ZONA_MORTA_GRAUS = 2.0f;
constexpr float ANGULO_RESPOSTA_FORTE_GRAUS = 6.0f;
constexpr float ANGULO_MAXIMO_LATERAL_GRAUS = 100.0f;
constexpr float DISTANCIA_FAIXA_CENTRAL_U1_MIN_CM = 48.0f;
constexpr float DISTANCIA_FAIXA_CENTRAL_U1_MAX_CM = 113.0f;
constexpr float DISTANCIA_CENTRO_U1_MIN_CM = 65.0f;
constexpr float DISTANCIA_CENTRO_U1_MAX_CM = 75.0f;
constexpr float VELOCIDADE_CENTRO_MINIMA = 0.50f;
constexpr float GANHO_CENTRO_U1 = 0.03f;
constexpr float VELOCIDADE_CENTRO_MAXIMA = 0.70f;
constexpr float DISTANCIA_LIMITE_LATERAL_CM = 43.0f;
constexpr float VELOCIDADE_AFASTAMENTO_LATERAL = 0.12f;

inline float comandoDistancia(float distanciaCm) {
  const float erroCm = distanciaCm - DISTANCIA_ALVO_CM;
  const float magnitudeErro = fabsf(erroCm);
  if (magnitudeErro <= ZONA_MORTA_DISTANCIA_CM) return 0.0f;

  const float erroForaDaZonaMorta = copysignf(
      magnitudeErro - ZONA_MORTA_DISTANCIA_CM, erroCm);
  const float comando = erroForaDaZonaMorta * GANHO_DISTANCIA;
  return fmaxf(-VELOCIDADE_MAXIMA_DISTANCIA,
               fminf(VELOCIDADE_MAXIMA_DISTANCIA, comando));
}

inline float comandoLateral(float anguloBolaGraus) {
  const float anguloAssinado = erroAngular(anguloBolaGraus, 0.0f);
  const float magnitudeAngulo = fabsf(anguloAssinado);
  if (magnitudeAngulo <= ANGULO_ZONA_MORTA_GRAUS) return 0.0f;

  float velocidade;
  if (magnitudeAngulo < ANGULO_RESPOSTA_FORTE_GRAUS) {
    const float progresso =
        (magnitudeAngulo - ANGULO_ZONA_MORTA_GRAUS) /
        (ANGULO_RESPOSTA_FORTE_GRAUS - ANGULO_ZONA_MORTA_GRAUS);
    velocidade = VELOCIDADE_LATERAL_MINIMA * progresso;
  } else {
    const float progresso = fminf(
        1.0f,
        (magnitudeAngulo - ANGULO_RESPOSTA_FORTE_GRAUS) /
            (ANGULO_MAXIMO_LATERAL_GRAUS -
             ANGULO_RESPOSTA_FORTE_GRAUS));
    velocidade = VELOCIDADE_LATERAL_MINIMA +
        (VELOCIDADE_LATERAL_MAXIMA - VELOCIDADE_LATERAL_MINIMA) * progresso;
  }

  return copysignf(velocidade, anguloAssinado);
}

inline bool faixaCentralU1(float distanciaCm, bool leituraValida) {
  return leituraValida && isfinite(distanciaCm) &&
      distanciaCm >= DISTANCIA_FAIXA_CENTRAL_U1_MIN_CM &&
      distanciaCm <= DISTANCIA_FAIXA_CENTRAL_U1_MAX_CM;
}

inline float comandoCentroU1(float distanciaCm, bool leituraValida) {
  if (!leituraValida || !isfinite(distanciaCm) ||
      (distanciaCm >= DISTANCIA_CENTRO_U1_MIN_CM &&
       distanciaCm <= DISTANCIA_CENTRO_U1_MAX_CM)) {
    return 0.0f;
  }

  const bool precisaIrParaEsquerda =
      distanciaCm < DISTANCIA_CENTRO_U1_MIN_CM;
  const float erroCm = precisaIrParaEsquerda
      ? DISTANCIA_CENTRO_U1_MIN_CM - distanciaCm
      : distanciaCm - DISTANCIA_CENTRO_U1_MAX_CM;
  const float velocidade = fminf(
      VELOCIDADE_CENTRO_MAXIMA,
      VELOCIDADE_CENTRO_MINIMA + erroCm * GANHO_CENTRO_U1);
  // Omni usa x positivo para a esquerda: afastar-se da parede direita
  // aumenta a leitura de U1.
  return precisaIrParaEsquerda ? velocidade : -velocidade;
}

inline float limitarComandoLateral(
    float comandoX, float distanciaDireitaCm, bool direitaValida,
    float distanciaEsquerdaCm, bool esquerdaValida) {
  const bool limiteDireita = direitaValida && isfinite(distanciaDireitaCm) &&
      distanciaDireitaCm < DISTANCIA_LIMITE_LATERAL_CM;
  const bool limiteEsquerda = esquerdaValida && isfinite(distanciaEsquerdaCm) &&
      distanciaEsquerdaCm < DISTANCIA_LIMITE_LATERAL_CM;

  if (limiteDireita && limiteEsquerda) return 0.0f;
  if (limiteDireita && comandoX <= 0.0f) {
    return VELOCIDADE_AFASTAMENTO_LATERAL;
  }
  if (limiteEsquerda && comandoX >= 0.0f) {
    return -VELOCIDADE_AFASTAMENTO_LATERAL;
  }
  return comandoX;
}
}  // namespace MovimentoDefesa

#endif
