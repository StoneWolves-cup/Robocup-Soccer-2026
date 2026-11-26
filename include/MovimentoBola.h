#ifndef MOVIMENTO_BOLA_H
#define MOVIMENTO_BOLA_H

#include <Angulo.h>
#include <math.h>

namespace MovimentoBola {
constexpr float ANGULO_SETOR_FRONTAL_GRAUS = 45.0f;
constexpr float ANGULO_LATERAL_GRAUS = 90.0f;
constexpr float ANGULO_INICIO_J_TRASEIRO_GRAUS = 110.0f;
constexpr float ANGULO_TRASEIRO_GRAUS = 180.0f;
constexpr float DESVIO_J_LONGE_GRAUS = 4.8f;
constexpr float DESVIO_J_MEDIA_GRAUS = 9.0f;
constexpr float DESVIO_J_PERTO_GRAUS = 16.6f;

inline float calcularAnguloJ(float anguloBolaGraus, float proximidade) {
  if (proximidade < 0.0f) proximidade = 0.0f;
  if (proximidade > 1.0f) proximidade = 1.0f;
  float desvio;
  if (proximidade < 1.0f / 3.0f) {
    desvio = DESVIO_J_LONGE_GRAUS;
  } else if (proximidade < 2.0f / 3.0f) {
    desvio = DESVIO_J_MEDIA_GRAUS;
  } else {
    desvio = DESVIO_J_PERTO_GRAUS;
  }
  const float assinado = erroAngular(anguloBolaGraus, 0.0f);
  const float magnitude = fabsf(assinado);
  float movimento;
  if (magnitude <= ANGULO_SETOR_FRONTAL_GRAUS) {
    movimento = magnitude;
  } else if (magnitude <= ANGULO_LATERAL_GRAUS) {
    const float progresso = (magnitude - ANGULO_SETOR_FRONTAL_GRAUS) /
        (ANGULO_LATERAL_GRAUS - ANGULO_SETOR_FRONTAL_GRAUS);
    movimento = ANGULO_SETOR_FRONTAL_GRAUS +
        (ANGULO_TRASEIRO_GRAUS - desvio - ANGULO_SETOR_FRONTAL_GRAUS) *
            progresso;
  } else if (magnitude <= ANGULO_INICIO_J_TRASEIRO_GRAUS) {
    const float progresso = (magnitude - ANGULO_LATERAL_GRAUS) /
        (ANGULO_INICIO_J_TRASEIRO_GRAUS - ANGULO_LATERAL_GRAUS);
    movimento = ANGULO_TRASEIRO_GRAUS - desvio * (1.0f - progresso);
  } else {
    const float progresso = (magnitude - ANGULO_INICIO_J_TRASEIRO_GRAUS) /
        (ANGULO_TRASEIRO_GRAUS - ANGULO_INICIO_J_TRASEIRO_GRAUS);
    movimento = ANGULO_TRASEIRO_GRAUS + desvio * progresso * progresso;
  }
  return copysignf(movimento, assinado);
}
}  // namespace MovimentoBola

#endif
