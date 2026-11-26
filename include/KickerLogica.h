#ifndef KICKER_LOGICA_H
#define KICKER_LOGICA_H

#include <math.h>
#include <stdint.h>

namespace KickerLogica {
constexpr uint32_t TEMPO_DISPARO_MS = 400;
constexpr uint32_t COOLDOWN_MS = 2000;
constexpr float BATERIA_MINIMA_PERCENTUAL = 40.0f;
constexpr float LIMITE_DISTANCIA_LATERAL_CM = 60.0f;
constexpr float LIMITE_DISTANCIA_ATAQUE_CM = 100.0f;

enum class Lado { Nenhum, Esquerda, Direita, Ambos };

inline bool distanciaValida(float distancia) {
  return isfinite(distancia) && distancia >= 2.0f && distancia <= 430.0f;
}

inline Lado selecionar(float frente, float esquerda, float direita,
                      bool leiturasValidas) {
  if (!leiturasValidas || !distanciaValida(frente) ||
      !distanciaValida(esquerda) || !distanciaValida(direita)) {
    return Lado::Nenhum;
  }
  const bool faixaEsquerda =
      esquerda >= 2.0f && esquerda <= LIMITE_DISTANCIA_LATERAL_CM;
  const bool faixaDireita =
      direita >= 2.0f && direita <= LIMITE_DISTANCIA_LATERAL_CM;
  if (faixaEsquerda == faixaDireita) return Lado::Ambos;
  return faixaEsquerda ? Lado::Esquerda : Lado::Direita;
}

inline Lado selecionarAtaque(float direita, bool leituraDireitaValida) {
  if (!leituraDireitaValida || !distanciaValida(direita) ||
      direita <= LIMITE_DISTANCIA_ATAQUE_CM) {
    return Lado::Nenhum;
  }
  return Lado::Esquerda;
}

inline bool bateriaPermite(bool temLeitura, float percentual) {
  return temLeitura && isfinite(percentual) &&
         percentual >= BATERIA_MINIMA_PERCENTUAL && percentual <= 100.0f;
}

inline bool cooldownConcluido(bool jaDisparou, uint32_t fimDisparoMs,
                             uint32_t agoraMs) {
  return !jaDisparou || (uint32_t)(agoraMs - fimDisparoMs) >= COOLDOWN_MS;
}
}  // namespace KickerLogica

#endif
