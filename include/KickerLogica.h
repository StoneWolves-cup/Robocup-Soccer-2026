#ifndef KICKER_LOGICA_H
#define KICKER_LOGICA_H

#include <math.h>
#include <stdint.h>

namespace KickerLogica {
constexpr uint32_t TEMPO_DISPARO_MS = 400;
constexpr uint32_t COOLDOWN_MS = 600;
constexpr float BATERIA_MINIMA_PERCENTUAL = 40.0f;

enum class Lado { Nenhum, Esquerda, Direita };

inline bool distanciaValida(float distancia) {
  return isfinite(distancia) && distancia >= 2.0f && distancia <= 430.0f;
}

inline Lado selecionar(float frente, float esquerda, float direita,
                      bool leiturasValidas) {
  if (!leiturasValidas || !distanciaValida(frente) ||
      !distanciaValida(esquerda) || !distanciaValida(direita)) {
    return Lado::Nenhum;
  }
  if (frente <= 80.0f) {
    if (esquerda > direita) return Lado::Direita;
    if (direita > esquerda) return Lado::Esquerda;
    return Lado::Nenhum;
  }
  const bool faixaEsquerda = esquerda < 65.0f;
  const bool faixaDireita = direita < 65.0f;
  if (faixaEsquerda == faixaDireita) return Lado::Nenhum;
  return faixaEsquerda ? Lado::Esquerda : Lado::Direita;
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
