#ifndef MOVIMENTO_DEFESA_H
#define MOVIMENTO_DEFESA_H

#include <Angulo.h>
#include <math.h>
#include <stdint.h>

namespace MovimentoDefesa {
constexpr float DISTANCIA_ALVO_CM = 39.0f;
constexpr float ZONA_MORTA_DISTANCIA_CM = 2.0f;
constexpr float GANHO_DISTANCIA = 0.20f;
constexpr float VELOCIDADE_MAXIMA_DISTANCIA = 0.90f;
constexpr float VELOCIDADE_LATERAL_MAXIMA = 1.0f;
constexpr float ANGULO_ZONA_MORTA_GRAUS = 0.8f;
constexpr float DISTANCIA_FAIXA_CENTRAL_U1_MIN_CM = 48.0f;
constexpr float DISTANCIA_FAIXA_CENTRAL_U1_MAX_CM = 113.0f;
constexpr float DISTANCIA_CENTRO_U1_MIN_CM = 65.0f;
constexpr float DISTANCIA_CENTRO_U1_MAX_CM = 75.0f;
constexpr float VELOCIDADE_CENTRO_MINIMA = 0.80f;
constexpr float GANHO_CENTRO_U1 = 0.18f;
constexpr float VELOCIDADE_CENTRO_MAXIMA = 0.80f;
constexpr float DISTANCIA_LIMITE_LATERAL_CM = 45.0f;
constexpr float VELOCIDADE_AFASTAMENTO_LATERAL = 0.70f;
constexpr float DISTANCIA_ATIVACAO_AVANCO_LDR_CM = 25.0f; 
constexpr float ANGULO_ZONA_MORTA_LONGE_GRAUS = 0.6f;
constexpr float ANGULO_ZONA_MORTA_PERTO_GRAUS = 5.0f;
constexpr float FATOR_EXPONENCIAL_LIMIAR = 3.0f;
constexpr float NORMALIZADOR_EXPONENCIAL_LIMIAR = 19.0855369f;
constexpr uint32_t TEMPO_ESTABILIDADE_AVANCO_MS = 3000;
constexpr uint32_t DURACAO_AVANCO_BOLA_MS = 1900;
constexpr uint32_t COOLDOWN_AVANCO_BOLA_MS = 500;
constexpr float VELOCIDADE_AVANCO_BOLA = 1.0f;

inline float progressoExponencialLimiar(float proximidade) {
  if (!isfinite(proximidade)) proximidade = 0.0f;
  proximidade = fmaxf(0.0f, fminf(1.0f, proximidade));
  return (expf(FATOR_EXPONENCIAL_LIMIAR * proximidade) - 1.0f) /
      NORMALIZADOR_EXPONENCIAL_LIMIAR;
}

inline float limiarAngular(float proximidade) {
  const float progresso = progressoExponencialLimiar(proximidade);
  return ANGULO_ZONA_MORTA_LONGE_GRAUS +
      (ANGULO_ZONA_MORTA_PERTO_GRAUS -
       ANGULO_ZONA_MORTA_LONGE_GRAUS) * progresso;
}

class AvancoBolaTemporizado {
 public:
  enum class Estado : uint8_t {
    Inativo,
    Estabilizando,
    Avancando,
    Cooldown,
  };

  void resetar() {
    _estado = Estado::Inativo;
    _inicioEstadoMs = 0;
  }

  void interromper(uint32_t agoraMs) {
    if (_estado == Estado::Avancando) {
      _estado = Estado::Cooldown;
      _inicioEstadoMs = agoraMs;
    } else if (_estado == Estado::Estabilizando) {
      _estado = Estado::Inativo;
      _inicioEstadoMs = agoraMs;
    }
  }

  bool atualizar(uint32_t agoraMs, bool bolaValida, bool bolaNaFaixaAvanco,
                 float proximidade, float anguloBolaGraus) {
    if (_estado == Estado::Avancando) {
      if (!bolaValida || !bolaNaFaixaAvanco || !isfinite(anguloBolaGraus)) {
        _estado = Estado::Cooldown;
        _inicioEstadoMs = agoraMs;
        return false;
      }
      if ((uint32_t)(agoraMs - _inicioEstadoMs) <
          DURACAO_AVANCO_BOLA_MS) {
        return true;
      }
      _estado = Estado::Cooldown;
      _inicioEstadoMs = agoraMs;
      return false;
    }

    if (_estado == Estado::Cooldown) {
      if ((uint32_t)(agoraMs - _inicioEstadoMs) <
          COOLDOWN_AVANCO_BOLA_MS) {
        return false;
      }
      _estado = Estado::Inativo;
    }

    const bool anguloEstavel = bolaValida && bolaNaFaixaAvanco &&
        isfinite(anguloBolaGraus) &&
        fabsf(erroAngular(anguloBolaGraus, 0.0f)) <=
            limiarAngular(proximidade);
    if (!anguloEstavel) {
      _estado = Estado::Inativo;
      _inicioEstadoMs = agoraMs;
      return false;
    }

    if (_estado == Estado::Inativo) {
      _estado = Estado::Estabilizando;
      _inicioEstadoMs = agoraMs;
      return false;
    }

    if ((uint32_t)(agoraMs - _inicioEstadoMs) <
        TEMPO_ESTABILIDADE_AVANCO_MS) {
      return false;
    }

    _estado = Estado::Avancando;
    _inicioEstadoMs = agoraMs;
    return true;
  }

  Estado estado() const { return _estado; }

 private:
  Estado _estado = Estado::Inativo;
  uint32_t _inicioEstadoMs = 0;
};

inline void vetorAvancoBola(float anguloBolaGraus, float &comandoX,
                            float &comandoY) {
  constexpr float RADIANOS_POR_GRAU = 0.01745329251994329577f;
  const float anguloRad =
      erroAngular(anguloBolaGraus, 0.0f) * RADIANOS_POR_GRAU;
  comandoX = sinf(anguloRad) * VELOCIDADE_AVANCO_BOLA;
  // Omni usa y positivo para frente; o avanco deve seguir a bola frontal.
  comandoY = cosf(anguloRad) * VELOCIDADE_AVANCO_BOLA;
}

inline bool priorizarAvancoLDR(
    float distanciaTraseiraCm, bool leituraValida, bool superiorBranco,
    bool centroBranco, bool avancoAtivo) {
  if (!leituraValida || !isfinite(distanciaTraseiraCm)) return false;
  if (avancoAtivo) return distanciaTraseiraCm < DISTANCIA_ALVO_CM;
  return distanciaTraseiraCm <= DISTANCIA_ATIVACAO_AVANCO_LDR_CM &&
      (superiorBranco || centroBranco);
}

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
