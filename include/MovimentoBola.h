#ifndef MOVIMENTO_BOLA_H
#define MOVIMENTO_BOLA_H

#include <Angulo.h>
#include <ProximidadeIR.h>
#include <math.h>

namespace MovimentoBola {
constexpr float ANGULO_SETOR_FRONTAL_GRAUS = 8.0f;
constexpr float ANGULO_LATERAL_GRAUS = 80.0f;
constexpr float ANGULO_TRASEIRO_GRAUS = 180.0f;
constexpr float ANGULO_ZONA_TRAVA_TRASEIRA_GRAUS = 165.0f;
constexpr float DESVIO_J_LONGE_GRAUS = 4.0f;
constexpr float DESVIO_J_MEDIA_GRAUS = 50.0f;
constexpr float DESVIO_J_PERTO_GRAUS = 60.0f;
constexpr float ANGULO_REFERENCIA_J_GRAUS = 90.0f;
constexpr float MOVIMENTO_J_LONGE_A_90_GRAUS = 105.0f;
constexpr float MOVIMENTO_J_MEDIA_A_90_GRAUS = 125.0f;
constexpr float MOVIMENTO_J_PERTO_A_90_GRAUS = 160.0f;

class ControladorJ {
 public:
  void resetar() {
    _ladoCurva = 1.0f;
    _ladoDefinido = false;
  }

  float calcularAngulo(float anguloBolaGraus, float proximidade) {
    if (proximidade < 0.0f) proximidade = 0.0f;
    if (proximidade > 1.0f) proximidade = 1.0f;

    const float desvio = calcularDesvio(proximidade);
    const float assinado = erroAngular(anguloBolaGraus, 0.0f);
    const float magnitude = fabsf(assinado);
    if (magnitude > ANGULO_SETOR_FRONTAL_GRAUS &&
        magnitude < ANGULO_ZONA_TRAVA_TRASEIRA_GRAUS) {
      _ladoCurva = assinado < 0.0f ? -1.0f : 1.0f;
      _ladoDefinido = true;
    } else if (magnitude >= ANGULO_ZONA_TRAVA_TRASEIRA_GRAUS &&
               !_ladoDefinido) {
      _ladoCurva = assinado < 0.0f ? -1.0f : 1.0f;
      _ladoDefinido = true;
    }

    const float lado = magnitude >= ANGULO_ZONA_TRAVA_TRASEIRA_GRAUS &&
                               _ladoDefinido
        ? _ladoCurva
        : (assinado < 0.0f ? -1.0f : 1.0f);
    const float progresso = magnitude <= ANGULO_LATERAL_GRAUS
        ? 0.0f
        : (magnitude - ANGULO_LATERAL_GRAUS) /
              (ANGULO_TRASEIRO_GRAUS - ANGULO_LATERAL_GRAUS);
    const float curvaSuave = progresso * progresso * (3.0f - 2.0f * progresso);
    const float progressoReferencia =
        (ANGULO_REFERENCIA_J_GRAUS - ANGULO_LATERAL_GRAUS) /
        (ANGULO_TRASEIRO_GRAUS - ANGULO_LATERAL_GRAUS);
    const float curvaReferencia = progressoReferencia * progressoReferencia *
        (3.0f - 2.0f * progressoReferencia);
    const float ajusteReferencia = calcularAlvoA90(proximidade) -
        ANGULO_REFERENCIA_J_GRAUS - desvio * curvaReferencia;
    float pesoAjuste = 0.0f;
    if (magnitude > ANGULO_SETOR_FRONTAL_GRAUS &&
        magnitude <= ANGULO_REFERENCIA_J_GRAUS) {
      pesoAjuste = (magnitude - ANGULO_SETOR_FRONTAL_GRAUS) /
          (ANGULO_REFERENCIA_J_GRAUS - ANGULO_SETOR_FRONTAL_GRAUS);
    } else if (magnitude > ANGULO_REFERENCIA_J_GRAUS) {
      pesoAjuste = (ANGULO_TRASEIRO_GRAUS - magnitude) /
          (ANGULO_TRASEIRO_GRAUS - ANGULO_REFERENCIA_J_GRAUS);
    }
    pesoAjuste = pesoAjuste * pesoAjuste * (3.0f - 2.0f * pesoAjuste);
    const float movimento =
        magnitude + desvio * curvaSuave + ajusteReferencia * pesoAjuste;
    return lado * movimento;
  }

 private:
  static float calcularAlvoA90(float proximidade) {
    if (proximidade < ProximidadeIR::INDICE_MINIMO_MEDIA) {
      const float progresso =
          proximidade / ProximidadeIR::INDICE_MINIMO_MEDIA;
      return MOVIMENTO_J_LONGE_A_90_GRAUS +
          (MOVIMENTO_J_MEDIA_A_90_GRAUS - MOVIMENTO_J_LONGE_A_90_GRAUS) *
              progresso;
    }
    if (proximidade < ProximidadeIR::INDICE_MINIMO_PERTO) {
      const float progresso =
          (proximidade - ProximidadeIR::INDICE_MINIMO_MEDIA) /
          (ProximidadeIR::INDICE_MINIMO_PERTO -
           ProximidadeIR::INDICE_MINIMO_MEDIA);
      return MOVIMENTO_J_MEDIA_A_90_GRAUS +
          (MOVIMENTO_J_PERTO_A_90_GRAUS - MOVIMENTO_J_MEDIA_A_90_GRAUS) *
              progresso;
    }
    return MOVIMENTO_J_PERTO_A_90_GRAUS;
  }

  static float calcularDesvio(float proximidade) {
    if (proximidade < ProximidadeIR::INDICE_MINIMO_MEDIA) {
      const float progresso =
          proximidade / ProximidadeIR::INDICE_MINIMO_MEDIA;
      return DESVIO_J_LONGE_GRAUS +
          (DESVIO_J_MEDIA_GRAUS - DESVIO_J_LONGE_GRAUS) * progresso;
    }
    if (proximidade < ProximidadeIR::INDICE_MINIMO_PERTO) {
      const float progresso =
          (proximidade - ProximidadeIR::INDICE_MINIMO_MEDIA) /
          (ProximidadeIR::INDICE_MINIMO_PERTO -
           ProximidadeIR::INDICE_MINIMO_MEDIA);
      return DESVIO_J_MEDIA_GRAUS +
          (DESVIO_J_PERTO_GRAUS - DESVIO_J_MEDIA_GRAUS) * progresso;
    }
    return DESVIO_J_PERTO_GRAUS;
  }

  float _ladoCurva = 1.0f;
  bool _ladoDefinido = false;
};
}  // namespace MovimentoBola

#endif
