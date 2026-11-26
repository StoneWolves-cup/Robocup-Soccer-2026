#ifndef PROXIMIDADE_IR_H
#define PROXIMIDADE_IR_H

#include <stddef.h>
#include <stdint.h>

namespace ProximidadeIR {
// Valores iniciais para calibrar com a bola real e as condicoes do campo.
constexpr uint8_t LIMIAR_SENSOR_ATIVO = 2;
constexpr float ATIVIDADE_LONGE = 2.0f;
constexpr float ATIVIDADE_PERTO = 25.0f;
constexpr float SENSORES_LONGE = 1.0f;
constexpr float SENSORES_PERTO = 5.0f;
constexpr float FILTRO_ALPHA = 0.2f;
constexpr float INDICE_MINIMO_PERTO = 2.0f / 3.0f;

struct Estimativa {
  float indice = 0.0f;
  uint8_t atividadeMaxima = 0;
  uint8_t sensoresAtivos = 0;
  bool valida = false;
};

inline float normalizar(float valor, float minimo, float maximo) {
  if (valor <= minimo) return 0.0f;
  if (valor >= maximo) return 1.0f;
  return (valor - minimo) / (maximo - minimo);
}

class Filtro {
 public:
  void resetar() { _estimativa = {}; }

  void atualizar(const uint8_t *atividades, size_t quantidade, bool bola) {
    if (!bola) {
      resetar();
      return;
    }

    uint8_t maxima = 0;
    uint8_t ativos = 0;
    for (size_t i = 0; i < quantidade; ++i) {
      if (atividades[i] > maxima) maxima = atividades[i];
      if (atividades[i] > LIMIAR_SENSOR_ATIVO) ++ativos;
    }
    const float indice = 0.5f *
        (normalizar(maxima, ATIVIDADE_LONGE, ATIVIDADE_PERTO) +
         normalizar(ativos, SENSORES_LONGE, SENSORES_PERTO));
    _estimativa.indice = _estimativa.valida
        ? _estimativa.indice + FILTRO_ALPHA * (indice - _estimativa.indice)
        : indice;
    _estimativa.atividadeMaxima = maxima;
    _estimativa.sensoresAtivos = ativos;
    _estimativa.valida = true;
  }

  const Estimativa &estimativa() const { return _estimativa; }

 private:
  Estimativa _estimativa;
};

inline const char *descricao(float indice) {
  return indice < 1.0f / 3.0f ? "LONGE" :
         indice < INDICE_MINIMO_PERTO ? "MEDIA" : "PERTO";
}

inline bool estaPerto(float indice) {
  return indice >= INDICE_MINIMO_PERTO;
}
}  // namespace ProximidadeIR

#endif
