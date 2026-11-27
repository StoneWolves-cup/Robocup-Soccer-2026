#ifndef KICKER_H
#define KICKER_H

#include <stdint.h>

namespace Kicker {
enum class ModoAutomatico {
  Desativado,
  ComUltrassom,
  AtaqueUltrassomDireito,
  Defesa,
  FimDeCurso
};

void begin();
void atualizar(ModoAutomatico modo, bool comunicacaoValida,
               bool bateriaValida, float bateriaPercentual,
               bool ultraValido, const float distancias[4],
               uint8_t mascaraUltra, bool bolaPerto);
bool dispararManual(bool direita);
}  // namespace Kicker

#endif
