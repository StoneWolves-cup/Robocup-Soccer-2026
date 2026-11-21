#ifndef KICKER_H
#define KICKER_H

#include <stdint.h>

namespace Kicker {
void begin();
void atualizar(bool estrategiaAtiva, bool comunicacaoValida,
               bool bateriaValida, float bateriaPercentual,
               bool ultraValido, const float distancias[4],
               uint8_t mascaraUltra);
bool dispararManual(bool direita);
}  // namespace Kicker

#endif
