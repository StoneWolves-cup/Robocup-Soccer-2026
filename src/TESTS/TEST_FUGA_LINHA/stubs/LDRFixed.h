#ifndef TEST_LDR_FIXED_H
#define TEST_LDR_FIXED_H

#include <stdint.h>

class LDRFixed {
 public:
  enum Regiao : uint8_t {
    Centro, Esquerda, Direita, Inferior, Superior, QuantidadeRegioes
  };
  bool regioes[QuantidadeRegioes] = {};
  bool regiaoBranca(Regiao regiao) const { return regioes[regiao]; }
};

#endif
