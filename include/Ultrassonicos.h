#ifndef ULTRASSONICOS_H
#define ULTRASSONICOS_H

#include <Arduino.h>

// Quatro sensores ultrassonicos com TRIG compartilhado.
class Ultrassonicos {
public:
  static constexpr uint8_t QUANTIDADE = 4;
  static constexpr uint8_t PINO_TRIG = 10;
  static const uint8_t PINOS_ECHO[QUANTIDADE];

  void begin();

  // Dispara os quatro sensores juntos. Leituras invalidas (sem eco ou fora
  // de 2 a 430 cm) retornam false no indice correspondente.
  void ler(float distanciasCm[QUANTIDADE], bool leiturasValidas[QUANTIDADE]);
};

#endif
