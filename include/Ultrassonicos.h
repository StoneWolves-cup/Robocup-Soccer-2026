#ifndef ULTRASSONICOS_H
#define ULTRASSONICOS_H

#include <Arduino.h>
#include <ConfigPinos.h>

class Ultrassonicos {
public:
  static constexpr uint8_t QUANTIDADE = 4;
  static constexpr uint8_t PINO_TRIG =
      ConfigPinos::Head::Ultrassonicos::TRIG;
  static const uint8_t PINOS_ECHO[QUANTIDADE];

  void begin();
  void iniciarLeitura();
  bool atualizar(float distanciasCm[QUANTIDADE],
                 bool leiturasValidas[QUANTIDADE]);
  void ler(float distanciasCm[QUANTIDADE],
           bool leiturasValidas[QUANTIDADE]);
};

#endif
