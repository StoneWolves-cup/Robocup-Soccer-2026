#ifndef ULTRASSONICOS_H
#define ULTRASSONICOS_H

#include <Arduino.h>
#include <ConfigPinos.h>

// Quatro sensores ultrassonicos com TRIG compartilhado.
class Ultrassonicos {
public:
  static constexpr uint8_t QUANTIDADE = 4;
  static constexpr uint8_t PINO_TRIG = ConfigPinos::Teste::Ultrassonicos::TRIG;
  static const uint8_t PINOS_ECHO[QUANTIDADE];

  // Configura o TRIG compartilhado e instala captura de bordas nos ECHO.
  void begin();

  // Dispara os quatro sensores juntos. Leituras invalidas (sem eco ou fora
  // de 2 a 430 cm) retornam false no indice correspondente.
  // Dispara o pulso e espera os ecos ate o timeout. A mascara de validade
  // evita tratar ausencia de eco como distancia segura.
  void ler(float distanciasCm[QUANTIDADE], bool leiturasValidas[QUANTIDADE]);
};

#endif
