#ifndef TSSP_H
#define TSSP_H

#include <Arduino.h>

// Driver de aquisicao: mede a atividade LOW dos 16 receptores TSSP4038.
// A interpretacao direcional acontece no ESP32-S3 de processamento.
class TSSP {
public:
  static constexpr uint8_t NUM_SENSORES = 16;
  static const uint8_t PINOS[NUM_SENSORES];

  TSSP();
  void begin();

  // Amostra continuamente. Retorna true ao encerrar uma janela de 5 ms.
  bool atualizar();

  // Atividade percentual LOW de cada sensor, na ordem do anel.
  uint8_t atividadePercentual(uint8_t indice) const;

private:
  static constexpr uint32_t JANELA_US = 5000;
  uint32_t lowCount_[NUM_SENSORES];
  uint8_t atividade_[NUM_SENSORES];
  uint32_t totalAmostras_;
  uint32_t inicioJanela_;
  void fecharJanela();
};

#endif
