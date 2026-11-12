#include <TSSP.h>
#include <ConfigPinos.h>

// Mapa copiado do teste TSSP de 16 sensores. Adeque a placa/cabos conforme
// o pinout real do ESP32-S3 usado no modulo de sensores.
const uint8_t TSSP::PINOS[TSSP::NUM_SENSORES] = {
  ConfigPinos::Head::TSSP::PINOS[0],
  ConfigPinos::Head::TSSP::PINOS[1],
  ConfigPinos::Head::TSSP::PINOS[2],
  ConfigPinos::Head::TSSP::PINOS[3],
  ConfigPinos::Head::TSSP::PINOS[4],
  ConfigPinos::Head::TSSP::PINOS[5],
  ConfigPinos::Head::TSSP::PINOS[6],
  ConfigPinos::Head::TSSP::PINOS[7],
  ConfigPinos::Head::TSSP::PINOS[8],
  ConfigPinos::Head::TSSP::PINOS[9],
  ConfigPinos::Head::TSSP::PINOS[10],
  ConfigPinos::Head::TSSP::PINOS[11],
  ConfigPinos::Head::TSSP::PINOS[12],
  ConfigPinos::Head::TSSP::PINOS[13],
  ConfigPinos::Head::TSSP::PINOS[14],
  ConfigPinos::Head::TSSP::PINOS[15]
};

TSSP::TSSP() : totalAmostras_(0), inicioJanela_(0) {
  for (uint8_t i = 0; i < NUM_SENSORES; ++i) {
    lowCount_[i] = 0;
    atividade_[i] = 0;
  }
}

void TSSP::begin() {
  for (uint8_t i = 0; i < NUM_SENSORES; ++i) {
    // GPIO 34..39 em algumas variantes ESP32 sao somente entrada e nao
    // possuem pull-up interno. INPUT evita presumir pull-up nesses pinos.
    pinMode(PINOS[i], (PINOS[i] >= 34 && PINOS[i] <= 39) ? INPUT : INPUT_PULLUP);
  }
  inicioJanela_ = micros();
}

bool TSSP::atualizar() {
  for (uint8_t i = 0; i < NUM_SENSORES; ++i) {
    if (digitalRead(PINOS[i]) == LOW) ++lowCount_[i];
  }
  ++totalAmostras_;

  const uint32_t agora = micros();
  if ((uint32_t)(agora - inicioJanela_) < JANELA_US) return false;
  fecharJanela();
  inicioJanela_ = agora;
  return true;
}

void TSSP::fecharJanela() {
  if (totalAmostras_ == 0) return;
  for (uint8_t i = 0; i < NUM_SENSORES; ++i) {
    const float percentual = 100.0f * lowCount_[i] / totalAmostras_;
    atividade_[i] = (uint8_t)(percentual + 0.5f);
    lowCount_[i] = 0;
  }
  totalAmostras_ = 0;
}

uint8_t TSSP::atividadePercentual(uint8_t indice) const {
  return (indice < NUM_SENSORES) ? atividade_[indice] : 0;
}
