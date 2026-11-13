#ifndef LDR_FIXED_H
#define LDR_FIXED_H

#include <stdint.h>

class LDRFixed {
public:
    enum Regiao : uint8_t {
        Centro,
        Esquerda,
        Direita,
        Inferior,
        QuantidadeRegioes
    };

    void begin();
    void update();

    void setThreshold(Regiao regiao, uint16_t limiar);
    uint16_t getThreshold(Regiao regiao) const;
    void setThresholds(uint16_t esquerda, uint16_t direita, uint16_t inferior);

    bool regiaoBranca(Regiao regiao) const;
    const bool *regioesBrancas() const;

private:
    static constexpr uint8_t SENSOR_COUNT = 15;

    uint16_t _thresholds[QuantidadeRegioes] = {0, 3332, 1304, 2312};
    bool _regioesBrancas[QuantidadeRegioes] = {};
    bool _sensoresCentroBrancos[SENSOR_COUNT] = {};
    bool _classificacaoInicializada = false;

    uint16_t lerSensor(uint8_t canal);
    bool classificarAcimaDoLimiar(uint16_t leitura, uint16_t limiar,
                                  bool &branco) const;
};

#endif
