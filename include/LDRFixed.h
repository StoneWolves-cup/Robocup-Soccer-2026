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

    bool regiaoBranca(Regiao regiao) const;
    const bool *regioesBrancas() const;

private:
    static constexpr uint8_t SENSOR_COUNT = 15;

    bool _regioesBrancas[QuantidadeRegioes] = {};
    bool _sensoresCentroBrancos[SENSOR_COUNT] = {};
    bool _classificacaoInicializada = false;

    uint16_t lerSensor(uint8_t canal);
    bool classificarAcimaDoLimiar(uint16_t leitura, uint16_t limiar,
                                  bool &branco) const;
};

#endif
