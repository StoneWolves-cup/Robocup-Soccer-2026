#ifndef LDR_FIXED_H
#define LDR_FIXED_H

#include <stdint.h>
#include <ConfigLDR.h>

class LDRFixed {
public:
    // As regioes perifericas usam media de pares; Centro agrega nove canais.
    enum Regiao : uint8_t {
        Centro,
        Esquerda,
        Direita,
        Inferior,
        QuantidadeRegioes
    };

    // Recebe um perfil inicial para que ataque e defesa possam calibrar Centro
    // e periferia independentemente.
    explicit LDRFixed(const ConfigLDR::Limiares &limiares = ConfigLDR::Ataque);
    // Configura ADC e selecao do multiplexador.
    void begin();
    // Le os canais, filtra ruido e atualiza classificacoes com histerese.
    void update();

    // Ajusta um limiar periférico individual; valores invalidos sao ignorados.
    void setThreshold(Regiao regiao, uint16_t limiar);
    uint16_t getThreshold(Regiao regiao) const;
    // Retorna a media mais recente usada para classificar a regiao.
    uint16_t getAverage(Regiao regiao) const;
    // Ajusta os tres limiares editaveis pela IHM.
    void setThresholds(uint16_t esquerda, uint16_t direita, uint16_t inferior);
    // Ajusta limites individuais do anel central e sua margem de histerese.
    void setCenterSensorThresholds(const uint16_t limiares[9],
                                   uint16_t margemHisterese);
    // Forca a proxima amostra a inicializar o estado com os novos limiares.
    void resetClassification();

    // Consulta a classificacao mais recente, sem executar uma nova leitura.
    bool regiaoBranca(Regiao regiao) const;
    // Retorna o vetor de estados na ordem do enum Regiao.
    const bool *regioesBrancas() const;

private:
    static constexpr uint8_t SENSOR_COUNT = 15;

    uint16_t _thresholds[QuantidadeRegioes] = {};
    uint16_t _averages[QuantidadeRegioes] = {};
    uint16_t _centerSensorThresholds[9] = {};
    uint16_t _centerHysteresisMargin = 0;
    bool _regioesBrancas[QuantidadeRegioes] = {};
    bool _sensoresCentroBrancos[SENSOR_COUNT] = {};
    bool _classificacaoInicializada = false;

    // Descarta conversoes apos trocar canal e faz media para reduzir ruido ADC.
    uint16_t lerSensor(uint8_t canal);
    // Aplica histerese ao estado anterior para evitar oscilacao no limite.
    bool classificarLimiar(uint16_t leitura, uint16_t limiar, bool &branco,
                           uint16_t margemHisterese,
                           bool brancoAcimaDoLimiar) const;
};

#endif
