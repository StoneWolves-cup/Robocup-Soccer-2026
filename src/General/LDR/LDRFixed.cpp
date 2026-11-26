#include <LDRFixed.h>
#include <Arduino.h>
#include <ConfigPinos.h>

namespace {
constexpr uint8_t SENSOR_COUNT = 15;
constexpr uint8_t CENTER_SENSOR_COUNT = 7;
constexpr uint8_t MIN_CENTER_WHITE_SENSORS = 2;
constexpr uint8_t SAMPLE_COUNT = 12;
constexpr uint8_t DISCARD_COUNT = 2;
constexpr uint16_t HYSTERESIS_MARGIN = 80;

constexpr uint8_t MUX_SIG = ConfigPinos::Body::LDR::MUX_SINAL;
constexpr uint8_t MUX_S0 = ConfigPinos::Body::LDR::MUX_S0;
constexpr uint8_t MUX_S1 = ConfigPinos::Body::LDR::MUX_S1;
constexpr uint8_t MUX_S2 = ConfigPinos::Body::LDR::MUX_S2;
constexpr uint8_t MUX_S3 = ConfigPinos::Body::LDR::MUX_S3;

const uint8_t CHANNELS_CENTER[CENTER_SENSOR_COUNT] = {
    1, 2, 3, 4, 5, 6, 7};
// A linha Centro e ignorada (usa CHANNELS_CENTER). Superior e a frente: C14 e
// C0, este emprestado do anel central; se o sensor frontal fisico for outro,
// troque-o aqui e em CHANNELS_CENTER/ConfigLDR.
const uint8_t CHANNELS_REGIONS[LDRFixed::QuantidadeRegioes][2] = {
    {0, 0}, {12, 13}, {8, 9}, {10, 11}, {14, 0}};
const uint8_t REGION_SENSOR_COUNTS[LDRFixed::QuantidadeRegioes] = {
    7, 2, 2, 2, 2};

// Atualiza os bits do multiplexador e aguarda assentamento eletrico do canal
// antes do ADC, evitando que a conversao ainda reflita o canal anterior.
void selectMuxChannel(uint8_t channel) {
    digitalWrite(MUX_S0, channel & 0x01);
    digitalWrite(MUX_S1, (channel >> 1) & 0x01);
    digitalWrite(MUX_S2, (channel >> 2) & 0x01);
    digitalWrite(MUX_S3, (channel >> 3) & 0x01);
    delayMicroseconds(10);
}
}  // namespace

LDRFixed::LDRFixed(const ConfigLDR::Limiares &limiares) {
    setThresholds(limiares.esquerda, limiares.direita, limiares.inferior,
                  limiares.superior);
    setCenterSensorThresholds(limiares.centro, limiares.histereseCentro);
}

// Inicializa o ADC na resolucao e faixa usadas pelos limiares calibrados.
void LDRFixed::begin() {
    pinMode(MUX_SIG, INPUT);
    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT);
    pinMode(MUX_S3, OUTPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(MUX_SIG, ADC_11db);
}

// Evita escrever fora do vetor quando recebe um enum invalido.
void LDRFixed::setThreshold(Regiao regiao, uint16_t limiar) {
    if (regiao >= Centro && regiao < QuantidadeRegioes) {
        _thresholds[regiao] = limiar;
    }
}

// Retorna zero para enum invalido para manter a leitura segura.
uint16_t LDRFixed::getThreshold(Regiao regiao) const {
    if (regiao >= Centro && regiao < QuantidadeRegioes) {
        return _thresholds[regiao];
    }
    return 0;
}

// Retorna zero para enum invalido, sem acessar fora do vetor de medias.
uint16_t LDRFixed::getAverage(Regiao regiao) const {
    if (regiao >= Centro && regiao < QuantidadeRegioes) {
        return _averages[regiao];
    }
    return 0;
}

uint16_t LDRFixed::getCenterSensorReading(uint8_t index) const {
    return index < CENTER_SENSOR_COUNT ? _centerSensorReadings[index] : 0;
}

uint16_t LDRFixed::getCenterSensorThreshold(uint8_t index) const {
    return index < CENTER_SENSOR_COUNT ? _centerSensorThresholds[index] : 0;
}

// Atualiza somente as regioes perifericas para preservar configuracao do centro.
void LDRFixed::setThresholds(uint16_t esquerda, uint16_t direita,
                            uint16_t inferior, uint16_t superior) {
    _thresholds[Esquerda] = esquerda;
    _thresholds[Direita] = direita;
    _thresholds[Inferior] = inferior;
    _thresholds[Superior] = superior;
}

// Copia valores para o objeto, evitando depender do tempo de vida do array
// fornecido pela configuracao de perfil.
void LDRFixed::setCenterSensorThresholds(const uint16_t limiares[7],
                                         uint16_t margemHisterese) {
    for (uint8_t i = 0; i < CENTER_SENSOR_COUNT; ++i) {
        _centerSensorThresholds[i] = limiares[i];
    }
    _centerHysteresisMargin = margemHisterese;
}

// Ao trocar perfil, a primeira amostra deve ser classificada sem herdar o
// estado branco/verde anterior, que foi obtido com outros limiares.
void LDRFixed::resetClassification() {
    _classificacaoInicializada = false;
}

// Faz varias conversoes por canal; as primeiras sao descartadas por causa do
// tempo de assentamento do multiplexador e as restantes sao mediadas.
uint16_t LDRFixed::lerSensor(uint8_t canal) {
    selectMuxChannel(canal);
    for (uint8_t i = 0; i < DISCARD_COUNT; ++i) {
        analogRead(MUX_SIG);
    }

    uint32_t total = 0;
    for (uint8_t i = 0; i < SAMPLE_COUNT; ++i) {
        total += analogRead(MUX_SIG);
    }
    return total / SAMPLE_COUNT;
}

// Mantem o estado anterior dentro da faixa morta; a margem impede alternancias
// rapidas causadas por ruido quando a leitura fica junto ao limiar.
bool LDRFixed::classificarLimiar(uint16_t leitura, uint16_t limiar,
                                bool &branco, uint16_t margemHisterese,
                                bool brancoAcimaDoLimiar) const {
    if (brancoAcimaDoLimiar) {
        if (static_cast<uint32_t>(leitura) >
            static_cast<uint32_t>(limiar) + margemHisterese) {
            branco = true;
        } else if (static_cast<uint32_t>(leitura) + margemHisterese < limiar) {
            branco = false;
        }
    } else {
        if (static_cast<uint32_t>(leitura) + margemHisterese < limiar) {
            branco = true;
        } else if (static_cast<uint32_t>(leitura) >
                   static_cast<uint32_t>(limiar) + margemHisterese) {
            branco = false;
        }
    }
    return branco;
}

// Atualiza o conjunto completo como uma unica rodada: centro e periferia usam
// medias/limiares proprios para tolerar variacao entre sensores e placas.
void LDRFixed::update() {
    uint16_t leituras[SENSOR_COUNT];
    for (uint8_t sensor = 0; sensor < SENSOR_COUNT; ++sensor) {
        leituras[sensor] = lerSensor(sensor);
    }

    uint8_t sensoresCentroBrancos = 0;
    for (uint8_t i = 0; i < CENTER_SENSOR_COUNT; ++i) {
        const uint8_t canal = CHANNELS_CENTER[i];
        _centerSensorReadings[i] = leituras[canal];
        if (!_classificacaoInicializada) {
            _sensoresCentroBrancos[canal] =
                leituras[canal] > _centerSensorThresholds[i];
        }
        if (classificarLimiar(
                leituras[canal], _centerSensorThresholds[i],
                _sensoresCentroBrancos[canal], _centerHysteresisMargin,
                true)) {
            ++sensoresCentroBrancos;
        }
    }
    _regioesBrancas[Centro] =
        sensoresCentroBrancos >= MIN_CENTER_WHITE_SENSORS;

    for (uint8_t regiao = Esquerda; regiao < QuantidadeRegioes; ++regiao) {
        uint32_t total = 0;
        for (uint8_t i = 0; i < REGION_SENSOR_COUNTS[regiao]; ++i) {
            total += leituras[CHANNELS_REGIONS[regiao][i]];
        }
        const uint16_t media = total / REGION_SENSOR_COUNTS[regiao];
        _averages[regiao] = media;
        _regioesBrancas[regiao] = media > _thresholds[regiao];
    }
    _classificacaoInicializada = true;
}

// Consulta apenas o ultimo resultado, deixando o controle de amostragem ao loop.
bool LDRFixed::regiaoBranca(Regiao regiao) const {
    return _regioesBrancas[regiao];
}

// Expõe o snapshot para telemetria sem copiar o vetor; o chamador nao deve
// guardar o ponteiro atraves de uma proxima atualizacao.
const bool *LDRFixed::regioesBrancas() const {
    return _regioesBrancas;
}
