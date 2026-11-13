#include <LDRFixed.h>
#include <Arduino.h>
#include <ConfigPinos.h>

namespace {
constexpr uint8_t SENSOR_COUNT = 15;
constexpr uint8_t CENTER_SENSOR_COUNT = 9;
constexpr uint8_t SAMPLE_COUNT = 12;
constexpr uint8_t DISCARD_COUNT = 2;
constexpr uint16_t HYSTERESIS_MARGIN = 80;

constexpr uint8_t MUX_SIG = ConfigPinos::Body::LDR::MUX_SINAL;
constexpr uint8_t MUX_S0 = ConfigPinos::Body::LDR::MUX_S0;
constexpr uint8_t MUX_S1 = ConfigPinos::Body::LDR::MUX_S1;
constexpr uint8_t MUX_S2 = ConfigPinos::Body::LDR::MUX_S2;
constexpr uint8_t MUX_S3 = ConfigPinos::Body::LDR::MUX_S3;

const uint8_t CHANNELS_CENTER[CENTER_SENSOR_COUNT] = {
    0, 1, 2, 3, 4, 5, 6, 7, 14};
const uint8_t CHANNELS_REGIONS[LDRFixed::QuantidadeRegioes][2] = {
    {0, 1}, {12, 13}, {8, 9}, {10, 11}};
const uint8_t REGION_SENSOR_COUNTS[LDRFixed::QuantidadeRegioes] = {
    9, 2, 2, 2};
const uint16_t CENTER_THRESHOLDS[CENTER_SENSOR_COUNT] = {
    2619, 2890, 3699, 1936, 2045, 1776, 2924, 1874, 3647};

void selectMuxChannel(uint8_t channel) {
    digitalWrite(MUX_S0, channel & 0x01);
    digitalWrite(MUX_S1, (channel >> 1) & 0x01);
    digitalWrite(MUX_S2, (channel >> 2) & 0x01);
    digitalWrite(MUX_S3, (channel >> 3) & 0x01);
    delayMicroseconds(10);
}
}  // namespace

void LDRFixed::begin() {
    pinMode(MUX_SIG, INPUT);
    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT);
    pinMode(MUX_S3, OUTPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(MUX_SIG, ADC_11db);
}

void LDRFixed::setThreshold(Regiao regiao, uint16_t limiar) {
    if (regiao >= Centro && regiao < QuantidadeRegioes) {
        _thresholds[regiao] = limiar;
    }
}

uint16_t LDRFixed::getThreshold(Regiao regiao) const {
    if (regiao >= Centro && regiao < QuantidadeRegioes) {
        return _thresholds[regiao];
    }
    return 0;
}

void LDRFixed::setThresholds(uint16_t esquerda, uint16_t direita,
                            uint16_t inferior) {
    _thresholds[Esquerda] = esquerda;
    _thresholds[Direita] = direita;
    _thresholds[Inferior] = inferior;
}

uint16_t LDRFixed::lerSensor(uint8_t canal) {
    selectMuxChannel(canal);
    for (uint8_t i = 0; i < DISCARD_COUNT; ++i) {
        analogRead(MUX_SIG);
    }

    uint32_t total = 0;
    for (uint8_t i = 0; i < SAMPLE_COUNT; ++i) {
        total += analogRead(MUX_SIG);
        delayMicroseconds(100);
    }
    return total / SAMPLE_COUNT;
}

bool LDRFixed::classificarAcimaDoLimiar(uint16_t leitura, uint16_t limiar,
                                        bool &branco) const {
    if (static_cast<uint32_t>(leitura) >
        static_cast<uint32_t>(limiar) + HYSTERESIS_MARGIN) {
        branco = true;
    } else if (static_cast<uint32_t>(leitura) + HYSTERESIS_MARGIN < limiar) {
        branco = false;
    }
    return branco;
}

void LDRFixed::update() {
    uint16_t leituras[SENSOR_COUNT];
    for (uint8_t sensor = 0; sensor < SENSOR_COUNT; ++sensor) {
        leituras[sensor] = lerSensor(sensor);
    }

    uint8_t sensoresCentroBrancos = 0;
    for (uint8_t i = 0; i < CENTER_SENSOR_COUNT; ++i) {
        const uint8_t canal = CHANNELS_CENTER[i];
        if (!_classificacaoInicializada) {
            _sensoresCentroBrancos[canal] =
                leituras[canal] > CENTER_THRESHOLDS[i];
        }
        if (classificarAcimaDoLimiar(
                leituras[canal], CENTER_THRESHOLDS[i],
                _sensoresCentroBrancos[canal])) {
            ++sensoresCentroBrancos;
        }
    }
    _regioesBrancas[Centro] = sensoresCentroBrancos >= 2;

    for (uint8_t regiao = Esquerda; regiao < QuantidadeRegioes; ++regiao) {
        uint32_t total = 0;
        for (uint8_t i = 0; i < REGION_SENSOR_COUNTS[regiao]; ++i) {
            total += leituras[CHANNELS_REGIONS[regiao][i]];
        }
        const uint16_t media = total / REGION_SENSOR_COUNTS[regiao];
        if (!_classificacaoInicializada) {
            _regioesBrancas[regiao] = media > _thresholds[regiao];
        }
        classificarAcimaDoLimiar(
            media, _thresholds[regiao], _regioesBrancas[regiao]);
    }
    _classificacaoInicializada = true;
}

bool LDRFixed::regiaoBranca(Regiao regiao) const {
    return _regioesBrancas[regiao];
}

const bool *LDRFixed::regioesBrancas() const {
    return _regioesBrancas;
}
