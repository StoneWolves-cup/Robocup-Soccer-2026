#include "../include/BatteryEstimator.h"

// ----------------------------------------------------------------------
// Curva de calibração: tensão (V) -> percentual (%)
// Deve estar em ordem DECRESCENTE de tensão para a interpolação funcionar.
// Meça a sua própria curva com um multímetro + descarga controlada.
// ----------------------------------------------------------------------
static const PontoBateria kCurva[] = {
    {4.20f, 100.0f},
    {4.10f,  90.0f},
    {4.00f,  80.0f},
    {3.90f,  60.0f},
    {3.80f,  40.0f},
    {3.70f,  20.0f},
    {3.60f,  10.0f},
    {3.20f,   0.0f},
};
static const int kCurvaLen = sizeof(kCurva) / sizeof(kCurva[0]);

// Fração da curva de tensão que ainda entra na estimativa combinada,
// mesmo com o Coulomb Counting ativo (ajuda a corrigir deriva/drift).
static const float kBlendVoltageWeightIdle = 1.0f;   // robô parado / corrente baixa
static const float kBlendVoltageWeightLoad = 0.05f;  // robô em movimento / corrente alta
static const float kCurrentIdleThresholdA  = 0.15f;  // abaixo disso, consideramos "parado"

BatteryEstimator::BatteryEstimator()
    : _vRaw(0), _vFiltered(0), _vOpenCircuit(0), _current(0),
      _consumedMah(0), _socVoltage(0), _socCoulomb(100.0f),
      _socCombined(100.0f), _initialized(false) {}

void BatteryEstimator::begin() {
    analogReadResolution(12);
#if HAS_CURRENT_SENSOR
    pinMode(PIN_CURRENT_ADC, INPUT);
#endif
    pinMode(PIN_VBAT_ADC, INPUT);

    // Primeira leitura "crua" define o ponto inicial do filtro, evitando
    // um transiente longo de convergência no início (t=0 já parte da
    // tensão real em vez de 0V).
    _vRaw = readVoltageOnce();
    _vFiltered = _vRaw;
    _vOpenCircuit = _vRaw;
    _socVoltage = voltageToPercent(_vOpenCircuit);
    _socCoulomb = _socVoltage;
    _socCombined = _socVoltage;
    _initialized = true;
}

float BatteryEstimator::readVoltageOnce() {
    // analogReadMilliVolts() já compensa a não-linearidade do ADC do ESP32,
    // é preferível a analogRead() bruto para medições de tensão.
    uint32_t mv = analogReadMilliVolts(PIN_VBAT_ADC);
    float vAdc = mv / 1000.0f;
    float vBat = vAdc * (DIVIDER_R1_OHM + DIVIDER_R2_OHM) / DIVIDER_R2_OHM;
    return vBat;
}

float BatteryEstimator::readCurrentOnce() {
#if HAS_CURRENT_SENSOR
    uint32_t mv = analogReadMilliVolts(PIN_CURRENT_ADC);
    float vSensor = mv / 1000.0f;
    float amps = (vSensor - CURRENT_SENS_OFFSET_V) / CURRENT_SENS_V_PER_A;
    return amps;
#else
    return 0.0f;
#endif
}

// Interpolação linear na tabela de calibração (Fase 3 do projeto)
float BatteryEstimator::voltageToPercent(float voltage) const {
    if (voltage >= kCurva[0].tensao) return 100.0f;
    if (voltage <= kCurva[kCurvaLen - 1].tensao) return 0.0f;

    for (int i = 0; i < kCurvaLen - 1; i++) {
        float vHigh = kCurva[i].tensao;
        float vLow  = kCurva[i + 1].tensao;
        if (voltage <= vHigh && voltage >= vLow) {
            float pHigh = kCurva[i].percentual;
            float pLow  = kCurva[i + 1].percentual;
            float frac = (voltage - vLow) / (vHigh - vLow);
            return pLow + frac * (pHigh - pLow);
        }
    }
    return 0.0f;
}

void BatteryEstimator::updateCoulombCounting(uint32_t dtMillis) {
#if HAS_CURRENT_SENSOR
    // Q_n = Q_{n-1} + I_n * dt   (Coulomb Counting, Fase 5)
    float dtHours = dtMillis / 3600000.0f;
    float mahThisStep = _current * 1000.0f * dtHours;
    _consumedMah += mahThisStep;

    float socDrop = (_consumedMah / BATTERY_CAPACITY_MAH) * 100.0f;
    _socCoulomb = 100.0f - socDrop;
    if (_socCoulomb > 100.0f) _socCoulomb = 100.0f;
    if (_socCoulomb < 0.0f) _socCoulomb = 0.0f;
#else
    (void)dtMillis;
    _socCoulomb = _socVoltage;
#endif
}

void BatteryEstimator::updateCombinedEstimate() {
#if HAS_CURRENT_SENSOR
    float absI = fabsf(_current);
    float wVoltage = (absI < kCurrentIdleThresholdA)
                          ? kBlendVoltageWeightIdle
                          : kBlendVoltageWeightLoad;
    float wCoulomb = 1.0f - wVoltage;
    _socCombined = wVoltage * _socVoltage + wCoulomb * _socCoulomb;

    // Quando o robô está parado (corrente baixa), a leitura de tensão é
    // confiável o suficiente para "recalibrar" o contador de Coulomb e
    // corrigir o drift acumulado da integração numérica.
    if (absI < kCurrentIdleThresholdA) {
        _consumedMah = (100.0f - _socVoltage) / 100.0f * BATTERY_CAPACITY_MAH;
    }
#else
    _socCombined = _socVoltage;
#endif
}

void BatteryEstimator::update(uint32_t dtMillis) {
    if (!_initialized) begin();

    // --- Fase 1: leitura bruta ---
    _vRaw = readVoltageOnce();

    // --- Fase 2: filtro digital passa-baixas (EMA) ---
    // y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
    _vFiltered = FILTER_ALPHA * _vRaw + (1.0f - FILTER_ALPHA) * _vFiltered;

    // --- Corrente e compensação de resistência interna ---
    _current = readCurrentOnce();
    // V_oc = V_terminal + I * Rint  (compensa a queda sob carga)
    _vOpenCircuit = _vFiltered + _current * BATTERY_RINT_OHM;

    // --- Fase 3: tensão -> percentual, via curva de calibração ---
    _socVoltage = voltageToPercent(_vOpenCircuit);

    // --- Fase 5: Coulomb Counting ---
    updateCoulombCounting(dtMillis);

    // --- Fase 6 (versão simples, sem Kalman): estimativa combinada ---
    updateCombinedEstimate();
}

void BatteryEstimator::resetCoulombCounter(float socPercent) {
    if (socPercent < 0.0f) socPercent = 0.0f;
    if (socPercent > 100.0f) socPercent = 100.0f;
    _consumedMah = (100.0f - socPercent) / 100.0f * BATTERY_CAPACITY_MAH;
    _socCoulomb = socPercent;
}