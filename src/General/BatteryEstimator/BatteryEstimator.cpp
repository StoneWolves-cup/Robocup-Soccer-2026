#include <BatteryEstimator.h>

BatteryEstimator::BatteryEstimator()
    : _voltage(0.0f), _isLow(false), _hasReading(false) {}

void BatteryEstimator::begin() {
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_VBAT_ADC, ADC_11db);
}

float BatteryEstimator::getPercentage() const {
    if (_voltage <= TENSAO_MINIMA) return 0.0f;
    if (_voltage >= TENSAO_MAXIMA) return 100.0f;

    return (_voltage - TENSAO_MINIMA) * 100.0f /
           (TENSAO_MAXIMA - TENSAO_MINIMA);
}

void BatteryEstimator::update(uint32_t dtMillis) {
    (void)dtMillis;

    int leitura_mV = analogReadMilliVolts(PIN_VBAT_ADC);
    float tensaoPino = leitura_mV / 1000.0f;
    _voltage = tensaoPino * (DIVIDER_R1_OHM + DIVIDER_R2_OHM) / DIVIDER_R2_OHM;
    _isLow = _voltage <= TENSAO_MINIMA;
    _hasReading = true;

    Serial.print("Tensao da bateria: ");
    Serial.print(_voltage, 2);
    Serial.println(" V");
    Serial.print("Bateria: ");
    Serial.print(getPercentage(), 0);
    Serial.println("%");

    if (_isLow) {
        Serial.println("ALERTA: BATERIA BAIXA!");
    } else {
        Serial.println("Bateria OK");
    }

    Serial.println("----------------------");
}
