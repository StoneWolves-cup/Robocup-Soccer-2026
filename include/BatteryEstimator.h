#ifndef BATTERY_ESTIMATOR_H
#define BATTERY_ESTIMATOR_H

#include <Arduino.h>
#include <ConfigPinos.h>

#define PIN_VBAT_ADC ConfigPinos::Body::Bateria::VBAT_ADC
#define DIVIDER_R1_OHM 36000.0f
#define DIVIDER_R2_OHM 10000.0f
#define TENSAO_MINIMA 10.5f
#define TENSAO_MAXIMA 12.6f
#define UPDATE_PERIOD_MS 500

class BatteryEstimator {
public:
    BatteryEstimator();

    void begin();
    void update(uint32_t dtMillis);

    float getVoltage() const { return _voltage; }
    float getPercentage() const;
    bool isLow() const { return _isLow; }
    bool hasReading() const { return _hasReading; }

private:
    float _voltage;
    bool _isLow;
    bool _hasReading;
};

#endif
