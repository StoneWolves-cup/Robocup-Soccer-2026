#ifndef IRRING_H
#define IRRING_H

#include <Arduino.h>

// Processamento do anel de 8 receptores TSSP4038.
// C_i: pulsos medidos na janela atual.
// S_i: media movel dos ultimos 12 valores C_i.
// O vetor usa w_i = S_i^2 quando S_i >= 0,25 * Smax.
class IRRing
{
public:
    static constexpr uint8_t PHYSICAL_SENSOR_COUNT = 8;
    static constexpr uint8_t LOGICAL_SENSOR_COUNT = 8;
    static constexpr uint8_t HISTORY_LENGTH = 12;

    static constexpr uint32_t DEFAULT_WINDOW_US = 5000;
    static constexpr uint32_t DEFAULT_DETECTION_THRESHOLD = 10;
    static constexpr float DEFAULT_FILTER_ALPHA = 0.65f;

    using ISRHandler = void (*)();

    struct Result
    {
        float angle;
        float strength;
        float normalizedStrength;
        uint32_t maxIntensity;
        uint8_t strongestSensor;
        uint32_t totalPulses;
        bool detected;
        bool virtualSensor16;
        uint8_t logicalSensor;
    };

    explicit IRRing(const uint8_t* sensorPins);

    bool begin();
    void end();
    void process();

    void setWindowUs(uint32_t windowUs);
    uint32_t getWindowUs() const;
    void setFilterAlpha(float alpha);
    float getFilterAlpha() const;
    void setDetectionThreshold(uint32_t threshold);
    uint32_t getDetectionThreshold() const;
    void setAngleOffset(float offset);
    float getAngleOffset() const;
    void setDirectionInverted(bool inverted);
    bool isDirectionInverted() const;

    float getAngle() const;
    float getStrength() const;
    float getNormalizedStrength() const;
    bool ballDetected() const;
    uint8_t getStrongestLogicalSensor() const;
    uint8_t getStrongestSensor() const;
    bool isVirtualSensor16Detected() const;
    uint32_t getSensorIntensity(uint8_t sensor) const;
    void copyIntensities(uint32_t* destination, size_t length) const;
    Result getResult() const;

    uint32_t getISRCount() const;
    uint32_t getProcessCount() const;
    void attachTask(TaskHandle_t taskHandle);
    TaskHandle_t getTaskHandle() const;

private:
    static IRRing* _instance;
    uint8_t _pins[PHYSICAL_SENSOR_COUNT];
    volatile uint32_t _pulseCount[PHYSICAL_SENSOR_COUNT];
    uint32_t _intensity[PHYSICAL_SENSOR_COUNT];
    uint32_t _history[PHYSICAL_SENSOR_COUNT][HISTORY_LENGTH];
    uint8_t _historyIndex;
    uint8_t _historyCount;

    volatile float _angle;
    volatile float _strength;
    volatile float _normalizedStrength;
    volatile uint32_t _maxIntensity;
    volatile uint8_t _strongestSensor;
    volatile uint32_t _totalPulses;
    volatile bool _detected;
    volatile bool _virtualSensor16;
    volatile uint8_t _logicalSensor;

    float _filteredX;
    float _filteredY;
    bool _filterInitialized;
    uint32_t _windowUs;
    float _filterAlpha;
    uint32_t _detectionThreshold;
    float _angleOffset;
    bool _directionInverted;
    hw_timer_t* _timer;
    TaskHandle_t _taskHandle;
    bool _running;
    volatile uint32_t _isrCount;
    uint32_t _processCount;
    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

    static const ISRHandler _sensorISRTable[PHYSICAL_SENSOR_COUNT];
    static void IRAM_ATTR timerISR();
    static void IRAM_ATTR sensorISR0();
    static void IRAM_ATTR sensorISR1();
    static void IRAM_ATTR sensorISR2();
    static void IRAM_ATTR sensorISR3();
    static void IRAM_ATTR sensorISR4();
    static void IRAM_ATTR sensorISR5();
    static void IRAM_ATTR sensorISR6();
    static void IRAM_ATTR sensorISR7();
    static void IRAM_ATTR handleSensorISR(uint8_t sensor);

    void attachSensorInterrupts();
    void detachSensorInterrupts();
    bool configureTimer();
    void destroyTimer();
    void snapshotCounters(uint32_t* destination);
    void publishResult(float angle, float strength, float normalizedStrength,
                       uint32_t maxIntensity, uint8_t strongestSensor,
                       uint32_t totalPulses, bool detected);
    static float normalizeAngle(float angle);
    void clearCounters();
    void clearIntensities();
    void clearResult();
    void resetFilter();
};

#endif
