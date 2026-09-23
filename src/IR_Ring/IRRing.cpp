#include <IRRing.h>

#include <math.h>
#include <string.h>

IRRing* IRRing::_instance = nullptr;

const IRRing::ISRHandler IRRing::_sensorISRTable[PHYSICAL_SENSOR_COUNT] = {
    IRRing::sensorISR0, IRRing::sensorISR1, IRRing::sensorISR2, IRRing::sensorISR3,
    IRRing::sensorISR4, IRRing::sensorISR5, IRRing::sensorISR6, IRRing::sensorISR7
};

namespace
{
constexpr float IR_TWO_PI = 6.283185307f;
constexpr float SENSOR_ANGLE_STEP = IR_TWO_PI / IRRing::PHYSICAL_SENSOR_COUNT;
}

IRRing::IRRing(const uint8_t* sensorPins)
{
    _instance = this;
    memcpy(_pins, sensorPins, sizeof(_pins));
    _timer = nullptr;
    _taskHandle = nullptr;
    _running = false;
    _windowUs = DEFAULT_WINDOW_US;
    _filterAlpha = DEFAULT_FILTER_ALPHA;
    _detectionThreshold = DEFAULT_DETECTION_THRESHOLD;
    _angleOffset = 0.0f;
    _directionInverted = false;
    _historyIndex = 0;
    _historyCount = 0;
    _filteredX = 0.0f;
    _filteredY = 0.0f;
    _filterInitialized = false;
    _isrCount = 0;
    _processCount = 0;
    clearCounters();
    clearIntensities();
    clearResult();
    resetFilter();
}

bool IRRing::begin()
{
    if (_running) return true;
    clearCounters();
    clearIntensities();
    clearResult();
    resetFilter();
    memset(_history, 0, sizeof(_history));
    _historyIndex = 0;
    _historyCount = 0;

    for (uint8_t i = 0; i < PHYSICAL_SENSOR_COUNT; ++i) {
        pinMode(_pins[i], INPUT);
    }
    if (!configureTimer()) return false;
    attachSensorInterrupts();
    _running = true;
    return true;
}

void IRRing::end()
{
    if (!_running) return;
    detachSensorInterrupts();
    destroyTimer();
    _running = false;
}

bool IRRing::configureTimer()
{
    _timer = timerBegin(0, 80, true);
    if (_timer == nullptr) return false;
    timerAttachInterrupt(_timer, &IRRing::timerISR, true);
    timerAlarmWrite(_timer, _windowUs, true);
    timerAlarmEnable(_timer);
    return true;
}

void IRRing::destroyTimer()
{
    if (_timer != nullptr) {
        timerEnd(_timer);
        _timer = nullptr;
    }
}

void IRAM_ATTR IRRing::timerISR()
{
    IRRing* instance = _instance;
    if (instance == nullptr || instance->_taskHandle == nullptr) return;
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(instance->_taskHandle, &higherPriorityTaskWoken);
    if (higherPriorityTaskWoken) portYIELD_FROM_ISR();
}

void IRRing::attachSensorInterrupts()
{
    for (uint8_t i = 0; i < PHYSICAL_SENSOR_COUNT; ++i) {
        attachInterrupt(digitalPinToInterrupt(_pins[i]), _sensorISRTable[i], FALLING);
    }
}

void IRRing::detachSensorInterrupts()
{
    for (uint8_t i = 0; i < PHYSICAL_SENSOR_COUNT; ++i) {
        detachInterrupt(digitalPinToInterrupt(_pins[i]));
    }
}

void IRAM_ATTR IRRing::handleSensorISR(uint8_t sensor)
{
    IRRing* instance = _instance;
    if (instance == nullptr || sensor >= PHYSICAL_SENSOR_COUNT) return;
    portENTER_CRITICAL_ISR(&instance->_mux);
    ++instance->_pulseCount[sensor];
    ++instance->_isrCount;
    portEXIT_CRITICAL_ISR(&instance->_mux);
}

#define IR_SENSOR_ISR(number) \
    void IRAM_ATTR IRRing::sensorISR##number() { handleSensorISR(number); }
IR_SENSOR_ISR(0)
IR_SENSOR_ISR(1)
IR_SENSOR_ISR(2)
IR_SENSOR_ISR(3)
IR_SENSOR_ISR(4)
IR_SENSOR_ISR(5)
IR_SENSOR_ISR(6)
IR_SENSOR_ISR(7)
#undef IR_SENSOR_ISR

void IRRing::snapshotCounters(uint32_t* destination)
{
    portENTER_CRITICAL(&_mux);
    for (uint8_t i = 0; i < PHYSICAL_SENSOR_COUNT; ++i) {
        destination[i] = _pulseCount[i];
        _pulseCount[i] = 0;
    }
    portEXIT_CRITICAL(&_mux);
}

void IRRing::process()
{
    if (!_running) return;
    ++_processCount;

    uint32_t current[PHYSICAL_SENSOR_COUNT] = {};
    snapshotCounters(current);
    const uint8_t historySlot = _historyIndex;
    _historyIndex = (_historyIndex + 1) % HISTORY_LENGTH;
    if (_historyCount < HISTORY_LENGTH) ++_historyCount;

    uint32_t totalPulses = 0;
    float smoothed[PHYSICAL_SENSOR_COUNT] = {};
    float maxSmoothed = 0.0f;
    uint8_t strongestSensor = 0;

    for (uint8_t i = 0; i < PHYSICAL_SENSOR_COUNT; ++i) {
        _history[i][historySlot] = current[i];
        totalPulses += current[i];
        uint32_t sum = 0;
        for (uint8_t k = 0; k < _historyCount; ++k) sum += _history[i][k];
        smoothed[i] = static_cast<float>(sum) / _historyCount;
        _intensity[i] = static_cast<uint32_t>(smoothed[i] + 0.5f);
        if (smoothed[i] > maxSmoothed) {
            maxSmoothed = smoothed[i];
            strongestSensor = i;
        }
    }

    float x = 0.0f;
    float y = 0.0f;
    float weightSum = 0.0f;
    const float cutoff = maxSmoothed * 0.25f;
    for (uint8_t i = 0; i < PHYSICAL_SENSOR_COUNT; ++i) {
        if (smoothed[i] < cutoff || smoothed[i] <= 0.0f) continue;
        const float weight = smoothed[i] * smoothed[i];
        const float angle = SENSOR_ANGLE_STEP * i;
        x += weight * cosf(angle);
        y += weight * sinf(angle);
        weightSum += weight;
    }

    if (maxSmoothed < _detectionThreshold || weightSum <= 0.0f) {
        publishResult(0.0f, 0.0f, 0.0f, static_cast<uint32_t>(maxSmoothed + 0.5f),
                      strongestSensor, totalPulses, false);
        return;
    }

    if (!_filterInitialized) {
        _filteredX = x;
        _filteredY = y;
        _filterInitialized = true;
    } else {
        _filteredX = _filterAlpha * _filteredX + (1.0f - _filterAlpha) * x;
        _filteredY = _filterAlpha * _filteredY + (1.0f - _filterAlpha) * y;
    }

    float angle = normalizeAngle(atan2f(_filteredY, _filteredX) * 180.0f / PI);
    angle = normalizeAngle(angle + _angleOffset);
    if (_directionInverted) angle = normalizeAngle(360.0f - angle);

    // D_raw da formula da equipe Air: media circular de Smax+d, d=-5..5.
    float draw = 0.0f;
    for (int8_t d = -5; d <= 5; ++d) {
        const uint8_t index = static_cast<uint8_t>(
            (static_cast<int>(strongestSensor) + d + PHYSICAL_SENSOR_COUNT * 2)
            % PHYSICAL_SENSOR_COUNT);
        draw += smoothed[index];
    }
    draw /= HISTORY_LENGTH;

    const float strength = hypotf(_filteredX, _filteredY);
    const float normalizedStrength = strength / (weightSum > 0.0f ? weightSum : 1.0f);
    publishResult(angle, draw, normalizedStrength,
                  static_cast<uint32_t>(maxSmoothed + 0.5f),
                  strongestSensor, totalPulses, true);
}

void IRRing::publishResult(float angle, float strength, float normalizedStrength,
                           uint32_t maxIntensity, uint8_t strongestSensor,
                           uint32_t totalPulses, bool detected)
{
    portENTER_CRITICAL(&_mux);
    _angle = angle;
    _strength = strength;
    _normalizedStrength = normalizedStrength;
    _maxIntensity = maxIntensity;
    _strongestSensor = strongestSensor;
    _totalPulses = totalPulses;
    _detected = detected;
    _virtualSensor16 = false;
    _logicalSensor = strongestSensor;
    portEXIT_CRITICAL(&_mux);
}

float IRRing::normalizeAngle(float angle)
{
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

float IRRing::getAngle() const { return getResult().angle; }
float IRRing::getStrength() const { return getResult().strength; }
float IRRing::getNormalizedStrength() const { return getResult().normalizedStrength; }
bool IRRing::ballDetected() const { return getResult().detected; }
uint8_t IRRing::getStrongestLogicalSensor() const { return getStrongestSensor(); }

uint8_t IRRing::getStrongestSensor() const
{
    portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    const uint8_t value = _strongestSensor + 1;
    portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    return value;
}

bool IRRing::isVirtualSensor16Detected() const { return false; }

uint32_t IRRing::getSensorIntensity(uint8_t sensor) const
{
    if (sensor >= PHYSICAL_SENSOR_COUNT) return 0;
    portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    const uint32_t value = _intensity[sensor];
    portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    return value;
}

void IRRing::copyIntensities(uint32_t* destination, size_t length) const
{
    if (destination == nullptr) return;
    if (length > PHYSICAL_SENSOR_COUNT) length = PHYSICAL_SENSOR_COUNT;
    portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    memcpy(destination, _intensity, length * sizeof(uint32_t));
    portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
}

IRRing::Result IRRing::getResult() const
{
    Result result;
    portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    result.angle = _angle;
    result.strength = _strength;
    result.normalizedStrength = _normalizedStrength;
    result.maxIntensity = _maxIntensity;
    result.strongestSensor = _strongestSensor + 1;
    result.totalPulses = _totalPulses;
    result.detected = _detected;
    result.virtualSensor16 = false;
    result.logicalSensor = _logicalSensor + 1;
    portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    return result;
}

uint32_t IRRing::getISRCount() const { return _isrCount; }
uint32_t IRRing::getProcessCount() const { return _processCount; }

void IRRing::attachTask(TaskHandle_t taskHandle) { _taskHandle = taskHandle; }
TaskHandle_t IRRing::getTaskHandle() const { return _taskHandle; }

void IRRing::setWindowUs(uint32_t windowUs)
{
    if (windowUs == 0) return;
    _windowUs = windowUs;
    if (_timer != nullptr) timerAlarmWrite(_timer, _windowUs, true);
}
uint32_t IRRing::getWindowUs() const { return _windowUs; }
void IRRing::setFilterAlpha(float alpha) { _filterAlpha = constrain(alpha, 0.0f, 1.0f); }
float IRRing::getFilterAlpha() const { return _filterAlpha; }
void IRRing::setDetectionThreshold(uint32_t threshold) { _detectionThreshold = threshold; }
uint32_t IRRing::getDetectionThreshold() const { return _detectionThreshold; }
void IRRing::setAngleOffset(float offset) { _angleOffset = normalizeAngle(offset); }
float IRRing::getAngleOffset() const { return _angleOffset; }
void IRRing::setDirectionInverted(bool inverted) { _directionInverted = inverted; }
bool IRRing::isDirectionInverted() const { return _directionInverted; }

void IRRing::clearCounters()
{
    portENTER_CRITICAL(&_mux);
    memset((void*)_pulseCount, 0, sizeof(_pulseCount));
    _isrCount = 0;
    portEXIT_CRITICAL(&_mux);
}

void IRRing::clearIntensities()
{
    portENTER_CRITICAL(&_mux);
    memset(_intensity, 0, sizeof(_intensity));
    portEXIT_CRITICAL(&_mux);
}

void IRRing::clearResult()
{
    portENTER_CRITICAL(&_mux);
    _angle = 0.0f;
    _strength = 0.0f;
    _normalizedStrength = 0.0f;
    _maxIntensity = 0;
    _strongestSensor = 0;
    _totalPulses = 0;
    _detected = false;
    _virtualSensor16 = false;
    _logicalSensor = 0;
    portEXIT_CRITICAL(&_mux);
}

void IRRing::resetFilter()
{
    _filteredX = 0.0f;
    _filteredY = 0.0f;
    _filterInitialized = false;
}
