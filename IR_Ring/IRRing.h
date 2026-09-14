#ifndef IRRING_H
#define IRRING_H

#include <Arduino.h>

// ============================================================
// IRRing
// Coroa de 15 sensores TSOP2240
//
// Geometria lógica:
//   15 sensores físicos
//   1 posição virtual (posição 16)
//
// Posição virtual 16:
//   337,5 graus
//   localizada entre o sensor 15 e o sensor 1
//
// ESP32 DevKit V1
// ESP32 Arduino Core 3.x
// ============================================================

class IRRing
{
public:

    // ========================================================
    // Quantidade de sensores
    // ========================================================

    static constexpr uint8_t PHYSICAL_SENSOR_COUNT = 15;
    static constexpr uint8_t LOGICAL_SENSOR_COUNT  = 16;


    // ========================================================
    // Configurações padrão
    // ========================================================

    static constexpr uint32_t DEFAULT_WINDOW_US = 5000;

    static constexpr uint32_t DEFAULT_DETECTION_THRESHOLD = 10;

    static constexpr float DEFAULT_FILTER_ALPHA = 0.65f;

    static constexpr float VIRTUAL_SENSOR_16_ANGLE = 337.5f;


    // ========================================================
    // Tipo das funções ISR
    // ========================================================

    using ISRHandler = void (*)();


    // ========================================================
    // Resultado da leitura
    // ========================================================

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


    // ========================================================
    // Construtor
    // ========================================================

    explicit IRRing(
        const uint8_t* sensorPins
    );


    // ========================================================
    // Inicialização
    // ========================================================

    bool begin();

    void end();


    // ========================================================
    // Processamento
    //
    // Deve ser chamado pela FreeRTOS Task.
    // ========================================================

    void process();


    // ========================================================
    // Configuração
    // ========================================================

    void setWindowUs(
        uint32_t windowUs
    );

    uint32_t getWindowUs() const;


    void setFilterAlpha(
        float alpha
    );

    float getFilterAlpha() const;


    void setDetectionThreshold(
        uint32_t threshold
    );

    uint32_t getDetectionThreshold() const;


    void setAngleOffset(
        float offset
    );

    float getAngleOffset() const;


    void setDirectionInverted(
        bool inverted
    );

    bool isDirectionInverted() const;


    // ========================================================
    // Resultado
    // ========================================================

    float getAngle() const;

    float getStrength() const;

    float getNormalizedStrength() const;

    bool ballDetected() const;


    // ========================================================
    // Sensor mais forte
    //
    // Retorna:
    //
    // 1  -> sensor físico 1
    // 2  -> sensor físico 2
    // ...
    // 15 -> sensor físico 15
    // 16 -> posição virtual
    // ========================================================

    uint8_t getStrongestLogicalSensor() const;


    // ========================================================
    // Sensor físico mais forte
    //
    // Retorna:
    //
    // 1..15
    // ========================================================

    uint8_t getStrongestSensor() const;


    // ========================================================
    // Posição virtual 16
    //
    // A posição 16 fica entre os sensores 15 e 1.
    // ========================================================

    bool isVirtualSensor16Detected() const;


    // ========================================================
    // Intensidade individual
    //
    // sensor:
    //   0 -> sensor físico 1
    //   ...
    //   14 -> sensor físico 15
    // ========================================================

    uint32_t getSensorIntensity(
        uint8_t sensor
    ) const;


    // ========================================================
    // Copia as intensidades dos sensores
    // ========================================================

    void copyIntensities(
        uint32_t* destination,
        size_t length
    ) const;


    // ========================================================
    // Resultado completo
    // ========================================================

    Result getResult() const;


    // ========================================================
    // Estatísticas
    // ========================================================

    uint32_t getISRCount() const;

    uint32_t getProcessCount() const;


    // ========================================================
    // Controle da Task
    // ========================================================

    void attachTask(
        TaskHandle_t taskHandle
    );


    TaskHandle_t getTaskHandle() const;


private:

    // ========================================================
    // Ponteiro para a instância
    //
    // Necessário porque as ISR do ESP32 são funções estáticas.
    // ========================================================

    static IRRing* _instance;


    // ========================================================
    // Pinos dos sensores físicos
    // ========================================================

    uint8_t _pins[
        PHYSICAL_SENSOR_COUNT
    ];


    // ========================================================
    // Contadores de pulsos
    //
    // Modificados pelas ISR.
    // ========================================================

    volatile uint32_t _pulseCount[
        PHYSICAL_SENSOR_COUNT
    ];


    // ========================================================
    // Intensidade capturada durante a última janela
    // ========================================================

    uint32_t _intensity[
        PHYSICAL_SENSOR_COUNT
    ];


    // ========================================================
    // Resultado atual
    // ========================================================

    volatile float _angle;

    volatile float _strength;

    volatile float _normalizedStrength;

    volatile uint32_t _maxIntensity;

    volatile uint8_t _strongestSensor;

    volatile uint32_t _totalPulses;

    volatile bool _detected;

    volatile bool _virtualSensor16;

    volatile uint8_t _logicalSensor;


    // ========================================================
    // Filtro vetorial
    // ========================================================

    float _filteredX;

    float _filteredY;

    bool _filterInitialized;


    // ========================================================
    // Configurações
    // ========================================================

    uint32_t _windowUs;

    float _filterAlpha;

    uint32_t _detectionThreshold;

    float _angleOffset;

    bool _directionInverted;


    // ========================================================
    // Timer de hardware
    // ========================================================

    hw_timer_t* _timer;


    // ========================================================
    // FreeRTOS
    // ========================================================

    TaskHandle_t _taskHandle;


    // ========================================================
    // Estado
    // ========================================================

    bool _running;


    // ========================================================
    // Estatísticas
    // ========================================================

    volatile uint32_t _isrCount;

    uint32_t _processCount;


    // ========================================================
    // Critical Section
    // ========================================================

    portMUX_TYPE _mux =
        portMUX_INITIALIZER_UNLOCKED;


    // ========================================================
    // Tabela das ISR
    //
    // Será definida no IRRing.cpp.
    // ========================================================

    static const ISRHandler
        _sensorISRTable[
            PHYSICAL_SENSOR_COUNT
        ];


    // ========================================================
    // Timer ISR
    // ========================================================

    static void IRAM_ATTR timerISR();


    // ========================================================
    // ISR dos sensores
    // ========================================================

    static void IRAM_ATTR sensorISR0();

    static void IRAM_ATTR sensorISR1();

    static void IRAM_ATTR sensorISR2();

    static void IRAM_ATTR sensorISR3();

    static void IRAM_ATTR sensorISR4();

    static void IRAM_ATTR sensorISR5();

    static void IRAM_ATTR sensorISR6();

    static void IRAM_ATTR sensorISR7();

    static void IRAM_ATTR sensorISR8();

    static void IRAM_ATTR sensorISR9();

    static void IRAM_ATTR sensorISR10();

    static void IRAM_ATTR sensorISR11();

    static void IRAM_ATTR sensorISR12();

    static void IRAM_ATTR sensorISR13();

    static void IRAM_ATTR sensorISR14();


    // ========================================================
    // Processamento interno da ISR
    // ========================================================

    static void IRAM_ATTR
    handleSensorISR(
        uint8_t sensor
    );


    // ========================================================
    // Configuração do hardware
    // ========================================================

    void attachSensorInterrupts();

    void detachSensorInterrupts();


    bool configureTimer();

    void destroyTimer();


    // ========================================================
    // Captura dos contadores
    // ========================================================

    void snapshotCounters();


    // ========================================================
    // Publicação do resultado
    // ========================================================

    void publishResult(
        float angle,
        float strength,
        float normalizedStrength,
        uint32_t maxIntensity,
        uint8_t strongestSensor,
        uint32_t totalPulses,
        bool detected
    );


    // ========================================================
    // Funções auxiliares
    // ========================================================

    static float normalizeAngle(
        float angle
    );


    float angularDistance(
        float a,
        float b
    ) const;


    // ========================================================
    // Limpeza
    // ========================================================

    void clearCounters();

    void clearIntensities();

    void clearResult();

    void resetFilter();
};

#endif
