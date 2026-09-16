#include "IRRing.h"

#include <math.h>
#include <string.h>


// ============================================================
// Instância global usada pelas ISR
// ============================================================

IRRing* IRRing::_instance = nullptr;


// ============================================================
// Tabela das ISR
//
// IMPORTANTE:
// Existem 15 sensores físicos.
// Portanto existem somente 15 ISR.
// A posição lógica 16 é virtual.
// ============================================================

const IRRing::ISRHandler
IRRing::_sensorISRTable[
    IRRing::PHYSICAL_SENSOR_COUNT
] =
{
    IRRing::sensorISR0,
    IRRing::sensorISR1,
    IRRing::sensorISR2,
    IRRing::sensorISR3,
    IRRing::sensorISR4,
    IRRing::sensorISR5,
    IRRing::sensorISR6,
    IRRing::sensorISR7,
    IRRing::sensorISR8,
    IRRing::sensorISR9,
    IRRing::sensorISR10,
    IRRing::sensorISR11,
    IRRing::sensorISR12,
    IRRing::sensorISR13,
    IRRing::sensorISR14
};


// ============================================================
// Tabela trigonométrica
//
// 16 posições lógicas:
//
// 1  =   0°
// 2  =  22.5°
// 3  =  45°
// ...
// 15 = 315°
// 16 = 337.5°
//
// A posição 16 NÃO é usada como sensor no cálculo.
// Ela é apenas uma posição lógica.
// ============================================================

namespace
{

constexpr float COS_TABLE[
    IRRing::LOGICAL_SENSOR_COUNT
] =
{
     1.000000000f,
     0.923879533f,
     0.707106781f,
     0.382683432f,
     0.000000000f,
    -0.382683432f,
    -0.707106781f,
    -0.923879533f,
    -1.000000000f,
    -0.923879533f,
    -0.707106781f,
    -0.382683432f,
     0.000000000f,
     0.382683432f,
     0.707106781f,
     0.923879533f
};


constexpr float SIN_TABLE[
    IRRing::LOGICAL_SENSOR_COUNT
] =
{
     0.000000000f,
     0.382683432f,
     0.707106781f,
     0.923879533f,
     1.000000000f,
     0.923879533f,
     0.707106781f,
     0.382683432f,
     0.000000000f,
    -0.382683432f,
    -0.707106781f,
    -0.923879533f,
    -1.000000000f,
    -0.923879533f,
    -0.707106781f,
    -0.382683432f
};

}


// ============================================================
// CONSTRUTOR
// ============================================================

IRRing::IRRing(
    const uint8_t* sensorPins
)
{
    _instance = this;


    // --------------------------------------------------------
    // Copia os GPIOs
    // --------------------------------------------------------

    for(
        uint8_t i = 0;
        i < PHYSICAL_SENSOR_COUNT;
        i++
    )
    {
        _pins[i] = sensorPins[i];
    }


    // --------------------------------------------------------
    // Hardware
    // --------------------------------------------------------

    _timer = nullptr;


    // --------------------------------------------------------
    // FreeRTOS
    // --------------------------------------------------------

    _taskHandle = nullptr;


    // --------------------------------------------------------
    // Estado
    // --------------------------------------------------------

    _running = false;


    // --------------------------------------------------------
    // Configuração
    // --------------------------------------------------------

    _windowUs =
        DEFAULT_WINDOW_US;

    _filterAlpha =
        DEFAULT_FILTER_ALPHA;

    _detectionThreshold =
        DEFAULT_DETECTION_THRESHOLD;

    _angleOffset = 0.0f;

    _directionInverted = false;


    // --------------------------------------------------------
    // Filtro
    // --------------------------------------------------------

    _filteredX = 0.0f;

    _filteredY = 0.0f;

    _filterInitialized = false;


    // --------------------------------------------------------
    // Estatísticas
    // --------------------------------------------------------

    _isrCount = 0;

    _processCount = 0;


    // --------------------------------------------------------
    // Limpa tudo
    // --------------------------------------------------------

    clearCounters();

    clearIntensities();

    clearResult();

    resetFilter();
}


// ============================================================
// BEGIN
// ============================================================

bool IRRing::begin()
{
    if(_running)
        return true;


    // --------------------------------------------------------
    // Limpa estado anterior
    // --------------------------------------------------------

    clearCounters();

    clearIntensities();

    clearResult();

    resetFilter();


    // --------------------------------------------------------
    // Configura GPIOs
    // --------------------------------------------------------

    for(
        uint8_t i = 0;
        i < PHYSICAL_SENSOR_COUNT;
        i++
    )
    {
        const uint8_t pin = _pins[i];


        // ----------------------------------------------------
        // GPIOs somente entrada
        //
        // GPIO34
        // GPIO35
        // GPIO36
        // GPIO39
        //
        // Não possuem pull-up interno.
        // ----------------------------------------------------

        if(
            pin == 34 ||
            pin == 35 ||
            pin == 36 ||
            pin == 39
        )
        {
            pinMode(
                pin,
                INPUT
            );
        }
        else
        {
            // ------------------------------------------------
            // Para os demais pinos.
            //
            // Se sua PCB já possuir resistores externos,
            // INPUT também pode ser utilizado.
            // ------------------------------------------------

            pinMode(
                pin,
                INPUT_PULLUP
            );
        }
    }


    // --------------------------------------------------------
    // Configura timer
    // --------------------------------------------------------

    if(!configureTimer())
    {
        return false;
    }


    // --------------------------------------------------------
    // Configura interrupções
    // --------------------------------------------------------

    attachSensorInterrupts();


    // --------------------------------------------------------
    // Sistema funcionando
    // --------------------------------------------------------

    _running = true;


    return true;
}


// ============================================================
// END
// ============================================================

void IRRing::end()
{
    if(!_running)
        return;


    detachSensorInterrupts();

    destroyTimer();


    _running = false;
}


// ============================================================
// CONFIGURAÇÃO DO TIMER
//
// ESP32 Arduino Core 3.x
//
// timerBegin(frequency)
//
// Aqui utilizamos 1 MHz.
//
// Portanto:
//
// 1 tick = 1 us
// ============================================================

bool IRRing::configureTimer()
{
    _timer =
        timerBegin(1000000);


    if(_timer == nullptr)
    {
        return false;
    }


    timerAttachInterrupt(
        _timer,
        &IRRing::timerISR
    );


    timerAlarm(
        _timer,
        _windowUs,
        true,
        0
    );


    return true;
}


// ============================================================
// DESTRÓI TIMER
// ============================================================

void IRRing::destroyTimer()
{
    if(_timer == nullptr)
        return;


    timerEnd(
        _timer
    );


    _timer = nullptr;
}


// ============================================================
// TIMER ISR
//
// Não faz cálculo.
//
// Apenas acorda a FreeRTOS Task.
// ============================================================

void IRAM_ATTR IRRing::timerISR()
{
    IRRing* instance =
        _instance;

    if(instance == nullptr)
        return;


    TaskHandle_t task =
        instance->_taskHandle;

    if(task == nullptr)
        return;


    BaseType_t higherPriorityTaskWoken =
        pdFALSE;


    vTaskNotifyGiveFromISR(
        task,
        &higherPriorityTaskWoken
    );


    if(higherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}


// ============================================================
// ATTACH INTERRUPTS
// ============================================================

void IRRing::attachSensorInterrupts()
{
    for(
        uint8_t i = 0;
        i < PHYSICAL_SENSOR_COUNT;
        i++
    )
    {
        attachInterrupt(
            digitalPinToInterrupt(_pins[i]),
            _sensorISRTable[i],
            FALLING
        );
    }
}


// ============================================================
// DETACH INTERRUPTS
// ============================================================

void IRRing::detachSensorInterrupts()
{
    for(
        uint8_t i = 0;
        i < PHYSICAL_SENSOR_COUNT;
        i++
    )
    {
        detachInterrupt(
            digitalPinToInterrupt(_pins[i])
        );
    }
}


// ============================================================
// SENSOR ISR GENÉRICA
// ============================================================

void IRAM_ATTR IRRing::handleSensorISR(
    uint8_t sensor
)
{
    IRRing* instance =
        _instance;

    if(instance == nullptr)
        return;

    if(sensor >= PHYSICAL_SENSOR_COUNT)
        return;


    portENTER_CRITICAL_ISR(
        &instance->_mux
    );


    instance->_pulseCount[sensor] =
        instance->_pulseCount[sensor] + 1U;


    instance->_isrCount =
        instance->_isrCount + 1U;


    portEXIT_CRITICAL_ISR(
        &instance->_mux
    );
}


// ============================================================
// ISR SENSOR 0
// ============================================================

void IRAM_ATTR IRRing::sensorISR0()
{
    handleSensorISR(0);
}


// ============================================================
// ISR SENSOR 1
// ============================================================

void IRAM_ATTR IRRing::sensorISR1()
{
    handleSensorISR(1);
}


// ============================================================
// ISR SENSOR 2
// ============================================================

void IRAM_ATTR IRRing::sensorISR2()
{
    handleSensorISR(2);
}


// ============================================================
// ISR SENSOR 3
// ============================================================

void IRAM_ATTR IRRing::sensorISR3()
{
    handleSensorISR(3);
}


// ============================================================
// ISR SENSOR 4
// ============================================================

void IRAM_ATTR IRRing::sensorISR4()
{
    handleSensorISR(4);
}


// ============================================================
// ISR SENSOR 5
// ============================================================

void IRAM_ATTR IRRing::sensorISR5()
{
    handleSensorISR(5);
}


// ============================================================
// ISR SENSOR 6
// ============================================================

void IRAM_ATTR IRRing::sensorISR6()
{
    handleSensorISR(6);
}


// ============================================================
// ISR SENSOR 7
// ============================================================

void IRAM_ATTR IRRing::sensorISR7()
{
    handleSensorISR(7);
}


// ============================================================
// ISR SENSOR 8
// ============================================================

void IRAM_ATTR IRRing::sensorISR8()
{
    handleSensorISR(8);
}


// ============================================================
// ISR SENSOR 9
// ============================================================

void IRAM_ATTR IRRing::sensorISR9()
{
    handleSensorISR(9);
}


// ============================================================
// ISR SENSOR 10
// ============================================================

void IRAM_ATTR IRRing::sensorISR10()
{
    handleSensorISR(10);
}


// ============================================================
// ISR SENSOR 11
// ============================================================

void IRAM_ATTR IRRing::sensorISR11()
{
    handleSensorISR(11);
}


// ============================================================
// ISR SENSOR 12
// ============================================================

void IRAM_ATTR IRRing::sensorISR12()
{
    handleSensorISR(12);
}


// ============================================================
// ISR SENSOR 13
// ============================================================

void IRAM_ATTR IRRing::sensorISR13()
{
    handleSensorISR(13);
}


// ============================================================
// ISR SENSOR 14
// ============================================================

void IRAM_ATTR IRRing::sensorISR14()
{
    handleSensorISR(14);
}


// ============================================================
// SNAPSHOT DOS CONTADORES
//
// Copia os pulsos acumulados pelas ISR.
//
// Depois zera os contadores.
//
// O cálculo pesado NÃO acontece dentro da ISR.
// ============================================================

void IRRing::snapshotCounters()
{
    portENTER_CRITICAL(
        &_mux
    );


    for(
        uint8_t i = 0;
        i < PHYSICAL_SENSOR_COUNT;
        i++
    )
    {
        _intensity[i] =
            _pulseCount[i];


        _pulseCount[i] =
            0;
    }


    portEXIT_CRITICAL(
        &_mux
    );
}


// ============================================================
// PROCESSAMENTO
// ============================================================

void IRRing::process()
{
    if(!_running)
        return;


    _processCount++;


    // --------------------------------------------------------
    // Obtém os pulsos da janela atual
    // --------------------------------------------------------

    snapshotCounters();


    float x = 0.0f;

    float y = 0.0f;


    uint32_t totalPulses = 0;

    uint32_t maxIntensity = 0;

    uint8_t strongestSensor = 0;


    // --------------------------------------------------------
    // Calcula vetor
    //
    // SOMENTE os 15 sensores físicos participam.
    //
    // A posição 16 é virtual e NÃO recebe intensidade.
    // --------------------------------------------------------

    for(
        uint8_t i = 0;
        i < PHYSICAL_SENSOR_COUNT;
        i++
    )
    {
        const uint32_t value =
            _intensity[i];


        totalPulses +=
            value;


        if(value > maxIntensity)
        {
            maxIntensity =
                value;

            strongestSensor =
                i;
        }


        x +=
            static_cast<float>(value) *
            COS_TABLE[i];


        y +=
            static_cast<float>(value) *
            SIN_TABLE[i];
    }


    // --------------------------------------------------------
    // Sem bola
    // --------------------------------------------------------

    if(
        totalPulses <
        _detectionThreshold
    )
    {
        publishResult(
            0.0f,
            0.0f,
            0.0f,
            maxIntensity,
            strongestSensor,
            totalPulses,
            false
        );


        return;
    }


    // --------------------------------------------------------
    // Filtro exponencial no vetor
    //
    // Isso é melhor do que filtrar o ângulo diretamente,
    // porque evita problemas na passagem 359° -> 0°.
    // --------------------------------------------------------

    if(!_filterInitialized)
    {
        _filteredX = x;

        _filteredY = y;

        _filterInitialized = true;
    }
    else
    {
        _filteredX =
            (
                _filterAlpha *
                _filteredX
            )
            +
            (
                (1.0f - _filterAlpha) *
                x
            );


        _filteredY =
            (
                _filterAlpha *
                _filteredY
            )
            +
            (
                (1.0f - _filterAlpha) *
                y
            );
    }


    // --------------------------------------------------------
    // Magnitude do vetor
    // --------------------------------------------------------

    const float strength =
        hypotf(
            _filteredX,
            _filteredY
        );


    // --------------------------------------------------------
    // Ângulo
    //
    // atan2 retorna radianos.
    // --------------------------------------------------------

    float angle =
        atan2f(
            _filteredY,
            _filteredX
        )
        * 180.0f
        / PI;


    angle =
        normalizeAngle(angle);


    // --------------------------------------------------------
    // Offset mecânico
    // --------------------------------------------------------

    angle +=
        _angleOffset;


    angle =
        normalizeAngle(angle);


    // --------------------------------------------------------
    // Inversão opcional
    // --------------------------------------------------------

    if(_directionInverted)
    {
        angle =
            normalizeAngle(
                360.0f - angle
            );
    }


    // --------------------------------------------------------
    // Intensidade normalizada
    //
    // Mostra quanto o sensor dominante representa
    // do sinal total.
    // --------------------------------------------------------

    float normalizedStrength =
        0.0f;


    if(totalPulses > 0)
    {
        normalizedStrength =
            static_cast<float>(
                maxIntensity
            )
            /
            static_cast<float>(
                totalPulses
            );
    }


    // --------------------------------------------------------
    // Publica resultado
    // --------------------------------------------------------

    publishResult(
        angle,
        strength,
        normalizedStrength,
        maxIntensity,
        strongestSensor,
        totalPulses,
        true
    );
}


// ============================================================
// PUBLICA RESULTADO
// ============================================================

void IRRing::publishResult(
    float angle,
    float strength,
    float normalizedStrength,
    uint32_t maxIntensity,
    uint8_t strongestSensor,
    uint32_t totalPulses,
    bool detected
)
{
    portENTER_CRITICAL(
        &_mux
    );


    _angle =
        angle;


    _strength =
        strength;


    _normalizedStrength =
        normalizedStrength;


    _maxIntensity =
        maxIntensity;


    _strongestSensor =
        strongestSensor;


    _totalPulses =
        totalPulses;


    _detected =
        detected;


    portEXIT_CRITICAL(
        &_mux
    );
}


// ============================================================
// NORMALIZA ÂNGULO
// ============================================================

float IRRing::normalizeAngle(
    float angle
)
{
    while(angle >= 360.0f)
    {
        angle -= 360.0f;
    }


    while(angle < 0.0f)
    {
        angle += 360.0f;
    }


    return angle;
}


// ============================================================
// DISTÂNCIA ANGULAR
// ============================================================

float IRRing::angularDistance(
    float a,
    float b
) const
{
    a =
        normalizeAngle(a);


    b =
        normalizeAngle(b);


    float difference =
        fabsf(a - b);


    if(difference > 180.0f)
    {
        difference =
            360.0f - difference;
    }


    return difference;
}


// ============================================================
// SENSOR VIRTUAL 16
//
// Posição:
//
// 337,5°
//
// Condições:
//
// 1. Bola detectada
// 2. Sensor 1 não tem sinal significativo
// 3. Sensor 15 não tem sinal significativo
// 4. Ângulo está próximo de 337,5°
// ============================================================

bool IRRing::isVirtualSensor16Detected() const
{
    if(!ballDetected())
        return false;


    // Sensor físico 1
    const uint32_t sensor1 =
        getSensorIntensity(0);


    // Sensor físico 15
    const uint32_t sensor15 =
        getSensorIntensity(14);


    if(
        sensor1 >=
        _detectionThreshold
    )
    {
        return false;
    }


    if(
        sensor15 >=
        _detectionThreshold
    )
    {
        return false;
    }


    const float angle =
        getAngle();


    constexpr float MAX_ERROR =
        11.25f;


    return
        angularDistance(
            angle,
            VIRTUAL_SENSOR_16_ANGLE
        )
        <= MAX_ERROR;
}


// ============================================================
// SENSOR LÓGICO MAIS FORTE
//
// Retorno:
//
// 1..15 = sensores físicos
// 16    = posição virtual
// ============================================================

uint8_t IRRing::getStrongestLogicalSensor() const
{
    if(
        isVirtualSensor16Detected()
    )
    {
        return 16;
    }


    return
        getStrongestSensor() + 1;
}


// ============================================================
// SENSOR FÍSICO MAIS FORTE
//
// Retorna 1..15.
// ============================================================

uint8_t IRRing::getStrongestSensor() const
{
    portENTER_CRITICAL(
        const_cast<portMUX_TYPE*>(&_mux)
    );


    const uint8_t sensor =
        _strongestSensor;


    portEXIT_CRITICAL(
        const_cast<portMUX_TYPE*>(&_mux)
    );


    return sensor + 1;
}


// ============================================================
// GET ANGLE
// ============================================================

float IRRing::getAngle() const
{
    portENTER_CRITICAL(
        const_cast<portMUX_TYPE*>(&_mux)
    );


    const float value =
        _angle;


    portEXIT_CRITICAL(
        const_cast<portMUX_TYPE*>(&_mux)
    );


    return value;
}


// ============================================================
// GET STRENGTH
// ============================================================

float IRRing::getStrength() const
{
    portENTER_CRITICAL(
        const_cast<portMUX_TYPE*>(&_mux)
    );


    const float value =
        _strength;


    portEXIT_CRITICAL(
        const_cast<portMUX_TYPE*>(&_mux)
    );


    return value;
}


// ============================================================
// GET NORMALIZED STRENGTH
// ============================================================

float IRRing::getNormalizedStrength() const
{
    portENTER_CRITICAL(
        const_cast<portMUX_TYPE*>(&_mux)
    );


    const float value =
        _normalizedStrength;


    portEXIT_CRITICAL(
        const_cast<portMUX_TYPE*>(&_mux)
    );


    return value;
}


// ============================================================
// BALL DETECTED
// ============================================================

bool IRRing::ballDetected() const
{
    portENTER_CRITICAL(
        const_cast<portMUX_TYPE*>(&_mux)
    );


    const bool value =
        _detected;


    portEXIT_CRITICAL(
        const_cast<portMUX_TYPE*>(&_mux)
    );


    return value;
}


// ============================================================
// SENSOR INTENSITY
//
// Índice interno:
//
// 0  -> sensor 1
// ...
// 14 -> sensor 15
// ============================================================

uint32_t IRRing::getSensorIntensity(
    uint8_t sensor
) const
{
    if(
        sensor >=
        PHYSICAL_SENSOR_COUNT
    )
    {
        return 0;
    }


    portENTER_CRITICAL(
        const_cast<portMUX_TYPE*>(&_mux)
    );


    const uint32_t value =
        _intensity[sensor];


    portEXIT_CRITICAL(
        const_cast<portMUX_TYPE*>(&_mux)
    );


    return value;
}


// ============================================================
// COPIA INTENSIDADES
// ============================================================

void IRRing::copyIntensities(
    uint32_t* destination,
    size_t length
) const
{
    if(destination == nullptr)
        return;


    if(
        length >
        PHYSICAL_SENSOR_COUNT
    )
    {
        length =
            PHYSICAL_SENSOR_COUNT;
    }


    portENTER_CRITICAL(
        const_cast<portMUX_TYPE*>(&_mux)
    );


    for(
        size_t i = 0;
        i < length;
        i++
    )
    {
        destination[i] =
            _intensity[i];
    }


    portEXIT_CRITICAL(
        const_cast<portMUX_TYPE*>(&_mux)
    );
}


// ============================================================
// RESULTADO COMPLETO
// ============================================================

IRRing::Result IRRing::getResult() const
{
    Result result;


    portENTER_CRITICAL(
        const_cast<portMUX_TYPE*>(&_mux)
    );


    result.angle =
        _angle;


    result.strength =
        _strength;


    result.normalizedStrength =
        _normalizedStrength;


    result.maxIntensity =
        _maxIntensity;


    result.strongestSensor =
        _strongestSensor + 1;


    result.totalPulses =
        _totalPulses;


    result.detected =
        _detected;


    result.virtualSensor16 =
        _virtualSensor16;


    result.logicalSensor =
        _logicalSensor;


    portEXIT_CRITICAL(
        const_cast<portMUX_TYPE*>(&_mux)
    );


    // --------------------------------------------------------
    // Atualiza informações lógicas fora da seção crítica
    // --------------------------------------------------------

    result.virtualSensor16 =
        isVirtualSensor16Detected();


    result.logicalSensor =
        getStrongestLogicalSensor();


    return result;
}


// ============================================================
// ISR COUNT
// ============================================================

uint32_t IRRing::getISRCount() const
{
    portENTER_CRITICAL(
        const_cast<portMUX_TYPE*>(&_mux)
    );


    const uint32_t value =
        _isrCount;


    portEXIT_CRITICAL(
        const_cast<portMUX_TYPE*>(&_mux)
    );


    return value;
}


// ============================================================
// PROCESS COUNT
// ============================================================

uint32_t IRRing::getProcessCount() const
{
    return _processCount;
}


// ============================================================
// ATTACH TASK
// ============================================================

void IRRing::attachTask(
    TaskHandle_t taskHandle
)
{
    portENTER_CRITICAL(
        &_mux
    );


    _taskHandle =
        taskHandle;


    portEXIT_CRITICAL(
        &_mux
    );
}


// ============================================================
// GET TASK HANDLE
// ============================================================

TaskHandle_t IRRing::getTaskHandle() const
{
    return _taskHandle;
}


// ============================================================
// WINDOW
// ============================================================

void IRRing::setWindowUs(
    uint32_t windowUs
)
{
    if(windowUs == 0)
        return;


    _windowUs =
        windowUs;


    if(_timer != nullptr)
    {
        timerAlarm(
            _timer,
            _windowUs,
            true,
            0
        );
    }
}


// ============================================================
// GET WINDOW
// ============================================================

uint32_t IRRing::getWindowUs() const
{
    return _windowUs;
}


// ============================================================
// FILTER ALPHA
// ============================================================

void IRRing::setFilterAlpha(
    float alpha
)
{
    if(alpha < 0.0f)
        alpha = 0.0f;


    if(alpha > 1.0f)
        alpha = 1.0f;


    _filterAlpha =
        alpha;
}


// ============================================================
// GET FILTER ALPHA
// ============================================================

float IRRing::getFilterAlpha() const
{
    return _filterAlpha;
}


// ============================================================
// DETECTION THRESHOLD
// ============================================================

void IRRing::setDetectionThreshold(
    uint32_t threshold
)
{
    _detectionThreshold =
        threshold;
}


// ============================================================
// GET DETECTION THRESHOLD
// ============================================================

uint32_t IRRing::getDetectionThreshold() const
{
    return _detectionThreshold;
}


// ============================================================
// ANGLE OFFSET
// ============================================================

void IRRing::setAngleOffset(
    float offset
)
{
    _angleOffset =
        normalizeAngle(offset);
}


// ============================================================
// GET ANGLE OFFSET
// ============================================================

float IRRing::getAngleOffset() const
{
    return _angleOffset;
}


// ============================================================
// DIRECTION INVERTED
// ============================================================

void IRRing::setDirectionInverted(
    bool inverted
)
{
    _directionInverted =
        inverted;
}


// ============================================================
// IS DIRECTION INVERTED
// ============================================================

bool IRRing::isDirectionInverted() const
{
    return _directionInverted;
}


// ============================================================
// CLEAR COUNTERS
// ============================================================

void IRRing::clearCounters()
{
    portENTER_CRITICAL(
        &_mux
    );


    for(
        uint8_t i = 0;
        i < PHYSICAL_SENSOR_COUNT;
        i++
    )
    {
        _pulseCount[i] =
            0;
    }


    _isrCount =
        0;


    portEXIT_CRITICAL(
        &_mux
    );
}


// ============================================================
// CLEAR INTENSITIES
// ============================================================

void IRRing::clearIntensities()
{
    portENTER_CRITICAL(
        &_mux
    );


    for(
        uint8_t i = 0;
        i < PHYSICAL_SENSOR_COUNT;
        i++
    )
    {
        _intensity[i] =
            0;
    }


    portEXIT_CRITICAL(
        &_mux
    );
}


// ============================================================
// CLEAR RESULT
// ============================================================

void IRRing::clearResult()
{
    portENTER_CRITICAL(
        &_mux
    );


    _angle =
        0.0f;


    _strength =
        0.0f;


    _normalizedStrength =
        0.0f;


    _maxIntensity =
        0;


    _strongestSensor =
        0;


    _totalPulses =
        0;


    _detected =
        false;


    _virtualSensor16 =
        false;


    _logicalSensor =
        0;


    portEXIT_CRITICAL(
        &_mux
    );
}


// ============================================================
// RESET FILTER
// ============================================================

void IRRing::resetFilter()
{
    _filteredX =
        0.0f;


    _filteredY =
        0.0f;


    _filterInitialized =
        false;
}
