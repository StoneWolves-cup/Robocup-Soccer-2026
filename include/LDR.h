#include <Arduino.h>

// ============================================================
// 74HC4051 / HW-178
// ============================================================

// Z / COM do multiplexador
constexpr uint8_t MUX_SIG = 34;

// Seleção do canal
constexpr uint8_t MUX_S0 = 25;
constexpr uint8_t MUX_S1 = 26;
constexpr uint8_t MUX_S2 = 27;

// Quantidade de LDRs
constexpr uint8_t SENSOR_COUNT = 8;

// LEDs dos sensores
constexpr uint8_t LED_PINS[SENSOR_COUNT] = {
    4, 5, 18, 19, 21, 22, 23, 32
};

// Quantidade de leituras para fazer média
constexpr uint8_t SAMPLE_COUNT = 12;

// Evita que pequenas variacoes perto do limiar troquem a classificacao.
constexpr uint16_t CLASSIFICATION_HYSTERESIS = 80;


// ============================================================
// CALIBRAÇÃO
// ============================================================

uint16_t whiteReference[SENSOR_COUNT] = {};
uint16_t greenReference[SENSOR_COUNT] = {};
uint16_t classificationThreshold[SENSOR_COUNT] = {};

bool whiteCaptured = false;
bool greenCaptured = false;
bool sensorIsWhite[SENSOR_COUNT] = {};


// ============================================================
// SELECIONA UM CANAL DO 4051
// ============================================================

void selectMuxChannel(uint8_t channel)
{
    digitalWrite(MUX_S0, channel & 0x01);
    digitalWrite(MUX_S1, (channel >> 1) & 0x01);
    digitalWrite(MUX_S2, (channel >> 2) & 0x01);

    // Tempo para o multiplexador estabilizar
    delayMicroseconds(5);
}


// ============================================================
// LÊ UM LDR
// ============================================================

uint16_t readSensor(uint8_t sensor)
{
    selectMuxChannel(sensor);

    uint32_t total = 0;

    for (uint8_t sample = 0; sample < SAMPLE_COUNT; ++sample)
    {
        total += analogRead(MUX_SIG);

        delayMicroseconds(150);
    }

    return total / SAMPLE_COUNT;
}


// ============================================================
// CAPTURA REFERÊNCIA
// ============================================================

void captureReference(uint16_t reference[SENSOR_COUNT])
{
    for (uint8_t sensor = 0; sensor < SENSOR_COUNT; ++sensor)
    {
        reference[sensor] = readSensor(sensor);
    }
}

void updateClassificationThresholds()
{
    for (uint8_t sensor = 0; sensor < SENSOR_COUNT; ++sensor)
    {
        classificationThreshold[sensor] =
            (whiteReference[sensor] + greenReference[sensor]) / 2;

        // Comeca pelo resultado mais proximo das referencias calibradas.
        sensorIsWhite[sensor] =
            abs(static_cast<int>(whiteReference[sensor]) -
                static_cast<int>(greenReference[sensor])) == 0 ||
            abs(static_cast<int>(whiteReference[sensor]) -
                static_cast<int>(classificationThreshold[sensor])) <=
                abs(static_cast<int>(greenReference[sensor]) -
                    static_cast<int>(classificationThreshold[sensor]));
    }
}

bool classifySensor(uint8_t sensor, uint16_t value)
{
    const bool whiteIsHigher =
        whiteReference[sensor] >= greenReference[sensor];

    const uint16_t threshold = classificationThreshold[sensor];
    const uint16_t referenceDifference =
        abs(static_cast<int>(whiteReference[sensor]) -
            static_cast<int>(greenReference[sensor]));
    const uint16_t margin =
        min(CLASSIFICATION_HYSTERESIS,
            static_cast<uint16_t>(referenceDifference / 4));

    if (whiteIsHigher)
    {
        if (value > threshold + margin)
        {
            sensorIsWhite[sensor] = true;
        }
        else if (value + margin < threshold)
        {
            sensorIsWhite[sensor] = false;
        }
    }
    else
    {
        if (value + margin < threshold)
        {
            sensorIsWhite[sensor] = true;
        }
        else if (value > threshold + margin)
        {
            sensorIsWhite[sensor] = false;
        }
    }

    return sensorIsWhite[sensor];
}


// ============================================================
// MOSTRA OS 8 SENSORES
// ============================================================

void printMeasurements()
{
    Serial.print("LDR: ");

    for (uint8_t sensor = 0; sensor < SENSOR_COUNT; ++sensor)
    {
        uint16_t value = readSensor(sensor);

        Serial.print(value);

        // Só classifica depois que as duas referências
        // tiverem sido capturadas.
        if (whiteCaptured && greenCaptured)
        {
            if (classifySensor(sensor, value))
            {
                Serial.print("(BRANCO)");
            }
            else
            {
                Serial.print("(VERDE)");
            }
        }

        if (sensor < SENSOR_COUNT - 1)
        {
            Serial.print(" | ");
        }
    }

    Serial.println();
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    // Entrada analógica
    pinMode(MUX_SIG, INPUT);

    // Seleção do 4051
    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT);

    // LEDs ligados permanentemente
    for (uint8_t led = 0; led < SENSOR_COUNT; ++led)
    {
        pinMode(LED_PINS[led], OUTPUT);
        digitalWrite(LED_PINS[led], HIGH);
    }
    // ADC do ESP32
    analogReadResolution(12);

    // Faixa maior do ADC
    analogSetPinAttenuation(MUX_SIG, ADC_11db);

    Serial.println();
    Serial.println("================================");
    Serial.println("LDR + HW-178 / 74HC4051");
    Serial.println("================================");

    Serial.println("Coloque os sensores sobre BRANCO.");
    Serial.println("Envie 'w' para capturar.");

    Serial.println("Depois coloque os sensores sobre VERDE.");
    Serial.println("Envie 'g' para capturar.");

    Serial.println("Envie 'r' para apagar a calibracao.");
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    if (Serial.available() > 0)
    {
        char command = Serial.read();

        if (command == 'w' || command == 'W')
        {
            captureReference(whiteReference);

            whiteCaptured = true;

            if (greenCaptured)
            {
                updateClassificationThresholds();
            }

            Serial.println("Referencia BRANCA capturada.");
        }

        else if (command == 'g' || command == 'G')
        {
            captureReference(greenReference);

            greenCaptured = true;

            if (whiteCaptured)
            {
                updateClassificationThresholds();
            }

            Serial.println("Referencia VERDE capturada.");
        }

        else if (command == 'r' || command == 'R')
        {
            whiteCaptured = false;
            greenCaptured = false;

            Serial.println("Calibracao apagada.");
        }
    }

    printMeasurements();

    delay(250);
}