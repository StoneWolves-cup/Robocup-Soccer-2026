#include <Arduino.h>

// Teste para 16 LDRs ligados aos canais C0..C15 de um CD74HC4067.
// A placa deve manter os LEDs IR ligados durante todas as leituras.
// Confirme os pinos abaixo com a fiação real antes de energizar.
constexpr uint8_t SENSOR_COUNT = 16;
constexpr uint8_t MUX_SIG = 15; // ADC1 no ESP32 DevKit V1 (GPIO4, somente entrada)
constexpr uint8_t MUX_S0 = 18;
constexpr uint8_t MUX_S1 = 5;
constexpr uint8_t MUX_S2 = 4;
constexpr uint8_t MUX_S3 = 2;
constexpr uint8_t SAMPLE_COUNT = 12;
constexpr uint8_t ADC_DISCARD_COUNT = 2;
constexpr uint16_t CALIBRATION_SAMPLES = 8;
constexpr uint16_t HYSTERESIS_DIVISOR = 8;

uint16_t whiteReference[SENSOR_COUNT] = {};
uint16_t greenReference[SENSOR_COUNT] = {};
uint16_t threshold[SENSOR_COUNT] = {};
bool whiteCaptured = false;
bool greenCaptured = false;
bool sensorIsWhite[SENSOR_COUNT] = {};

void selectMuxChannel(uint8_t channel)
{
    digitalWrite(MUX_S0, channel & 0x01);
    digitalWrite(MUX_S1, (channel >> 1) & 0x01);
    digitalWrite(MUX_S2, (channel >> 2) & 0x01);
    digitalWrite(MUX_S3, (channel >> 3) & 0x01);
    delayMicroseconds(10);
}

uint16_t readSensor(uint8_t channel)
{
    selectMuxChannel(channel);
    for (uint8_t i = 0; i < ADC_DISCARD_COUNT; ++i) analogRead(MUX_SIG);

    uint32_t total = 0;
    for (uint8_t i = 0; i < SAMPLE_COUNT; ++i)
    {
        total += analogRead(MUX_SIG);
        delayMicroseconds(100);
    }
    return total / SAMPLE_COUNT;
}

uint16_t captureAverage(uint8_t channel)
{
    uint32_t total = 0;
    for (uint8_t i = 0; i < CALIBRATION_SAMPLES; ++i)
    {
        total += readSensor(channel);
        delay(2);
    }
    return total / CALIBRATION_SAMPLES;
}

void captureReference(uint16_t reference[SENSOR_COUNT])
{
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
        reference[i] = captureAverage(i);
}

void prepareClassification()
{
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
    {
        threshold[i] = (whiteReference[i] + greenReference[i]) / 2;
        sensorIsWhite[i] = whiteReference[i] >= greenReference[i];
    }
}

bool isWhite(uint8_t sensor, uint16_t value)
{
    const bool whiteHigher = whiteReference[sensor] >= greenReference[sensor];
    const uint16_t difference = abs((int)whiteReference[sensor] - (int)greenReference[sensor]);
    const uint16_t margin = difference / HYSTERESIS_DIVISOR;

    if (whiteHigher)
    {
        if ((uint32_t)value > (uint32_t)threshold[sensor] + margin) sensorIsWhite[sensor] = true;
        else if ((uint32_t)value + margin < threshold[sensor]) sensorIsWhite[sensor] = false;
    }
    else
    {
        if ((uint32_t)value + margin < threshold[sensor]) sensorIsWhite[sensor] = true;
        else if ((uint32_t)value > (uint32_t)threshold[sensor] + margin) sensorIsWhite[sensor] = false;
    }
    return sensorIsWhite[sensor];
}

void printReadings()
{
    uint16_t activeCount = 0;
    int32_t weightedPosition = 0;

    Serial.print("C0-C15: ");
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
    {
        const uint16_t value = readSensor(i);
        Serial.print(value);
        if (whiteCaptured && greenCaptured && isWhite(i, value))
        {
            Serial.print("*");
            ++activeCount;
            weightedPosition += (int32_t)i * 1000;
        }
        if (i + 1 < SENSOR_COUNT) Serial.print(" | ");
    }

    Serial.print(" || linha branca: ");
    if (!(whiteCaptured && greenCaptured))
    {
        Serial.println("calibre com w e g");
    }
    else if (activeCount == 0)
    {
        Serial.println("nao detectada");
    }
    else
    {
        Serial.print(activeCount);
        Serial.print(" sensores | centro (indice x1000): ");
        Serial.println(weightedPosition / activeCount);
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(MUX_SIG, INPUT);
    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT);
    pinMode(MUX_S3, OUTPUT);

    analogReadResolution(12);
    analogSetPinAttenuation(MUX_SIG, ADC_11db);

    Serial.println("Teste de linha branca - 16 LDRs / CD74HC4067");
    Serial.println("Comandos: w = calibrar branco | g = calibrar verde | r = reiniciar");
    Serial.println("Mantenha a placa na mesma altura e com LEDs IR ligados ao calibrar.");
}

void loop()
{
    while (Serial.available())
    {
        const char command = Serial.read();
        if (command == 'w' || command == 'W')
        {
            Serial.println("Capturando branco...");
            captureReference(whiteReference);
            whiteCaptured = true;
            if (greenCaptured) prepareClassification();
            Serial.println("Referencia branca capturada.");
        }
        else if (command == 'g' || command == 'G')
        {
            Serial.println("Capturando verde...");
            captureReference(greenReference);
            greenCaptured = true;
            if (whiteCaptured) prepareClassification();
            Serial.println("Referencia verde capturada.");
        }
        else if (command == 'r' || command == 'R')
        {
            whiteCaptured = false;
            greenCaptured = false;
            Serial.println("Calibracao reiniciada.");
        }
    }

    printReadings();
    delay(150);
}
