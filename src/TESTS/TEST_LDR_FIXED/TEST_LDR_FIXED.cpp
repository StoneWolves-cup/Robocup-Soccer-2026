#include <Arduino.h>

constexpr uint8_t SENSOR_COUNT = 15;
constexpr uint8_t REGION_COUNT = 3;
constexpr uint8_t CENTER_SENSOR_COUNT = 9;
constexpr uint8_t MUX_SIG = 14;
constexpr uint8_t MUX_S0 = 10;
constexpr uint8_t MUX_S1 = 11;
constexpr uint8_t MUX_S2 = 12;
constexpr uint8_t MUX_S3 = 13;
constexpr uint8_t SAMPLE_COUNT = 12;
constexpr uint8_t ADC_DISCARD_COUNT = 2;
constexpr uint16_t HYSTERESIS_MARGIN = 80;

const char *const REGION_NAMES[REGION_COUNT] = {
    "Esquerda", "Direita", "Inferior"};
const uint8_t REGION_CHANNELS[REGION_COUNT][2] = {
    {12, 13}, {8, 9}, {10, 11}};
const uint16_t REGION_THRESHOLDS[REGION_COUNT] = {
    3332, 1304, 2312};

const uint8_t CENTER_SENSOR_CHANNELS[CENTER_SENSOR_COUNT] = {
    0, 1, 2, 3, 4, 5, 6, 7, 14};
const uint16_t CENTER_SENSOR_THRESHOLDS[CENTER_SENSOR_COUNT] = {
    2619, 2890, 3699, 1936, 2045, 1876, 2924, 1974, 3647};

bool regionIsWhite[REGION_COUNT] = {};
bool centerSensorIsWhite[CENTER_SENSOR_COUNT] = {};
bool classificationInitialized = false;

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
    for (uint8_t i = 0; i < ADC_DISCARD_COUNT; ++i)
        analogRead(MUX_SIG);

    uint32_t total = 0;
    for (uint8_t i = 0; i < SAMPLE_COUNT; ++i)
    {
        total += analogRead(MUX_SIG);
        delayMicroseconds(100);
    }
    return total / SAMPLE_COUNT;
}

uint16_t averageRegion(uint8_t region, const uint16_t readings[SENSOR_COUNT])
{
    uint32_t total = 0;
    for (uint8_t i = 0; i < 2; ++i)
        total += readings[REGION_CHANNELS[region][i]];
    return total / 2;
}

bool classifyWhiteAboveThreshold(
    uint16_t value, uint16_t threshold, bool &isWhite)
{
    if (static_cast<uint32_t>(value) >
        static_cast<uint32_t>(threshold) + HYSTERESIS_MARGIN)
    {
        isWhite = true;
    }
    else if (static_cast<uint32_t>(value) + HYSTERESIS_MARGIN < threshold)
    {
        isWhite = false;
    }
    return isWhite;
}

void initializeClassification(const uint16_t readings[SENSOR_COUNT])
{
    for (uint8_t i = 0; i < CENTER_SENSOR_COUNT; ++i)
    {
        centerSensorIsWhite[i] =
            readings[CENTER_SENSOR_CHANNELS[i]] >
            CENTER_SENSOR_THRESHOLDS[i];
    }

    for (uint8_t region = 0; region < REGION_COUNT; ++region)
    {
        regionIsWhite[region] =
            averageRegion(region, readings) > REGION_THRESHOLDS[region];
    }
    classificationInitialized = true;
}

void printReadings()
{
    uint16_t readings[SENSOR_COUNT];
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
        readings[i] = readSensor(i);

    if (!classificationInitialized)
        initializeClassification(readings);

    Serial.print("C0-C14: ");
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
    {
        Serial.print(readings[i]);
        if (i + 1 < SENSOR_COUNT)
            Serial.print(" | ");
    }

    uint8_t whiteCenterSensors = 0;
    for (uint8_t i = 0; i < CENTER_SENSOR_COUNT; ++i)
    {
        if (classifyWhiteAboveThreshold(
                readings[CENTER_SENSOR_CHANNELS[i]],
                CENTER_SENSOR_THRESHOLDS[i],
                centerSensorIsWhite[i]))
        {
            ++whiteCenterSensors;
        }
    }

    Serial.print(" || Centro: ");
    const bool centerIsWhite = whiteCenterSensors >= 2;
    Serial.print(centerIsWhite ? "BRANCO" : "VERDE");
    Serial.print(" (LDRs brancos ");
    Serial.print(whiteCenterSensors);
    Serial.print("/9; limiar de votacao: 2)");

    Serial.print(" || Regioes: ");
    for (uint8_t region = 0; region < REGION_COUNT; ++region)
    {
        const uint16_t average = averageRegion(region, readings);
        const bool isWhite = classifyWhiteAboveThreshold(
            average, REGION_THRESHOLDS[region], regionIsWhite[region]);

        Serial.print(REGION_NAMES[region]);
        Serial.print(": ");
        Serial.print(isWhite ? "BRANCO" : "VERDE");
        Serial.print(" (media ");
        Serial.print(average);
        Serial.print(", limiar ");
        Serial.print(REGION_THRESHOLDS[region]);
        Serial.print(")");
        if (region + 1 < REGION_COUNT)
            Serial.print(" | ");
    }
    Serial.println();
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

    Serial.println("Teste LDR com limiares fixos - 15 LDRs / CD4067BM / ESP32-S3");
    Serial.println("Branco = leitura acima do limiar; histerese = 80 ADC.");
    Serial.println("Centro: C0-C7/C14, voto branco com pelo menos 2 de 9 sensores.");
    Serial.println("Centro por canal:");
    for (uint8_t i = 0; i < CENTER_SENSOR_COUNT; ++i)
    {
        Serial.print("C");
        Serial.print(CENTER_SENSOR_CHANNELS[i]);
        Serial.print("=");
        Serial.print(CENTER_SENSOR_THRESHOLDS[i]);
        if (i + 1 < CENTER_SENSOR_COUNT)
            Serial.print(" | ");
    }
    Serial.println();
    Serial.println("Regioes fixas: esquerda=3332, direita=1304, inferior=2312.");
    Serial.println("Sem calibracao w/g; altere os valores constantes no firmware.");
}

void loop()
{
    printReadings();
    delay(150);
}
