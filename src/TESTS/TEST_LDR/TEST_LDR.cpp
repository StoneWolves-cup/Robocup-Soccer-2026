#include <Arduino.h>
#include <ConfigPinos.h>

constexpr uint8_t SENSOR_COUNT = 15;
constexpr uint8_t REGION_COUNT = 4;
constexpr uint8_t MUX_SIG = ConfigPinos::Body::LDR::MUX_SINAL;
constexpr uint8_t MUX_S0 = ConfigPinos::Body::LDR::MUX_S0;
constexpr uint8_t MUX_S1 = ConfigPinos::Body::LDR::MUX_S1;
constexpr uint8_t MUX_S2 = ConfigPinos::Body::LDR::MUX_S2;
constexpr uint8_t MUX_S3 = ConfigPinos::Body::LDR::MUX_S3;
constexpr uint8_t SAMPLE_COUNT = 12;
constexpr uint8_t ADC_DISCARD_COUNT = 2;
constexpr uint16_t CALIBRATION_SAMPLES = 8;
constexpr uint16_t HYSTERESIS_DIVISOR = 8;

const char *const REGION_NAMES[REGION_COUNT] = {
    "Centro", "Esquerda", "Direita", "Inferior"};

const uint8_t REGION_CHANNELS[REGION_COUNT][9] = {
    {0, 1, 2, 3, 4, 5, 6, 7, 14},
    {12, 13},
    {8, 9},
    {10, 11}};

const uint8_t REGION_SENSOR_COUNTS[REGION_COUNT] = {9, 2, 2, 2};

uint16_t whiteReference[SENSOR_COUNT] = {};
uint16_t greenReference[SENSOR_COUNT] = {};
uint16_t threshold[SENSOR_COUNT] = {};
uint16_t regionWhiteReference[REGION_COUNT] = {};
uint16_t regionGreenReference[REGION_COUNT] = {};
uint16_t regionThreshold[REGION_COUNT] = {};
bool whiteCaptured = false;
bool greenCaptured = false;
bool regionWhiteIsHigher[REGION_COUNT] = {};
bool regionIsWhite[REGION_COUNT] = {};
bool sensorWhiteIsHigher[SENSOR_COUNT] = {};
bool sensorIsWhite[SENSOR_COUNT] = {};

uint16_t absoluteDifference(uint16_t first, uint16_t second)
{
    return first >= second ? first - second : second - first;
}

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
        sensorWhiteIsHigher[i] = whiteReference[i] >= greenReference[i];
        sensorIsWhite[i] = sensorWhiteIsHigher[i];
    }

    for (uint8_t region = 0; region < REGION_COUNT; ++region)
    {
        uint32_t whiteTotal = 0;
        uint32_t greenTotal = 0;
        uint32_t thresholdTotal = 0;

        for (uint8_t i = 0; i < REGION_SENSOR_COUNTS[region]; ++i)
        {
            const uint8_t channel = REGION_CHANNELS[region][i];
            whiteTotal += whiteReference[channel];
            greenTotal += greenReference[channel];
            thresholdTotal += threshold[channel];
        }

        const uint8_t count = REGION_SENSOR_COUNTS[region];
        regionWhiteReference[region] = whiteTotal / count;
        regionGreenReference[region] = greenTotal / count;
        regionThreshold[region] = thresholdTotal / count;
        regionWhiteIsHigher[region] =
            regionWhiteReference[region] >= regionGreenReference[region];
        regionIsWhite[region] = regionWhiteIsHigher[region];
    }
}

bool isRegionWhite(uint8_t region, uint16_t value)
{
    const uint16_t difference =
        absoluteDifference(regionWhiteReference[region], regionGreenReference[region]);
    const uint16_t margin = difference / HYSTERESIS_DIVISOR;
    const uint16_t threshold = regionThreshold[region];

    if (regionWhiteIsHigher[region])
    {
        if ((uint32_t)value > (uint32_t)threshold + margin)
            regionIsWhite[region] = true;
        else if ((uint32_t)value + margin < threshold)
            regionIsWhite[region] = false;
    }
    else
    {
        if ((uint32_t)value + margin < threshold)
            regionIsWhite[region] = true;
        else if ((uint32_t)value > (uint32_t)threshold + margin)
            regionIsWhite[region] = false;
    }
    return regionIsWhite[region];
}

bool isCenterSensorWhite(uint8_t sensor, uint16_t value)
{
    const uint16_t difference =
        absoluteDifference(whiteReference[sensor], greenReference[sensor]);
    const uint16_t margin = difference / HYSTERESIS_DIVISOR;
    const uint16_t sensorThreshold = threshold[sensor];

    if (sensorWhiteIsHigher[sensor])
    {
        if ((uint32_t)value > (uint32_t)sensorThreshold + margin)
            sensorIsWhite[sensor] = true;
        else if ((uint32_t)value + margin < sensorThreshold)
            sensorIsWhite[sensor] = false;
    }
    else
    {
        if ((uint32_t)value + margin < sensorThreshold)
            sensorIsWhite[sensor] = true;
        else if ((uint32_t)value > (uint32_t)sensorThreshold + margin)
            sensorIsWhite[sensor] = false;
    }
    return sensorIsWhite[sensor];
}

void printReadings()
{
    uint16_t readings[SENSOR_COUNT];

    Serial.print("C0-C14: ");
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
    {
        readings[i] = readSensor(i);
        Serial.print(readings[i]);
        if (i + 1 < SENSOR_COUNT)
            Serial.print(" | ");
    }

    Serial.print(" || Regioes: ");
    if (!(whiteCaptured && greenCaptured))
    {
        Serial.println("calibre branco (w) e verde (g)");
    }
    else
    {
        for (uint8_t region = 0; region < REGION_COUNT; ++region)
        {
            uint32_t total = 0;
            uint8_t whiteSensors = 0;
            for (uint8_t i = 0; i < REGION_SENSOR_COUNTS[region]; ++i)
            {
                const uint8_t sensor = REGION_CHANNELS[region][i];
                total += readings[sensor];
                if (region == 0 && isCenterSensorWhite(sensor, readings[sensor]))
                    ++whiteSensors;
            }

            const uint16_t average = total / REGION_SENSOR_COUNTS[region];
            Serial.print(REGION_NAMES[region]);
            Serial.print(": ");
            const bool regionIsWhite = region == 0
                ? whiteSensors >= 2
                : isRegionWhite(region, average);
            Serial.print(regionIsWhite ? "BRANCO" : "VERDE");
            if (region == 0)
            {
                Serial.print(" (LDRs brancos ");
                Serial.print(whiteSensors);
                Serial.print("/9; movimento ideal: ");
                Serial.print(regionIsWhite ? "para frente" : "parado");
                Serial.print(")");
            }
            else
            {
                Serial.print(" (media ");
                Serial.print(average);
                Serial.print(", limiar ");
                Serial.print(regionThreshold[region]);
                Serial.print(")");
            }
            if (region + 1 < REGION_COUNT)
                Serial.print(" | ");
        }
        Serial.println();
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

    Serial.println("Teste regional - 15 LDRs / CD4067BM / ESP32-S3");
    Serial.println("MUX: SIG=14, S0=10, S1=11, S2=12, S3=13; C15 sem uso");
    Serial.println("Regioes: Centro C0-C7/C14, Esquerda C12-C13, Direita C8-C9, Inferior C10-C11");
    Serial.println("Comandos: w = calibrar branco | g = calibrar verde | r = reiniciar");
    Serial.println("Mantenha a placa na mesma altura durante as duas calibracoes.");
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
