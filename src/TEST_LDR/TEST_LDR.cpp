/*
 * Leitura de 16 LDRs usando um multiplexador CD74HC4067 / 74HC4067.
 *
 * Ligações do multiplexador:
 *   SIG -> MUX_SIG
 *   S0..S3 -> MUX_S0..MUX_S3
 *   C0..C15 -> LDRs da esquerda para a direita
 *   EN -> GND
 *
 * Ajuste MANUALMENTE INITIAL_THRESHOLD apos observar as leituras no Monitor
 * Serial (115200 baud). WHITE_IS_HIGHER depende da ligacao do divisor do LDR.
 */
#include <Arduino.h>

constexpr uint8_t SENSOR_COUNT = 16;

// Ajuste estes pinos conforme a montagem da sua placa.
constexpr uint8_t MUX_SIG = 34;
constexpr uint8_t MUX_S0 = 25;
constexpr uint8_t MUX_S1 = 26;
constexpr uint8_t MUX_S2 = 27;
constexpr uint8_t MUX_S3 = 33;

constexpr uint8_t SAMPLE_COUNT = 8;
// Limiar inicial para ADC de 12 bits (0 a 4095). Ajuste apos observar
// leituras reais do verde e do branco.
constexpr uint16_t INITIAL_THRESHOLD = 2048;
constexpr bool WHITE_IS_HIGHER = true;

void selectMuxChannel(uint8_t channel)
{
	digitalWrite(MUX_S0, channel & 0x01);
	digitalWrite(MUX_S1, (channel >> 1) & 0x01);
	digitalWrite(MUX_S2, (channel >> 2) & 0x01);
	digitalWrite(MUX_S3, (channel >> 3) & 0x01);
	delayMicroseconds(5);
}

uint16_t readSensor(uint8_t sensor)
{
	selectMuxChannel(sensor);

	// A primeira conversão pode ainda refletir o canal anterior.
	analogRead(MUX_SIG);

	uint32_t total = 0;
	for (uint8_t sample = 0; sample < SAMPLE_COUNT; ++sample)
	{
		total += analogRead(MUX_SIG);
		delayMicroseconds(80);
	}

	return total / SAMPLE_COUNT;
}

bool isWhite(uint16_t value)
{
	return WHITE_IS_HIGHER ? value >= INITIAL_THRESHOLD
						   : value <= INITIAL_THRESHOLD;
}

void printMeasurements()
{
	uint16_t values[SENSOR_COUNT] = {};
	uint16_t activeCount = 0;
	int32_t weightedPosition = 0;

	Serial.print("LDR: ");

	for (uint8_t sensor = 0; sensor < SENSOR_COUNT; ++sensor)
	{
		values[sensor] = readSensor(sensor);
		const bool lineDetected = isWhite(values[sensor]);

		Serial.print(values[sensor]);
		Serial.print(lineDetected ? "*" : "-");

		if (lineDetected)
		{
			++activeCount;
			weightedPosition += static_cast<int32_t>(sensor) * 1000;
		}

		if (sensor < SENSOR_COUNT - 1)
		{
			Serial.print(" | ");
		}
	}

	Serial.print(" | ativos: ");
	Serial.print(activeCount);

	if (activeCount > 0)
	{
		// Posição em milésimos do índice: 0 = esquerda, 15000 = direita.
		Serial.print(" | posicao: ");
		Serial.println(weightedPosition / activeCount);
	}
	else
	{
		Serial.println(" | linha nao detectada");
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

	Serial.println("LDR 16 canais + CD74HC4067");
	Serial.print("Limiar fixo: ");
	Serial.println(INITIAL_THRESHOLD);
	Serial.println("Ajuste INITIAL_THRESHOLD no codigo para separar verde e branco.");
}

void loop()
{
	printMeasurements();
	delay(100);
}
