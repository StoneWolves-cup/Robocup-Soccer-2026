#include <Arduino.h>
#include <BNO055.h>

namespace
{
constexpr uint32_t INTERVALO_LEITURA_MS = 200;
BNO055 sensorBNO;
uint32_t ultimaLeitura = 0;
}

void setup()
{
	Serial.begin(115200);
	delay(1000);
	Serial.println("Teste BNO055: procurando sensor...");

	if (!sensorBNO.begin())
	{
		Serial.println("ERRO: BNO055 nao reconhecido. Verifique alimentacao e I2C (SDA=21, SCL=22).");
		while (true)
		{
			delay(1000);
		}
	}

	Serial.println("BNO055 reconhecido. Heading zerado na posicao atual.");
	Serial.println("Gire o sensor e acompanhe o heading em graus.");
}

void loop()
{
	const uint32_t agora = millis();
	if (agora - ultimaLeitura < INTERVALO_LEITURA_MS)
	{
		return;
	}
	ultimaLeitura = agora;

	sensorBNO.update();

	Serial.print("Angulo: ");
	Serial.print(sensorBNO.getHeading(), 1);
	Serial.print(" graus | Velocidade Z: ");
	Serial.print(sensorBNO.getGyroZ(), 1);
	Serial.println(" graus/s");
}
