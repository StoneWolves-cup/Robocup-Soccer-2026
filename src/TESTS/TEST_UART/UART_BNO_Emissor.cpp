#include <Arduino.h>
#include <BNO055.h>

namespace
{
constexpr int UART_RX_PIN = 18;
constexpr int UART_TX_PIN = 17;
constexpr uint32_t INTERVALO_ENVIO_MS = 100;

BNO055 sensorBNO;
uint32_t ultimoEnvioMs = 0;
}

void setup()
{
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    Serial.println("=== Emissor UART com BNO055 ===");
    if (!sensorBNO.begin())
    {
        Serial.println("ERRO: BNO055 nao encontrado. Verifique a conexao I2C.");
        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("Enviando heading e gyro Z para o ESP receptor.");
}

void loop()
{
    const uint32_t agoraMs = millis();
    if (agoraMs - ultimoEnvioMs < INTERVALO_ENVIO_MS)
    {
        return;
    }
    ultimoEnvioMs = agoraMs;

    sensorBNO.update();

    uint8_t sistema = 0;
    uint8_t gyro = 0;
    uint8_t acelerometro = 0;
    uint8_t magnetometro = 0;
    sensorBNO.getCalibration(sistema, gyro, acelerometro, magnetometro);

    Serial1.printf("BNO heading=%.2f deg gyroZ=%.2f deg/s calib=%u,%u,%u,%u\n",
                   sensorBNO.getHeading(), sensorBNO.getGyroZ(),
                   static_cast<unsigned>(sistema),
                   static_cast<unsigned>(gyro),
                   static_cast<unsigned>(acelerometro),
                   static_cast<unsigned>(magnetometro));
}
