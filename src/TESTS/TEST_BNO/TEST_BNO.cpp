#include <Arduino.h>
#include <BNO055.h>
#include <PID.h>

namespace
{
// Leituras e controle em 100 Hz; envio de diagnostico em 10 Hz.
constexpr uint32_t INTERVALO_SENSOR_MS = 10;
constexpr uint32_t INTERVALO_PID_US = 10000;
constexpr uint32_t INTERVALO_SERIAL_MS = 100;
constexpr uint32_t INTERVALO_CALIBRACAO_MS = 1000;

// Ganhos e limites preservados da mecanica antiga do teste.
BNO055 sensorBNO;
PID pidHeading(1.5f, 0.0f, 0.10f, -255.0f, 255.0f);

float headingDesejado = 0.0f;
float pidOutput = 0.0f;

uint32_t ultimoSensorMs = 0;
uint32_t ultimoPIDUs = 0;
uint32_t ultimoPrintMs = 0;
uint32_t inicioCalibracaoMs = 0;
bool aguardandoCalibracao = false;

float calcularErroAngular(float desejado, float atual)
{
    // Escolhe o menor caminho entre os angulos, inclusive ao cruzar +/-180.
    float erro = desejado - atual;

    while (erro > 180.0f)
    {
        erro -= 360.0f;
    }
    while (erro < -180.0f)
    {
        erro += 360.0f;
    }

    return erro;
}
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=== Teste BNO055 + PID de heading ===");

    if (!sensorBNO.begin())
    {
        Serial.println("ERRO: BNO055 nao reconhecido. Verifique alimentacao e I2C (SDA=21, SCL=22).");
        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("Calibracao magnetica: use a serial para definir a referencia inicial.");
    Serial.println("Aponte o robo na direcao inicial desejada e envie 'c'.");
    Serial.println("O heading mostrado sera relativo a essa direcao (0 graus).");
    inicioCalibracaoMs = millis();
    aguardandoCalibracao = true;
}

void loop()
{
    // Atualiza a orientacao e calcula o PID em tarefas temporizadas separadas.
    const uint32_t agoraMs = millis();
    if (agoraMs - ultimoSensorMs >= INTERVALO_SENSOR_MS)
    {
        ultimoSensorMs = agoraMs;
        sensorBNO.update();
    }

    if (aguardandoCalibracao)
    {
        const bool comandoCalibrar = Serial.available() && (Serial.read() == 'c' || Serial.read() == 'C');
        const bool tempoEsgotado = agoraMs - inicioCalibracaoMs >= INTERVALO_CALIBRACAO_MS;
        if (comandoCalibrar || tempoEsgotado)
        {
            // Fallback automatico mantem o teste utilizavel sem terminal serial.
            sensorBNO.update();
            sensorBNO.calibrateNorth();
            pidHeading.reset();
            aguardandoCalibracao = false;
            ultimoSensorMs = agoraMs;
            ultimoPIDUs = micros();
            ultimoPrintMs = agoraMs;
            Serial.println(comandoCalibrar ? "Referencia magnetica definida: 0 graus." : "Referencia definida automaticamente pela direcao atual.");
            Serial.println("Envie 'c' novamente para recalibrar a referencia.");
        }
        return;
    }

    if (Serial.available())
    {
        const char comando = Serial.read();
        if (comando == 'c' || comando == 'C')
        {
            sensorBNO.update();
            sensorBNO.calibrateNorth();
            pidHeading.reset();
            Serial.println("Referencia magnetica recalibrada: heading = 0 graus.");
        }
    }

    const uint32_t agoraUs = micros();
    const uint32_t decorridoUs = agoraUs - ultimoPIDUs;
    if (decorridoUs >= INTERVALO_PID_US)
    {
        ultimoPIDUs = agoraUs;
        const float dt = decorridoUs / 1000000.0f;
        const float erro = calcularErroAngular(headingDesejado, sensorBNO.getHeading());
        pidOutput = pidHeading.calculate(erro, sensorBNO.getGyroZ(), dt);
    }

    if (agoraMs - ultimoPrintMs >= INTERVALO_SERIAL_MS)
    {
        ultimoPrintMs = agoraMs;

        Serial.print("Heading: ");
        Serial.print(sensorBNO.getHeading(), 2);
        Serial.print(" | Desejado: ");
        Serial.print(headingDesejado, 2);
        Serial.print(" | Erro: ");
        Serial.print(calcularErroAngular(headingDesejado, sensorBNO.getHeading()), 2);
        Serial.print(" | Gyro Z: ");
        Serial.print(sensorBNO.getGyroZ(), 3);
        Serial.print(" | PID: ");
        Serial.println(pidOutput, 2);
    }
}
