#ifndef BNO055_SENSOR_H
#define BNO055_SENSOR_H

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <ConfigPinos.h>

class BNO055
{
public:
    BNO055();

    // Inicia I2C e o BNO em modo NDOF; false indica sensor indisponivel.
    bool begin(uint8_t sda = ConfigPinos::Head::BNO055::SDA,
               uint8_t scl = ConfigPinos::Head::BNO055::SCL);
    // Atualiza heading compensado e velocidade angular em graus/s.
    void update();
    // Define o rumo lido agora como zero relativo.
    void zero();
    // Captura a orientacao atual como referencia de norte da aplicacao.
    void calibrateNorth();

    // Rumo em graus no intervalo [-180, 180).
    float getHeading() const;
    // Velocidade angular no eixo Z, convertida de rad/s para graus/s.
    float getGyroZ() const;
    // Le os niveis de calibracao reportados pelo sensor.
    void getCalibration(uint8_t &system, uint8_t &gyro,
                        uint8_t &accel, uint8_t &mag);

private:
    Adafruit_BNO055 sensor;
    sensors_event_t orientationEvent;
    sensors_event_t gyroEvent;
    float headingZero;
    float heading;
    float gyroZDegreesPerSecond;
    float rawHeading;
};

#endif
