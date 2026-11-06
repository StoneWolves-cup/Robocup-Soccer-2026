#include <BNO055.h>
#include <Wire.h>

BNO055::BNO055()
    : sensor(55, 0x28),
      headingZero(0.0f),
      heading(0.0f),
      gyroZDegreesPerSecond(0.0f),
      rawHeading(0.0f)
{
}

bool BNO055::begin(uint8_t sda, uint8_t scl)
{
    Wire.begin(sda, scl);

    // NDOF combina acelerometro, giroscopio e magnetometro.
    if (!sensor.begin(OPERATION_MODE_NDOF))
    {
        return false;
    }

    delay(100);
    sensor.setExtCrystalUse(true);
    update();
    return true;
}

void BNO055::update()
{
    sensor.getEvent(&orientationEvent);
    sensor.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);

    rawHeading = orientationEvent.orientation.x;
    // Com headingZero=0 (padrao), mantem o rumo absoluto calculado pelo
    // NDOF em relacao ao norte magnetico. zero() continua disponivel para
    // aplicacoes que queiram explicitamente um referencial relativo.
    heading = rawHeading - headingZero;
    while (heading >= 180.0f)
    {
        heading -= 360.0f;
    }
    while (heading < -180.0f)
    {
        heading += 360.0f;
    }

    // Adafruit reports angular velocity in rad/s.
    gyroZDegreesPerSecond = gyroEvent.gyro.z * 180.0f / PI;
}

void BNO055::zero()
{
    sensor.getEvent(&orientationEvent);
    headingZero = orientationEvent.orientation.x;
    heading = 0.0f;
}

void BNO055::calibrateNorth()
{
    sensor.getEvent(&orientationEvent);
    rawHeading = orientationEvent.orientation.x;
    headingZero = rawHeading;
    heading = 0.0f;
}

float BNO055::getHeading() const
{
    return heading;
}

float BNO055::getGyroZ() const
{
    return gyroZDegreesPerSecond;
}

void BNO055::getCalibration(uint8_t &system, uint8_t &gyro,
                            uint8_t &accel, uint8_t &mag)
{
    sensor.getCalibration(&system, &gyro, &accel, &mag);
}
