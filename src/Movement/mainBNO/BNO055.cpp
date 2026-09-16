#include <BNO055.h>
#include <Wire.h>

namespace
{
constexpr uint8_t BNO_SDA = 21;
constexpr uint8_t BNO_SCL = 22;
}

BNO055::BNO055()
    : sensor(55, 0x28),
      headingZero(0.0f),
      heading(0.0f),
      gyroZDegreesPerSecond(0.0f)
{
}

bool BNO055::begin()
{
    Wire.begin(BNO_SDA, BNO_SCL);

    if (!sensor.begin(OPERATION_MODE_IMUPLUS))
    {
        return false;
    }

    delay(100);
    sensor.setExtCrystalUse(true);
    update();
    zero();
    return true;
}

void BNO055::update()
{
    sensor.getEvent(&orientationEvent);
    sensor.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);

    heading = orientationEvent.orientation.x - headingZero;
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

float BNO055::getHeading() const
{
    return heading;
}

float BNO055::getGyroZ() const
{
    return gyroZDegreesPerSecond;
}
