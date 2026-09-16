#ifndef BNO055_SENSOR_H
#define BNO055_SENSOR_H

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>

class BNO055
{
public:
    BNO055();

    bool begin();
    void update();
    void zero();

    float getHeading() const;
    float getGyroZ() const;

private:
    Adafruit_BNO055 sensor;
    sensors_event_t orientationEvent;
    sensors_event_t gyroEvent;
    float headingZero;
    float heading;
    float gyroZDegreesPerSecond;
};

#endif
