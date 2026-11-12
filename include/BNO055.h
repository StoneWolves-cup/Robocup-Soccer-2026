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

    bool begin(uint8_t sda = ConfigPinos::Head::BNO055::SDA,
               uint8_t scl = ConfigPinos::Head::BNO055::SCL);
    void update();
    void zero();
    void calibrateNorth();

    float getHeading() const;
    float getGyroZ() const;
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
