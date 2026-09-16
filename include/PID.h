#ifndef PID_H
#define PID_H

#include <Arduino.h>


class PID
{
private:

    float kp;
    float ki;
    float kd;


    float integral;


    float outputMin;
    float outputMax;
    float integralLimit;


public:

    PID(
        float kp,
        float ki,
        float kd,
        float outputMin,
        float outputMax
    );


    void reset();

    void setIntegralLimit(float limit);

    float calculate(
        float error,
        float gyroZDegreesPerSecond,
        float dt
    );
};

#endif