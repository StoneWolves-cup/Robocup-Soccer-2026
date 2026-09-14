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


public:

    PID(
        float kp,
        float ki,
        float kd,
        float outputMin,
        float outputMax
    );


    void reset();


    float calculate(
        float error,
        float gyroZ,
        float dt
    );
};

#endif