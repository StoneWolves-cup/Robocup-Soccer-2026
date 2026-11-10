#include <PID.h>


// ============================================================
// CONSTRUTOR
// ============================================================

PID::PID(
    float kp,
    float ki,
    float kd,
    float outputMin,
    float outputMax
)
{
    this->kp =
        kp;

    this->ki =
        ki;

    this->kd =
        kd;


    this->outputMin =
        outputMin;

    this->outputMax =
        outputMax;


    integralLimit = 100.0f;
    reset();
}

void PID::setIntegralLimit(float limit)
{
    integralLimit = fabs(limit);
}


// ============================================================
// RESET
// ============================================================

void PID::reset()
{
    integral =
        0.0f;
}


// ============================================================
// CALCULATE
// ============================================================

float PID::calculate(
    float error,
    float gyroZDegreesPerSecond,
    float dt
)
{
    if (
        dt <= 0.0f
    )
    {
        return 0.0f;
    }


    // ========================================================
    // PROPORCIONAL
    // ========================================================

    float P =
        kp * error;


    // ========================================================
    // INTEGRAL
    // ========================================================

    integral +=
        error * dt;


    // ========================================================
    // ANTI WIND-UP
    // ========================================================

    if (
        integral > integralLimit
    )
    {
        integral =
            integralLimit;
    }


    if (
        integral < -integralLimit
    )
    {
        integral =
            -integralLimit;
    }


    float I =
        ki * integral;


    // ========================================================
    // DERIVATIVO
    // ========================================================

    float D =
        -kd * gyroZDegreesPerSecond;


    // ========================================================
    // PID
    // ========================================================

    float output =
        P + I + D;


    // ========================================================
    // LIMITES
    // ========================================================

    if (
        output > outputMax
    )
    {
        output =
            outputMax;
    }


    if (
        output < outputMin
    )
    {
        output =
            outputMin;
    }


    return output;
}