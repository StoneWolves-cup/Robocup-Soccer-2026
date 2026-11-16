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

    // Define ganhos, saturacao da saida e o limite integral inicial.
    PID(
        float kp,
        float ki,
        float kd,
        float outputMin,
        float outputMax
    );

    // Atualiza ganhos sem descartar a memoria integral atual.
    void setGains(float kp, float ki, float kd);

    // Limpa o estado acumulado para iniciar uma nova correcao sem residuo.
    void reset();

    // Aplica limite simetrico ao acumulador integral.
    void setIntegralLimit(float limit);

    // Calcula a saida PID; gyro Z fornece a velocidade para o termo derivativo.
    float calculate(
        float error,
        float gyroZDegreesPerSecond,
        float dt
    );
};

#endif