#ifndef MOTORES_H
#define MOTORES_H

#include "Omni.h"


class Motores
{
private:

    // ========================================================
    // L298N 1
    // ========================================================

    // M1
    static constexpr uint8_t M1_PWM  = 13;
    static constexpr uint8_t M1_IN1 = 12;
    static constexpr uint8_t M1_IN2 = 14;

    // M3
    static constexpr uint8_t M3_PWM  = 25;
    static constexpr uint8_t M3_IN1 = 27;
    static constexpr uint8_t M3_IN2 = 26;


    // ========================================================
    // L298N 2
    // ========================================================

    // M4
    static constexpr uint8_t M4_PWM = 15;
    static constexpr uint8_t M4_IN1 = 2;
    static constexpr uint8_t M4_IN2 = 4;

    // M2
    static constexpr uint8_t M2_PWM  = 5;
    static constexpr uint8_t M2_IN1 = 16;
    static constexpr uint8_t M2_IN2 = 17;

    // Ajuste para inverter individualmente um motor, se necessario.
    static constexpr float INV_M1 = 1.0f;
    static constexpr float INV_M2 = 1.0f;
    static constexpr float INV_M3 = 1.0f;
    static constexpr float INV_M4 = 1.0f;


    // ========================================================
    // MOTOR INDIVIDUAL
    // ========================================================

    void controlarMotor(
        uint8_t en,
        uint8_t in1,
        uint8_t in2,
        float comando
    );


public:

    Motores();

    void begin();

    void parar();

    void definir(
        float m1,
        float m2,
        float m3,
        float m4
    );
};

#endif