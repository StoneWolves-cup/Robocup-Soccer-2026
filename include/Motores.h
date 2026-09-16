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
    static constexpr uint8_t M1_EN  = 33;
    static constexpr uint8_t M1_IN1 = 25;
    static constexpr uint8_t M1_IN2 = 26;

    // M3
    static constexpr uint8_t M3_EN  = 12;
    static constexpr uint8_t M3_IN1 = 27;
    static constexpr uint8_t M3_IN2 = 14;


    // ========================================================
    // L298N 2
    // ========================================================

    // M4
    static constexpr uint8_t M4_EN  = 5;
    static constexpr uint8_t M4_IN1 = 17;
    static constexpr uint8_t M4_IN2 = 16;

    // M2
    static constexpr uint8_t M2_EN  = 15;
    static constexpr uint8_t M2_IN1 = 4;
    static constexpr uint8_t M2_IN2 = 2;

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