#ifndef MOTORES_H
#define MOTORES_H

#include "Omni.h"
#include "ConfigPinos.h"


class Motores
{
private:

// ========================================================
// TB6612FNG 1
// ========================================================

    static constexpr uint8_t M1_PWM  = ConfigPinos::Body::Motores::M1_PWM;
    static constexpr uint8_t M1_IN1 = ConfigPinos::Body::Motores::M1_IN1;
    static constexpr uint8_t M1_IN2 = ConfigPinos::Body::Motores::M1_IN2;

    // M3
    static constexpr uint8_t M3_PWM  = ConfigPinos::Body::Motores::M3_PWM;
    static constexpr uint8_t M3_IN1 = ConfigPinos::Body::Motores::M3_IN1;
    static constexpr uint8_t M3_IN2 = ConfigPinos::Body::Motores::M3_IN2;


    // ========================================================
    // TB6612FNG 2
    // ========================================================

    // M4
    static constexpr uint8_t M4_PWM = ConfigPinos::Body::Motores::M4_PWM;
    static constexpr uint8_t M4_IN1 = ConfigPinos::Body::Motores::M4_IN1;
    static constexpr uint8_t M4_IN2 = ConfigPinos::Body::Motores::M4_IN2;

    // M2
    static constexpr uint8_t M2_PWM  = ConfigPinos::Body::Motores::M2_PWM;
    static constexpr uint8_t M2_IN1 = ConfigPinos::Body::Motores::M2_IN1;
    static constexpr uint8_t M2_IN2 = ConfigPinos::Body::Motores::M2_IN2;

    // Fatores de montagem: permitem corrigir uma roda ligada ao contrario sem
    // alterar a convencao de movimento usada pela cinemática omnidirecional.
    static constexpr float INV_M1 = 1.0f;
    static constexpr float INV_M2 = 1.0f;
    static constexpr float INV_M3 = 1.0f;
    static constexpr float INV_M4 = 1.0f;


    // ========================================================
    // MOTOR INDIVIDUAL
    // ========================================================

    // Converte comando assinado em direcao e PWM para um canal do driver.
    void controlarMotor(
        uint8_t en,
        uint8_t in1,
        uint8_t in2,
        float comando
    );


public:

    Motores();

    // Configura os pinos do driver e inicia em estado parado por seguranca.
    void begin();

    // Desabilita PWM e direcao de todas as rodas.
    void parar();

    // Aplica os comandos normalizados m1..m4; valores sao limitados a [-1, 1].
    void definir(
        float m1,
        float m2,
        float m3,
        float m4
    );
};

#endif
