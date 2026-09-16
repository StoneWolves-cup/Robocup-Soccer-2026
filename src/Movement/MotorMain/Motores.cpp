#include <Motores.h>


// ============================================================
// CONSTRUTOR
// ============================================================

Motores::Motores()
{
}


// ============================================================
// BEGIN
// ============================================================

void Motores::begin()
{
    pinMode(M1_EN, OUTPUT);
    pinMode(M1_IN1, OUTPUT);
    pinMode(M1_IN2, OUTPUT);

    pinMode(M2_EN, OUTPUT);
    pinMode(M2_IN1, OUTPUT);
    pinMode(M2_IN2, OUTPUT);

    pinMode(M3_EN, OUTPUT);
    pinMode(M3_IN1, OUTPUT);
    pinMode(M3_IN2, OUTPUT);

    pinMode(M4_EN, OUTPUT);
    pinMode(M4_IN1, OUTPUT);
    pinMode(M4_IN2, OUTPUT);


    parar();
}


// ============================================================
// MOTOR INDIVIDUAL
// ============================================================

void Motores::controlarMotor(
    uint8_t en,
    uint8_t in1,
    uint8_t in2,
    float comando
)
{
    const float zonaMorta = 0.02f;

    // --------------------------------------------------------
    // PARADO
    // --------------------------------------------------------

    if (fabs(comando) < zonaMorta)
    {
        digitalWrite(en, LOW);

        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);

        return;
    }

    int velocidade = (int)(fabs(comando) * 255.0f);

    if (velocidade > 255)
    {
        velocidade = 255;
    }


    // --------------------------------------------------------
    // SENTIDO POSITIVO
    // --------------------------------------------------------

    if (comando > 0)
    {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);

        analogWrite(en, velocidade);

        return;
    }


    // --------------------------------------------------------
    // SENTIDO NEGATIVO
    // --------------------------------------------------------

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    analogWrite(en, velocidade);
}


// ============================================================
// DEFINIR OS 4 MOTORES
// ============================================================

void Motores::definir(
    float m1,
    float m2,
    float m3,
    float m4
)
{
    controlarMotor(
        M1_EN,
        M1_IN1,
        M1_IN2,
        m1 * INV_M1
    );


    controlarMotor(
        M2_EN,
        M2_IN1,
        M2_IN2,
        m2 * INV_M2
    );


    controlarMotor(
        M3_EN,
        M3_IN1,
        M3_IN2,
        m3 * INV_M3
    );


    controlarMotor(
        M4_EN,
        M4_IN1,
        M4_IN2,
        m4 * INV_M4
    );
}


// ============================================================
// PARAR
// ============================================================

void Motores::parar()
{
    digitalWrite(M1_EN, LOW);
    digitalWrite(M2_EN, LOW);
    digitalWrite(M3_EN, LOW);
    digitalWrite(M4_EN, LOW);

    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, LOW);

    digitalWrite(M2_IN1, LOW);
    digitalWrite(M2_IN2, LOW);

    digitalWrite(M3_IN1, LOW);
    digitalWrite(M3_IN2, LOW);

    digitalWrite(M4_IN1, LOW);
    digitalWrite(M4_IN2, LOW);
}