#include <Arduino.h>

// ========================================================
// TB6612FNG 1
// ========================================================

    static constexpr uint8_t M1_PWM  = 4;
    static constexpr uint8_t M1_IN1 = 6;
    static constexpr uint8_t M1_IN2 = 5;

    // M3
    static constexpr uint8_t M3_PWM  = 16;
    static constexpr uint8_t M3_IN1 = 7;
    static constexpr uint8_t M3_IN2 = 15;


    // ========================================================
    // TB6612FNG 2
    // ========================================================

    // M4
    static constexpr uint8_t M4_PWM = 9;
    static constexpr uint8_t M4_IN1 = 3;
    static constexpr uint8_t M4_IN2 = 46;

    // M2
    static constexpr uint8_t M2_PWM  = 37;
    static constexpr uint8_t M2_IN1 = 8;
    static constexpr uint8_t M2_IN2 = 38;

constexpr uint32_t TEMPO_SENTIDO_MS = 2000;
constexpr uint32_t TEMPO_PARADO_MS = 1000;
constexpr float POTENCIA_PERCENTUAL = 89.0f;
constexpr uint8_t POTENCIA_PWM = (255 * POTENCIA_PERCENTUAL + 50) / 100;

void configurarMotor(uint8_t en, uint8_t in1, uint8_t in2)
{
    pinMode(en, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    digitalWrite(en, LOW);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
}

void definirSentido(bool frente)
{
    const uint8_t nivelIn1 = frente ? HIGH : LOW;
    const uint8_t nivelIn2 = frente ? LOW : HIGH;

    digitalWrite(M1_IN1, nivelIn1);
    digitalWrite(M1_IN2, nivelIn2);
    digitalWrite(M2_IN1, nivelIn1);
    digitalWrite(M2_IN2, nivelIn2);
    digitalWrite(M3_IN1, nivelIn1);
    digitalWrite(M3_IN2, nivelIn2);
    digitalWrite(M4_IN1, nivelIn1);
    digitalWrite(M4_IN2, nivelIn2);
}

void habilitarMotores(bool habilitado)
{
    const uint8_t potencia = habilitado ? POTENCIA_PWM : 0;
    analogWrite(M1_PWM, potencia);
    analogWrite(M2_PWM, potencia);
    analogWrite(M3_PWM, potencia);
    analogWrite(M4_PWM, potencia);
}

void setup()
{
    configurarMotor(M1_PWM, M1_IN1, M1_IN2);
    configurarMotor(M2_PWM, M2_IN1, M2_IN2);
    configurarMotor(M3_PWM, M3_IN1, M3_IN2);
    configurarMotor(M4_PWM, M4_IN1, M4_IN2);
}

void loop()
{
    // Todos os motores giram juntos no primeiro sentido, em 67% de potencia.
    definirSentido(true);
    habilitarMotores(true);
    delay(TEMPO_SENTIDO_MS);

    habilitarMotores(false);
    delay(TEMPO_PARADO_MS);

    // Todos os motores giram juntos no sentido oposto.
    definirSentido(false);
    habilitarMotores(true);
    delay(TEMPO_SENTIDO_MS);

    habilitarMotores(false);
    delay(TEMPO_PARADO_MS);
}
