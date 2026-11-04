#include <Arduino.h>

// Pinos dos quatro motores nos dois drivers L298N.
constexpr uint8_t M1_EN = 15;
constexpr uint8_t M1_IN1 = 2;
constexpr uint8_t M1_IN2 = 4;

constexpr uint8_t M3_EN = 5;
constexpr uint8_t M3_IN1 = 16;
constexpr uint8_t M3_IN2 = 17;

constexpr uint8_t M4_EN = 13;
constexpr uint8_t M4_IN1 = 12;
constexpr uint8_t M4_IN2 = 14;

constexpr uint8_t M2_EN = 25;
constexpr uint8_t M2_IN1 = 27;
constexpr uint8_t M2_IN2 = 26;

constexpr uint32_t TEMPO_SENTIDO_MS = 2000;
constexpr uint32_t TEMPO_PARADO_MS = 1000;
constexpr uint8_t POTENCIA_MAXIMA = HIGH; // EN em HIGH aplica 100% do sinal.

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
    const uint8_t nivel = habilitado ? POTENCIA_MAXIMA : LOW;
    digitalWrite(M1_EN, nivel);
    digitalWrite(M2_EN, nivel);
    digitalWrite(M3_EN, nivel);
    digitalWrite(M4_EN, nivel);
}

void setup()
{
    configurarMotor(M1_EN, M1_IN1, M1_IN2);
    configurarMotor(M2_EN, M2_IN1, M2_IN2);
    configurarMotor(M3_EN, M3_IN1, M3_IN2);
    configurarMotor(M4_EN, M4_IN1, M4_IN2);
}

void loop()
{
    // Todos os motores giram juntos no primeiro sentido, em potencia maxima.
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
