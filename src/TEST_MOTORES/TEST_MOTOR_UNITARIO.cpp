#include <Arduino.h>

// M1 no L298N 1
constexpr uint8_t M1_PWM = 15;
constexpr uint8_t M1_IN1 = 2;
constexpr uint8_t M1_IN2 = 4;
constexpr uint32_t TEMPO_SENTIDO_MS = 2000;
constexpr uint32_t TEMPO_PARADO_MS = 1000;
constexpr uint8_t POTENCIA_MAXIMA = HIGH; // EN em HIGH aplica 100% do sinal.

void setup()
{
    pinMode(M1_PWM, OUTPUT);
    pinMode(M1_IN1, OUTPUT);
    pinMode(M1_IN2, OUTPUT);

    digitalWrite(M1_PWM, LOW);
}

void loop()
{
    // Um sentido: HIGH / LOW.
    digitalWrite(M1_IN1, HIGH);
    digitalWrite(M1_IN2, LOW);
    digitalWrite(M1_PWM, POTENCIA_MAXIMA);
    delay(TEMPO_SENTIDO_MS);

    // Para antes de inverter o sentido.
    digitalWrite(M1_PWM, LOW);
    delay(TEMPO_PARADO_MS);

    // Sentido oposto: LOW / HIGH.
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, HIGH);
    digitalWrite(M1_PWM, POTENCIA_MAXIMA);
    delay(TEMPO_SENTIDO_MS);

    // Para antes de repetir o ciclo.
    digitalWrite(M1_PWM, LOW);
    delay(TEMPO_PARADO_MS);
}
