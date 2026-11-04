#include <Arduino.h>
#include <Motores.h>

Motores motores;

// Velocidade fixa de teste: 50%.
constexpr float VELOCIDADE_HORARIA = 0.5f;

void setup()
{
    Serial.begin(115200);
    motores.begin();

    // Aciona os quatro motores continuamente no sentido positivo do driver.
    motores.definir(
        VELOCIDADE_HORARIA,
        VELOCIDADE_HORARIA,
        VELOCIDADE_HORARIA,
        VELOCIDADE_HORARIA
    );
    Serial.println("Teste: M1, M2, M3 e M4 girando continuamente a 50%.");
}

void loop()
{
    // Sem comandos pelo Monitor Serial. Os motores continuam acionados.
}
