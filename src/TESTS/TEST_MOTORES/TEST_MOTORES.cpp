#include <Arduino.h>
#include <MotoresTeste.h>

Motores motores;
Omni omni;

void setup() {
    Serial.begin(115200);
    motores.begin();
    motores.parar();
    Serial.println("Teste de translacao: frente, esquerda, tras, direita.");
}

void loop() {
    executarTesteTranslacaoMotores(motores, omni);
}
