#include <Arduino.h>
#include <Motores.h>
#include <Omni.h>

namespace {
constexpr uint32_t TEMPO_MOVIMENTO_MS = 5000;
constexpr uint32_t TEMPO_PARADO_MS = 1000;
constexpr float POTENCIA_TESTE = 0.35f;

Motores motores;
Omni omni;

void executarMovimento(const char *nome, float x, float y) {
    Serial.printf("Movimento: %s\n", nome);

    const ComandosMotores rodas = omni.calcular(x, y, 0.0f);
    motores.definir(rodas.m1 * POTENCIA_TESTE,
                    rodas.m2 * POTENCIA_TESTE,
                    rodas.m3 * POTENCIA_TESTE,
                    rodas.m4 * POTENCIA_TESTE);
    delay(TEMPO_MOVIMENTO_MS);

    motores.parar();
    delay(TEMPO_PARADO_MS);
}
} // namespace

void setup() {
    Serial.begin(115200);
    motores.begin();
    motores.parar();
    Serial.println("Teste de translacao: frente, esquerda, tras, direita.");
}

void loop() {
    executarMovimento("frente", 0.0f, 1.0f);
    executarMovimento("esquerda", 1.0f, 0.0f);
    executarMovimento("tras", 0.0f, -1.0f);
    executarMovimento("direita", -1.0f, 0.0f);
    delay(2000);
}
