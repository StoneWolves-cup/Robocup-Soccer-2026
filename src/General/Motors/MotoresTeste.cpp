#include <Arduino.h>
#include <MotoresTeste.h>

namespace {
constexpr uint32_t TEMPO_MOVIMENTO_MS = 2000;
constexpr uint32_t TEMPO_PARADO_MS = 1000;
constexpr float POTENCIA_TESTE = 1.0f;

void executarMovimento(Motores &motores, Omni &omni, const char *nome,
                       float x, float y) {
    // Um eixo por vez facilita identificar inversoes mecanicas com potencia baixa.
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

void executarTesteTranslacaoMotores(Motores &motores, Omni &omni) {
    // Sequencia percorre os quatro sentidos cardeais e termina com pausa.
    executarMovimento(motores, omni, "frente", 0.0f, 1.0f);
    executarMovimento(motores, omni, "esquerda", 1.0f, 0.0f);
    executarMovimento(motores, omni, "tras", 0.0f, -1.0f);
    executarMovimento(motores, omni, "direita", -1.0f, 0.0f);
    delay(2000);
}
