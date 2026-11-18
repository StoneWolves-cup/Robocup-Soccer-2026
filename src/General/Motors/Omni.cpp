#include <Omni.h>
#include <math.h>

namespace {
float maiorComponenteTranslacao(float x, float y) {
    return fmaxf(fabsf(y - x), fabsf(y + x));
}
}  // namespace

// ============================================================
// CONSTRUTOR
// ============================================================

Omni::Omni()
{
}


// ============================================================
// NORMALIZAÇÃO
// ============================================================

float Omni::normalizarValor(
    float valor
)
{
    // O clamp final protege os limites do PWM mesmo se a entrada for invalida.
    if (valor > 1.0f)
        return 1.0f;

    if (valor < -1.0f)
        return -1.0f;

    return valor;
}


// ============================================================
// CINEMÁTICA OMNIDIRECIONAL
// ============================================================

ComandosMotores Omni::calcular(
    float x,
    float y,
    float rotacao
)
{
    ComandosMotores motores;


    // ========================================================
    // CINEMÁTICA
    // ========================================================

    // Reserva primeiro a faixa completa do comando de rotacao. A translacao
    // ocupa apenas o espaco restante, sem normalizacao posterior que reduza
    // a correcao angular solicitada pelo controlador de rumo.
    if (rotacao > 1.0f) rotacao = 1.0f;
    if (rotacao < -1.0f) rotacao = -1.0f;
    const float translacaoMaxima = maiorComponenteTranslacao(x, y);
    const float espacoTranslacao = 1.0f - fabsf(rotacao);
    const float escala = translacaoMaxima > espacoTranslacao &&
                         translacaoMaxima > 0.0f
        ? espacoTranslacao / translacaoMaxima
        : 1.0f;
    x *= escala;
    y *= escala;

    // Disposicao fisica:
    //   M1 dianteira esquerda     M2 dianteira direita
    //   M4 traseira esquerda      M3 traseira direita
    //
    // Para rodas mecanum em X, uma translacao lateral usa pares
    // diagonais. x positivo significa esquerda, como no Body/main.cpp.
    // y positivo significa frente; rotacao positiva segue a convencao
    // atual do controlador de rumo.
    motores.m1 = y - x + rotacao; // dianteira esquerda
    motores.m2 = y + x - rotacao; // dianteira direita
    motores.m4 = y + x + rotacao; // traseira esquerda
    motores.m3 = y - x - rotacao; // traseira direita


    // ========================================================
    // NORMALIZAÇÃO FINAL
    // ========================================================

    motores.m1 =
        normalizarValor(motores.m1);

    motores.m2 =
        normalizarValor(motores.m2);

    motores.m3 =
        normalizarValor(motores.m3);

    motores.m4 =
        normalizarValor(motores.m4);


    return motores;
}
