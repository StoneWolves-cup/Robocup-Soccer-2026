#include <Omni.h>


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

    motores.m1 =
        y + x + rotacao;


    motores.m3 =
        y - x - rotacao;


    motores.m4 =
        y - x + rotacao;


    motores.m2 =
        y + x - rotacao;


    // ========================================================
    // NORMALIZAÇÃO
    // ========================================================

    float maior =
        fabs(motores.m1);


    maior =
        max(
            maior,
            fabs(motores.m2)
        );

    maior =
        max(
            maior,
            fabs(motores.m3)
        );

    maior =
        max(
            maior,
            fabs(motores.m4)
        );


    if (maior > 1.0f)
    {
        motores.m1 /= maior;
        motores.m2 /= maior;
        motores.m3 /= maior;
        motores.m4 /= maior;
    }


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