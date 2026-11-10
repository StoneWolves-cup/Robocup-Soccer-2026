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
