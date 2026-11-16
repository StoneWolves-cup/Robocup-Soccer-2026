#ifndef OMNI_H
#define OMNI_H

#include <Arduino.h>


struct ComandosMotores
{
    // Comandos normalizados na faixa [-1, 1], na ordem usada por Motores.
    float m1;
    float m2;
    float m3;
    float m4;
};


class Omni
{
private:

    // Limita o valor depois da mistura para evitar PWM fora da faixa valida.
    float normalizarValor(
        float valor
    );


public:

    Omni();

    // Converte translacao (x esquerda, y frente) e rotacao em velocidades
    // de roda mecanum; normaliza o conjunto preservando a proporcao entre rodas.
    ComandosMotores calcular(
        float x,
        float y,
        float rotacao
    );
};

#endif