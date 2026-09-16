#ifndef OMNI_H
#define OMNI_H

#include <Arduino.h>


struct ComandosMotores
{
    float m1;
    float m2;
    float m3;
    float m4;
};


class Omni
{
private:

    float normalizarValor(
        float valor
    );


public:

    Omni();


    ComandosMotores calcular(
        float x,
        float y,
        float rotacao
    );
};

#endif