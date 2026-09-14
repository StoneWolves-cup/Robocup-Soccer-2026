#include <Arduino.h>

#include "IRRing.h"
#include "IRTask.h"


// ============================================================
// PINOS DOS 15 TSOP2240
//
// ESP32 DevKit V1
//
// Sensor físico:
//
// 1  -> GPIO 23
// 2  -> GPIO 19
// 3  -> GPIO 18
// 4  -> GPIO 5
// 5  -> GPIO 17  (RX2)
//
// 6  -> GPIO 16  (TX2)
// 7  -> GPIO 4
// 8  -> GPIO 2
// 9  -> GPIO 25
// 10 -> GPIO 33
//
// 11 -> GPIO 32
// 12 -> GPIO 34
// 13 -> GPIO 39 (VN)
// 14 -> GPIO 36 (VP)
// 15 -> GPIO 35
//
// A posição 16 é VIRTUAL.
//
// Ela fica entre o sensor 15 e o sensor 1:
//                  337,5°
//
// Não existe GPIO para o sensor 16.
// ============================================================

const uint8_t sensorPins[
    IRRing::PHYSICAL_SENSOR_COUNT
] =
{
    23,
    19,
    18,
    5,
    17,

    16,
    4,
    2,
    25,
    33,

    32,
    34,
    39,
    36,
    35
};


// ============================================================
// OBJETOS
// ============================================================

IRRing ring(sensorPins);

IRTask irTask(ring);


// ============================================================
// SETUP
// ============================================================

void setup()
{
    // --------------------------------------------------------
    // Serial
    // --------------------------------------------------------

    Serial.begin(115200);

    delay(500);


    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        "     IR RING - ROBOCUP SOCCER"
    );

    Serial.println(
        "     ESP32 DEVKIT V1"
    );

    Serial.println(
        "     15 TSOP2240 + 1 POSICAO VIRTUAL"
    );

    Serial.println(
        "========================================"
    );


    // --------------------------------------------------------
    // Configuração do IRRing
    // --------------------------------------------------------

    if(!ring.begin())
    {
        Serial.println(
            "ERRO: Falha ao iniciar IRRing!"
        );

        while(true)
        {
            delay(1000);
        }
    }


    Serial.println(
        "IRRing iniciado."
    );


    // --------------------------------------------------------
    // Configurações
    // --------------------------------------------------------

    // Janela de medição:
    //
    // 5 ms
    //
    // O Timer do IRRing será configurado para
    // acordar a IRTask a cada 5 ms.

    irTask.setPeriodMs(5);


    // --------------------------------------------------------
    // Filtro exponencial
    //
    // Quanto maior:
    //     mais estável
    //
    // Quanto menor:
    //     mais rápido
    //
    // 0.65 é um ponto inicial.
    // --------------------------------------------------------

    ring.setFilterAlpha(0.65f);


    // --------------------------------------------------------
    // Limite mínimo para considerar que existe bola.
    // --------------------------------------------------------

    ring.setDetectionThreshold(10);


    // --------------------------------------------------------
    // Inicia a FreeRTOS Task
    // --------------------------------------------------------

    if(!irTask.begin())
    {
        Serial.println(
            "ERRO: Falha ao criar IRTask!"
        );

        while(true)
        {
            delay(1000);
        }
    }


    Serial.println(
        "IRTask iniciada."
    );


    Serial.println(
        "Sistema pronto."
    );

    Serial.println();
}


// ============================================================
// LOOP PRINCIPAL
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // O loop principal fica livre.
    //
    // Aqui futuramente podemos colocar:
    //
    // - Controle dos motores
    // - ESP-NOW
    // - Estratégia
    // - Dribbler
    // - Odômetro
    // - BNO055
    // - Ultrassônicos
    //
    // A leitura IR NÃO precisa ficar aqui.
    // --------------------------------------------------------

    static uint32_t lastPrint = 0;


    // --------------------------------------------------------
    // Debug geral a cada 100 ms
    //
    // Isso NÃO controla a leitura IR.
    // É apenas telemetria.
    // --------------------------------------------------------

    if(
        millis() - lastPrint >= 100
    )
    {
        lastPrint =
            millis();


        IRRing::Result result =
            ring.getResult();


        Serial.print(
            "IR | "
        );


        if(result.detected)
        {
            Serial.print(
                "BOLA"
            );

            Serial.print(
                " | Angulo: "
            );

            Serial.print(
                result.angle,
                2
            );

            Serial.print(
                " deg"
            );

            Serial.print(
                " | Forca: "
            );

            Serial.print(
                result.strength,
                2
            );

            Serial.print(
                " | Pulsos: "
            );

            Serial.print(
                result.totalPulses
            );

            Serial.print(
                " | Sensor: "
            );

            Serial.print(
                result.logicalSensor
            );


            // ------------------------------------------------
            // Posição virtual 16
            // ------------------------------------------------

            if(
                result.virtualSensor16
            )
            {
                Serial.print(
                    " | VIRTUAL 16"
                );
            }
        }
        else
        {
            Serial.print(
                "SEM BOLA"
            );
        }


        Serial.println();
    }


    // --------------------------------------------------------
    // Não precisamos de delay para o IR.
    //
    // A IRTask está rodando separadamente.
    // --------------------------------------------------------

    delay(1);
}
