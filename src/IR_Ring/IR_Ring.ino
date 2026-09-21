#include <Arduino.h>

#include "IRRing.h"
#include "IRTask.h"


// ============================================================
// PINOS DOS 8 TSSP4038
//
// ESP32 DevKit V1
//
// Sensor físico:
//
// A ordem abaixo define os ângulos 0°, 45°, ..., 315°.
// Ajuste a ordem dos GPIOs para coincidir com a montagem mecânica.
//
// O TSSP4038 possui saída ativa em nível baixo; o processamento
// conta as bordas FALLING geradas pela portadora de 38 kHz.
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
    2
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
        "     8 TSSP4038 - vetor ponderado"
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
    // Filtro exponencial do vetor
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

    ring.setDetectionThreshold(2);


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


            Serial.print(" | Sensor max: ");
            Serial.print(result.strongestSensor);
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
