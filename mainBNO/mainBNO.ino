#include <Arduino.h>

#include "BNO055.h"
#include "PID.h"


// ============================================================
// OBJETOS
// ============================================================

BNO055 sensorBNO;


PID pidHeading(
    1.5f,       // KP
    0.0f,       // KI
    0.10f,      // KD
    -255.0f,
    255.0f
);


// ============================================================
// CONTROLE
// ============================================================

float headingDesejado = 0.0f;

float pidOutput = 0.0f;


// ============================================================
// TEMPO
// ============================================================

uint32_t ultimoSensor = 0;

uint32_t ultimoPID = 0;

uint32_t ultimoPrint = 0;


// ============================================================
// ERRO ANGULAR
// ============================================================

float calcularErroAngular(
    float desejado,
    float atual
)
{
    float erro =
        desejado - atual;


    while (
        erro > 180.0f
    )
    {
        erro -= 360.0f;
    }


    while (
        erro < -180.0f
    )
    {
        erro += 360.0f;
    }


    return erro;
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(
        115200
    );


    delay(1000);


    Serial.println();

    Serial.println(
        "===================================="
    );

    Serial.println(
        "       ROBOCUP - BNO055 + PID"
    );

    Serial.println(
        "===================================="
    );


    // ========================================================
    // BNO
    // ========================================================

    if (
        !sensorBNO.begin()
    )
    {
        Serial.println(
            "ERRO AO INICIAR BNO055!"
        );


        while (true)
        {
            delay(1000);
        }
    }


    // ========================================================
    // ESPERA CALMA
    // ========================================================

    Serial.println();

    Serial.println(
        "Deixe o BNO parado..."
    );

    delay(1000);


    // ========================================================
    // ZERO
    // ========================================================

    sensorBNO.update();

    sensorBNO.zero();


    // ========================================================
    // PID
    // ========================================================

    pidHeading.reset();


    // ========================================================
    // TEMPOS
    // ========================================================

    ultimoSensor =
        millis();

    ultimoPID =
        micros();

    ultimoPrint =
        millis();


    Serial.println();

    Serial.println(
        "Sistema pronto."
    );

    Serial.println(
        "Heading desejado: 0 graus"
    );

    Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // ========================================================
    // ATUALIZA BNO
    //
    // 100 Hz
    // ========================================================

    if (
        millis() -
        ultimoSensor >= 10
    )
    {
        ultimoSensor =
            millis();


        sensorBNO.update();
    }


    // ========================================================
    // PID
    //
    // 100 Hz
    // ========================================================

    uint32_t agora =
        micros();


    float dt =
        (
            agora -
            ultimoPID
        )
        /
        1000000.0f;


    if (
        dt >= 0.01f
    )
    {
        ultimoPID =
            agora;


        // ----------------------------------------------------
        // Heading
        // ----------------------------------------------------

        float heading =
            sensorBNO.getHeading();


        // ----------------------------------------------------
        // Erro
        // ----------------------------------------------------

        float erro =
            calcularErroAngular(
                headingDesejado,
                heading
            );


        // ----------------------------------------------------
        // Gyro
        // ----------------------------------------------------

        float gyroZ =
            sensorBNO.getGyroZ();


        // ----------------------------------------------------
        // PID
        // ----------------------------------------------------

        pidOutput =
            pidHeading.calculate(
                erro,
                gyroZ,
                dt
            );
    }


    // ========================================================
    // SERIAL
    // ========================================================

    if (
        millis() -
        ultimoPrint >= 100
    )
    {
        ultimoPrint =
            millis();


        float heading =
            sensorBNO.getHeading();


        float erro =
            calcularErroAngular(
                headingDesejado,
                heading
            );


        Serial.print(
            "Heading: "
        );

        Serial.print(
            heading,
            2
        );


        Serial.print(
            " | Desejado: "
        );

        Serial.print(
            headingDesejado,
            2
        );


        Serial.print(
            " | Erro: "
        );

        Serial.print(
            erro,
            2
        );


        Serial.print(
            " | Gyro Z: "
        );

        Serial.print(
            sensorBNO.getGyroZ(),
            3
        );


        Serial.print(
            " | PID: "
        );

        Serial.println(
            pidOutput,
            2
        );
    }
}