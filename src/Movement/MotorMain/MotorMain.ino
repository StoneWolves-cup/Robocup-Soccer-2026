#include <Motores.h>
#include <BNO055.h>
#include <PID.h>
float calcularErroAngular(float desejado, float atual);
void aplicarMovimento();

// ============================================================
// OBJETOS
// ============================================================

BNO055 sensorBNO;

PID pidHeading(
    0.018f,     // KP: erro em graus
    0.001f,     // KI: erro acumulado
    0.025f,     // KD: velocidade angular em graus/s
    -1.0f,
    1.0f
);

Motores motores;

Omni omni;


// ============================================================
// HEADING
// ============================================================

float headingDesejado = 0.0f;

float pidOutput = 0.0f;


// ============================================================
// MOVIMENTO
// ============================================================
//
// X = lateral
// Y = frente
//
// Esses valores são normalizados:
//
// -1 → máximo para um lado
//  0 → parado
// +1 → máximo para outro lado
//
// ============================================================

float movimentoX = 0.0f;

float movimentoY = 1.0f;


// ============================================================
// ROTAÇÃO MANUAL
// ============================================================

float rotacaoManual = 0.0f;

float calcularCorrecaoBNO()
{
    return pidOutput;
}


// ============================================================
// TEMPOS
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


    while (erro > 180.0f)
    {
        erro -= 360.0f;
    }


    while (erro < -180.0f)
    {
        erro += 360.0f;
    }


    return erro;
}


// ============================================================
// APLICAR MOVIMENTO
// ============================================================

void aplicarMovimento()
{
    float correcao =
        calcularCorrecaoBNO();

    // A correcao e somada ao comando manual para manter o heading
    // durante o deslocamento, em vez de interromper a translacao.
    float rotacao = rotacaoManual + correcao;

    ComandosMotores comando =
        omni.calcular(
            movimentoX,
            movimentoY,
            rotacao
        );


    motores.definir(
        comando.m1,
        comando.m2,
        comando.m3,
        comando.m4
    );
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
        "========================================"
    );

    Serial.println(
        " STONE WOLVES"
    );

    Serial.println(
        " BNO055 + PID + OMNI + 4 MOTORES"
    );

    Serial.println(
        "========================================"
    );


    // ========================================================
    // MOTORES
    // ========================================================

    motores.begin();


    // ========================================================
    // BNO
    // ========================================================

    if (!sensorBNO.begin())
    {
        Serial.println(
            "ERRO AO INICIAR BNO055!"
        );


        motores.parar();


        while (true)
        {
            delay(1000);
        }
    }


    // ========================================================
    // ESTABILIZAÇÃO
    // ========================================================

    Serial.println();

    Serial.println(
        "Deixe o robo completamente parado..."
    );

    delay(1000);


    // ========================================================
    // ZERO
    // ========================================================

    sensorBNO.zero();


    // ========================================================
    // PID
    // ========================================================

    pidHeading.reset();
    pidHeading.setIntegralLimit(35.0f);


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
    // BNO055
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


    if (dt >= 0.01f)
    {
        ultimoPID =
            agora;


        // ----------------------------------------------------
        // HEADING
        // ----------------------------------------------------

        float heading =
            sensorBNO.getHeading();


        // ----------------------------------------------------
        // ERRO
        // ----------------------------------------------------

        float erro =
            calcularErroAngular(
                headingDesejado,
                heading
            );


        // ----------------------------------------------------
        // GYRO
        // ----------------------------------------------------

        float gyroZ = sensorBNO.getGyroZ();


        // ----------------------------------------------------
        // PID
        // ----------------------------------------------------

        pidOutput =
            pidHeading.calculate(
                erro,
                gyroZ,
                dt
            );


        // ----------------------------------------------------
        // MOVIMENTO
        // ----------------------------------------------------

        aplicarMovimento();
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


        Serial.print("H:");

        Serial.print(
            heading,
            2
        );


        Serial.print(" E:");

        Serial.print(
            erro,
            2
        );


        Serial.print(" GZ:");

        Serial.print(
            sensorBNO.getGyroZ(),
            3
        );


        Serial.print(" PID:");

        Serial.println(
            pidOutput,
            3
        );
    }
}