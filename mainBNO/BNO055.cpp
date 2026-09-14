#include "BNO055.h"


// ============================================================
// CONSTRUTOR
// ============================================================

BNO055::BNO055()
    : bno(
        55,
        ADDRESS,
        &Wire
    )
{
}


// ============================================================
// BEGIN
// ============================================================

bool BNO055::begin()
{
    Serial.println();
    Serial.println(
        "================================"
    );

    Serial.println(
        "        BNO055 ROBOT IMU"
    );

    Serial.println(
        "================================"
    );


    // --------------------------------------------------------
    // I2C
    // --------------------------------------------------------

    Wire.begin(
        SDA_PIN,
        SCL_PIN
    );

    Wire.setClock(
        400000
    );


    // --------------------------------------------------------
    // INICIALIZA BNO
    // --------------------------------------------------------

    if (!bno.begin())
    {
        Serial.println(
            "ERRO: BNO055 nao encontrado!"
        );

        initialized = false;

        return false;
    }


    Serial.println(
        "BNO055 encontrado!"
    );


    // --------------------------------------------------------
    // ESPERA BOOT
    // --------------------------------------------------------

    delay(100);


    // --------------------------------------------------------
    // CRISTAL EXTERNO
    // --------------------------------------------------------

    bno.setExtCrystalUse(
        true
    );


    // --------------------------------------------------------
    // MODO NDOF
    // --------------------------------------------------------

    bno.setMode(
        OPERATION_MODE_NDOF
    );


    delay(100);


    // --------------------------------------------------------
    // PRIMEIRA LEITURA
    // --------------------------------------------------------

    update();


    initialized = true;


    Serial.println(
        "BNO055 pronto!"
    );


    return true;
}


// ============================================================
// UPDATE
// ============================================================

void BNO055::update()
{
    sensors_event_t orientation;

    sensors_event_t gyro;


    // --------------------------------------------------------
    // ORIENTAÇÃO
    // --------------------------------------------------------

    bno.getEvent(
        &orientation,
        Adafruit_BNO055::VECTOR_EULER
    );


    rawHeading =
        orientation.orientation.x;

    rawRoll =
        orientation.orientation.y;

    rawPitch =
        orientation.orientation.z;


    // --------------------------------------------------------
    // GYRO
    // --------------------------------------------------------

    bno.getEvent(
        &gyro,
        Adafruit_BNO055::VECTOR_GYROSCOPE
    );


    gyroX =
        gyro.gyro.x;

    gyroY =
        gyro.gyro.y;

    gyroZ =
        gyro.gyro.z;


    // --------------------------------------------------------
    // NORMALIZA HEADING
    // --------------------------------------------------------

    while (
        rawHeading < 0.0f
    )
    {
        rawHeading += 360.0f;
    }


    while (
        rawHeading >= 360.0f
    )
    {
        rawHeading -= 360.0f;
    }
}


// ============================================================
// ZERO
// ============================================================

void BNO055::zero()
{
    update();


    zeroHeading =
        rawHeading;


    Serial.print(
        "Zero definido em: "
    );

    Serial.println(
        zeroHeading,
        2
    );
}


// ============================================================
// HEADING RELATIVO
// ============================================================

float BNO055::getHeading()
{
    float angle =
        rawHeading -
        zeroHeading;


    while (
        angle >= 180.0f
    )
    {
        angle -= 360.0f;
    }


    while (
        angle < -180.0f
    )
    {
        angle += 360.0f;
    }


    return angle;
}


// ============================================================
// HEADING ABSOLUTO
// ============================================================

float BNO055::getRawHeading()
{
    return rawHeading;
}


// ============================================================
// GYRO X
// ============================================================

float BNO055::getGyroX()
{
    return gyroX;
}


// ============================================================
// GYRO Y
// ============================================================

float BNO055::getGyroY()
{
    return gyroY;
}


// ============================================================
// GYRO Z
// ============================================================

float BNO055::getGyroZ()
{
    return gyroZ;
}


// ============================================================
// PITCH
// ============================================================

float BNO055::getPitch()
{
    return rawPitch;
}


// ============================================================
// ROLL
// ============================================================

float BNO055::getRoll()
{
    return rawRoll;
}


// ============================================================
// CALIBRAÇÃO
// ============================================================

void BNO055::printCalibration()
{
    uint8_t system;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;


    bno.getCalibration(
        &system,
        &gyro,
        &accel,
        &mag
    );


    Serial.print(
        "CAL | SYS:"
    );

    Serial.print(
        system
    );


    Serial.print(
        " GYRO:"
    );

    Serial.print(
        gyro
    );


    Serial.print(
        " ACC:"
    );

    Serial.print(
        accel
    );


    Serial.print(
        " MAG:"
    );

    Serial.println(
        mag
    );
}


// ============================================================
// ESTADO
// ============================================================

bool BNO055::isInitialized()
{
    return initialized;
}