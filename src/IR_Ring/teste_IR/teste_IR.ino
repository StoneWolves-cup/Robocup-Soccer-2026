// ============================================================
// TESTE DE UM ÚNICO SENSOR TSOP2240
// ESP32 + TSOP2240
// Leitura por interrupção
// ============================================================

#define SENSOR_PIN 4

// Janela de medição em milissegundos
#define TEMPO_TESTE_MS 100

volatile uint32_t contadorPulsos = 0;

// ------------------------------------------------------------
// Interrupção do TSOP
// ------------------------------------------------------------
void IRAM_ATTR interrupcaoSensor()
{
    contadorPulsos++;
}

// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------
void setup()
{
    Serial.begin(115200);

    pinMode(SENSOR_PIN, INPUT);

    attachInterrupt(
        digitalPinToInterrupt(SENSOR_PIN),
        interrupcaoSensor,
        CHANGE
    );

    Serial.println();
    Serial.println("========================================");
    Serial.println("   TESTE INDIVIDUAL - TSOP2240");
    Serial.println("========================================");
    Serial.print("GPIO utilizado: ");
    Serial.println(SENSOR_PIN);
    Serial.println();
}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------
void loop()
{
    // Zera o contador antes de começar a medição
    noInterrupts();
    contadorPulsos = 0;
    interrupts();

    // Mede durante a janela definida
    delay(TEMPO_TESTE_MS);

    // Copia o contador com segurança
    noInterrupts();
    uint32_t pulsos = contadorPulsos;
    interrupts();

    // --------------------------------------------------------
    // Resultado
    // --------------------------------------------------------

    Serial.print("GPIO ");
    Serial.print(SENSOR_PIN);

    Serial.print(" | Pulsos: ");
    Serial.print(pulsos);

    if (pulsos > 0)
    {
        Serial.println(" | SINAL IR");
    }
    else
    {
        Serial.println(" | SEM SINAL");
    }

    delay(200);
}
