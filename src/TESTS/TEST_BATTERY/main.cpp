#include <BatteryEstimator.h>
#include <math.h>

BatteryEstimator battery;

uint32_t lastUpdateMs = 0;
uint32_t lastLogMs = 0;

void setup() {
    Serial.begin(115200);
    delay(500);

    battery.begin();
    lastUpdateMs = millis();
    lastLogMs = lastUpdateMs;

    Serial.println();
    Serial.println("=== Teste de leitura percentual da bateria ===");
    Serial.printf("ADC: GPIO%d | Divisor: R1=%.0f ohms, R2=%.0f ohms\n",
                  PIN_VBAT_ADC, DIVIDER_R1_OHM, DIVIDER_R2_OHM);
    Serial.println("Curva configurada para bateria Li-ion 1S.");
    Serial.println("Confira o divisor e a curva em BatteryEstimator.h para sua bateria.");
}

void loop() {
    const uint32_t now = millis();

    if (now - lastUpdateMs >= UPDATE_PERIOD_MS) {
        battery.update(now - lastUpdateMs);
        lastUpdateMs = now;
    }

    if (now - lastLogMs >= LOG_PERIOD_MS) {
        lastLogMs = now;

        const float voltage = battery.getFilteredVoltage();
        const float percentage = battery.getPercentCombined();
        const bool readingValid = isfinite(voltage) && voltage > 0.1f &&
                                  isfinite(percentage) &&
                                  percentage >= 0.0f && percentage <= 100.0f;

        Serial.printf(
            "%s | Tensao: %.3f V | Percentual: %.1f %% | "
            "SOC por tensao: %.1f %% | SOC por coulomb: %.1f %%\n",
            readingValid ? "LEITURA OK" : "FALHA NA LEITURA",
            voltage,
            percentage,
            battery.getPercentVoltageBased(),
            battery.getPercentCoulomb());

        if (!readingValid) {
            Serial.println("Verifique a bateria, o divisor resistivo e o pino ADC.");
        }
    }
}
