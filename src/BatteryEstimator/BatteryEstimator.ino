// ======================================================================
// Estimador de nível de bateria para ESP32
// Fases implementadas: 1 (medição), 2 (filtragem RC + digital),
// 3 (curva de calibração), 4/5 (corrente + Coulomb Counting, se
// HAS_CURRENT_SENSOR estiver ativo em BatteryEstimator.h),
// 6 (estimativa combinada simples — sem Kalman ainda).
//
// Circuito esperado (ver BatteryEstimator.h para os valores):
//
//   Bateria (+) --- R1 ---+--- R2 --- GND
//                         |
//                       ADC (PIN_VBAT_ADC)
//
//   Um capacitor C entre o nó do ADC e GND forma o filtro RC analógico
//   (H(s) = 1 / (RCs + 1)); o filtro digital (EMA) complementa ou
//   substitui esse filtro, dependendo do que você quiser comparar.
// ======================================================================

#include "../include/BatteryEstimator.h"

BatteryEstimator battery;

uint32_t lastUpdateMs = 0;
uint32_t lastLogMs = 0;

void setup() {
    Serial.begin(115200);
    delay(200);

    battery.begin();

    Serial.println();
    Serial.println("=== Estimador de bateria iniciado ===");
    Serial.printf("Tensao inicial: %.3f V | SOC inicial: %.1f %%\n",
                   battery.getFilteredVoltage(),
                   battery.getPercentCombined());

    lastUpdateMs = millis();
    lastLogMs = millis();
}

void loop() {
    uint32_t now = millis();

    if (now - lastUpdateMs >= UPDATE_PERIOD_MS) {
        uint32_t dt = now - lastUpdateMs;
        battery.update(dt);
        lastUpdateMs = now;
    }

    if (now - lastLogMs >= LOG_PERIOD_MS) {
        lastLogMs = now;

        Serial.printf(
            "Vraw=%.3fV  Vfilt=%.3fV  Voc=%.3fV  I=%.3fA  "
            "SOCv=%.1f%%  SOCq=%.1f%%  SOCcomb=%.1f%%  consumido=%.1fmAh\n",
            battery.getRawVoltage(),
            battery.getFilteredVoltage(),
            battery.getOpenCircuitVoltage(),
            battery.getCurrentA(),
            battery.getPercentVoltageBased(),
            battery.getPercentCoulomb(),
            battery.getPercentCombined(),
            battery.getConsumedMah());

        // Ponto de extensão: aqui você pode enviar battery.getPercentCombined()
        // via ESP-NOW para a ESP32 dos motores, do mesmo jeito que já faz
        // com os dados do anel de sensores IR, para exibir o nível de
        // bateria em outro lugar (LED, display, telemetria etc).
    }
}
