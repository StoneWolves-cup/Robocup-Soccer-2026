#ifndef BATTERY_ESTIMATOR_H
#define BATTERY_ESTIMATOR_H

#include <Arduino.h>

// ======================================================================
// CONFIGURAÇÃO DE HARDWARE
// ======================================================================

// Pino ADC do ESP32 ligado ao ponto médio do divisor de tensão
#define PIN_VBAT_ADC        34

// Se houver sensor de corrente (ex: ACS712 / INA219 via leitura analógica
// simples), defina HAS_CURRENT_SENSOR. Caso contrário, o estimador roda
// apenas com Coulomb Counting desativado e usa somente a tensão (Fase 1-3).
#define HAS_CURRENT_SENSOR  0
#define PIN_CURRENT_ADC     35

// Divisor de tensão: Vadc = Vbat * R2 / (R1 + R2)
// Valores de exemplo (ajuste para o seu circuito real e confira a faixa
// segura de entrada do ADC do seu ESP32 antes de ligar a bateria).
#define DIVIDER_R1_OHM      100000.0f
#define DIVIDER_R2_OHM      33000.0f

// Sensor de corrente (ex: ACS712-05B): sensibilidade em V/A e offset em V
// no ponto de corrente zero. Ajuste conforme o datasheet/calibração real.
#define CURRENT_SENS_V_PER_A 0.185f
#define CURRENT_SENS_OFFSET_V 1.65f

// Resistência interna aproximada da bateria (ohms), usada para compensar
// a queda de tensão sob carga: V_oc = V_terminal + I * Rint
#define BATTERY_RINT_OHM    0.15f

// Capacidade nominal da bateria em mAh, usada no Coulomb Counting
#define BATTERY_CAPACITY_MAH 2200.0f

// Constante de tempo do filtro digital passa-baixas (EMA), 0 < alpha <= 1.
// Valores menores = filtro mais forte (mais lento, menos ruído).
#define FILTER_ALPHA         0.05f

// A cada quantos ms o estimador é atualizado
#define UPDATE_PERIOD_MS     200

// Período de log em ms
#define LOG_PERIOD_MS        1000

// ======================================================================
// CURVA DE CALIBRAÇÃO (tensão em circuito aberto -> SOC %)
// Ilustrativa para Li-ion 1S. Substitua pelos valores medidos da sua
// bateria (descarga lenta e estável, sem carga aplicada).
// ======================================================================
struct PontoBateria {
    float tensao;
    float percentual;
};

class BatteryEstimator {
public:
    BatteryEstimator();

    // Chamar uma vez no setup()
    void begin();

    // Chamar periodicamente no loop(); dtMillis = tempo desde a última
    // chamada, em milissegundos (usado na integração de corrente).
    void update(uint32_t dtMillis);

    // --- Leituras brutas / filtradas ---
    float getRawVoltage() const     { return _vRaw; }
    float getFilteredVoltage() const { return _vFiltered; }
    float getCurrentA() const       { return _current; }

    // --- Tensão de circuito aberto estimada (compensa Rint) ---
    float getOpenCircuitVoltage() const { return _vOpenCircuit; }

    // --- Estimativas de SOC (State of Charge), em % ---
    float getPercentVoltageBased() const { return _socVoltage; }
    float getPercentCoulomb() const      { return _socCoulomb; }
    float getPercentCombined() const     { return _socCombined; }

    // Carga consumida acumulada (mAh) — só relevante com sensor de corrente
    float getConsumedMah() const { return _consumedMah; }

    // Reseta o Coulomb Counter para um SOC conhecido (ex: bateria cheia)
    void resetCoulombCounter(float socPercent);

private:
    float _vRaw;
    float _vFiltered;
    float _vOpenCircuit;
    float _current;
    float _consumedMah;

    float _socVoltage;
    float _socCoulomb;
    float _socCombined;

    bool _initialized;

    float readVoltageOnce();
    float readCurrentOnce();
    float voltageToPercent(float voltage) const;
    void updateCoulombCounting(uint32_t dtMillis);
    void updateCombinedEstimate();
};

#endif // BATTERY_ESTIMATOR_H