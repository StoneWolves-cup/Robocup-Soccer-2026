#include <Arduino.h>

// ============================================================
// PINOS
// ============================================================

#define FIM_CURSO_1 15
#define FIM_CURSO_2 2

#define RELE_1 4
#define RELE_2 5


// ============================================================
// TEMPOS
// ============================================================

// Tempo que o relé permanece acionado
#define TEMPO_DISPARO 100

// Tempo mínimo entre disparos
#define TEMPO_BLOQUEIO 400


// ============================================================
// CONTROLE DO SOLENOIDE 1
// ============================================================

bool solenoide1Ativo = false;

unsigned long inicioDisparo1 = 0;
unsigned long ultimoDisparo1 = 0;

unsigned long contadorSolenoide1 = 0;


// ============================================================
// CONTROLE DO SOLENOIDE 2
// ============================================================

bool solenoide2Ativo = false;

unsigned long inicioDisparo2 = 0;
unsigned long ultimoDisparo2 = 0;

unsigned long contadorSolenoide2 = 0;


// ============================================================
// SETUP
// ============================================================

void setup() {

    Serial.begin(115200);

    // Fins de curso
    pinMode(FIM_CURSO_1, INPUT_PULLDOWN);
    pinMode(FIM_CURSO_2, INPUT_PULLDOWN);

    // Relés
    pinMode(RELE_1, OUTPUT);
    pinMode(RELE_2, OUTPUT);

    // HIGH = relé desligado
    digitalWrite(RELE_1, HIGH);
    digitalWrite(RELE_2, HIGH);

    Serial.println("=================================");
    Serial.println("Sistema iniciado!");
    Serial.println("Rele 1 -> D4");
    Serial.println("Rele 2 -> D5");
    Serial.println("Disparo: 100 ms");
    Serial.println("Bloqueio: 400 ms");
    Serial.println("=================================");

    Serial.println("Contador Solenoide 1: 0");
    Serial.println("Contador Solenoide 2: 0");
}


// ============================================================
// LOOP
// ============================================================

void loop() {

    unsigned long agora = millis();


    // ========================================================
    // LEITURA DOS FINS DE CURSO
    // ========================================================

    bool chave1 = digitalRead(FIM_CURSO_1);
    bool chave2 = digitalRead(FIM_CURSO_2);


    // ========================================================
    // FIM DE CURSO 1 → RELE 1 / D4
    // ========================================================

    // Enquanto o fim de curso estiver pressionado,
    // o solenoide dispara novamente quando estiver liberado.

    if (chave1 == HIGH) {

        if (!solenoide1Ativo &&
            agora - ultimoDisparo1 >= TEMPO_BLOQUEIO) {

            // LOW = relé acionado
            digitalWrite(RELE_1, LOW);

            solenoide1Ativo = true;

            inicioDisparo1 = agora;
            ultimoDisparo1 = agora;

            // Incrementa o contador
            contadorSolenoide1++;

            Serial.println("SOLENOIDE 1 -> DISPAROU");

            Serial.print("Contador Solenoide 1: ");
            Serial.println(contadorSolenoide1);
        }
    }


    // ========================================================
    // FIM DE CURSO 2 → RELE 2 / D5
    // ========================================================

    // Enquanto o fim de curso estiver pressionado,
    // o solenoide dispara novamente quando estiver liberado.

    if (chave2 == HIGH) {

        if (!solenoide2Ativo &&
            agora - ultimoDisparo2 >= TEMPO_BLOQUEIO) {

            // LOW = relé acionado
            digitalWrite(RELE_2, LOW);

            solenoide2Ativo = true;

            inicioDisparo2 = agora;
            ultimoDisparo2 = agora;

            // Incrementa o contador
            contadorSolenoide2++;

            Serial.println("SOLENOIDE 2 -> DISPAROU");

            Serial.print("Contador Solenoide 2: ");
            Serial.println(contadorSolenoide2);
        }
    }


    // ========================================================
    // DESLIGA SOLENOIDE 1 APÓS 100 ms
    // ========================================================

    if (solenoide1Ativo &&
        agora - inicioDisparo1 >= TEMPO_DISPARO) {

        // HIGH = relé desligado
        digitalWrite(RELE_1, HIGH);

        solenoide1Ativo = false;

        Serial.println("SOLENOIDE 1 -> DESLIGADO");
    }


    // ========================================================
    // DESLIGA SOLENOIDE 2 APÓS 100 ms
    // ========================================================

    if (solenoide2Ativo &&
        agora - inicioDisparo2 >= TEMPO_DISPARO) {

        // HIGH = relé desligado
        digitalWrite(RELE_2, HIGH);

        solenoide2Ativo = false;

        Serial.println("SOLENOIDE 2 -> DESLIGADO");
    }


    // ========================================================
    // INTERVALO DE LEITURA
    // ========================================================

    delay(50);
}