#include <Arduino.h>

void tarefaSensores(void *parameter) {
  while (true) {
    // Ler sensores
    Serial.println("Tarefa de sensores");

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void tarefaMotores(void *parameter) {
  while (true) {
    // Controlar motores
    Serial.println("Tarefa de motores");

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Serial.begin(115200);

  xTaskCreatePinnedToCore(
    tarefaSensores,  // função da tarefa
    "Sensores",      // nome
    4096,            // stack em bytes
    nullptr,         // parâmetro
    1,               // prioridade
    nullptr,         // handle opcional
    0                // núcleo
  );

  xTaskCreatePinnedToCore(
    tarefaMotores,
    "Motores",
    4096,
    nullptr,
    1,
    nullptr,
    1                // núcleo
  );
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}