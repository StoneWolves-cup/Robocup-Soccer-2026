#include <Arduino.h>
#include <BNO055.h>

namespace {
constexpr int UART_RX_PIN = 18;
constexpr int UART_TX_PIN = 17;
constexpr uint32_t INTERVALO_ENVIO_MS = 10;

BNO055 sensorBNO;
uint32_t ultimoEnvioMs = 0;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  Serial.println("=== Emissor de angulo BNO055 ===");
  if (!sensorBNO.begin()) {
    Serial.println("ERRO: BNO055 nao encontrado. Verifique a conexao I2C.");
    while (true) delay(1000);
  }

  sensorBNO.calibrateNorth();
  Serial.println("Enviando somente o angulo (graus), um valor por linha.");
  Serial.println("Envie 'c' neste monitor para redefinir o rumo zero.");
}

void loop() {
  if (Serial.available()) {
    const char comando = Serial.read();
    if (comando == 'c' || comando == 'C') {
      sensorBNO.update();
      sensorBNO.calibrateNorth();
      Serial.println("Referencia de rumo recalibrada.");
    }
  }

  const uint32_t agoraMs = millis();
  if ((uint32_t)(agoraMs - ultimoEnvioMs) < INTERVALO_ENVIO_MS) return;
  ultimoEnvioMs = agoraMs;

  sensorBNO.update();
  const float heading = sensorBNO.getHeading();
  Serial1.printf("%.2f\n", heading);
  Serial.printf("Rumo: %.2f graus\n", heading);
}
