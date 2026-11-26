#include <Arduino.h>
#include <ConfigPinos.h>

namespace {
constexpr uint32_t INTERVALO_MS = 1000;
constexpr int PINO_RELE = ConfigPinos::Body::Kicker::IN_DIREITA;

uint32_t ultimaTrocaMs = 0;
bool releAtivo = false;
}  // namespace

void setup() {
  Serial.begin(115200);

  pinMode(PINO_RELE, OUTPUT);
  digitalWrite(PINO_RELE, HIGH);

  Serial.println("Teste do rele: acionando e desligando a cada 1 segundo.");
}

void loop() {
  
  const uint32_t agora = millis();

  if ((uint32_t)(agora - ultimaTrocaMs) >= INTERVALO_MS) {
    ultimaTrocaMs = agora;
    releAtivo = !releAtivo;

    digitalWrite(PINO_RELE, releAtivo ? LOW : HIGH);
    Serial.printf("Rele %s\n", releAtivo ? "acionado" : "desligado");
  }
}
