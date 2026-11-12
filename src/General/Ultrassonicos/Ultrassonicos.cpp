#include <Ultrassonicos.h>
#include <driver/gpio.h>

namespace {
constexpr uint32_t TIMEOUT_ECHO_US = 25000;

volatile uint32_t inicioPulsoUs[Ultrassonicos::QUANTIDADE] = {};
volatile uint32_t duracaoPulsoUs[Ultrassonicos::QUANTIDADE] = {};
volatile bool pulsoIniciado[Ultrassonicos::QUANTIDADE] = {};
volatile bool leituraConcluida[Ultrassonicos::QUANTIDADE] = {};

void IRAM_ATTR registrarBorda(uint8_t indice, int nivel) {
  const uint32_t agora = micros();
  if (nivel == HIGH) {
    inicioPulsoUs[indice] = agora;
    pulsoIniciado[indice] = true;
  } else if (pulsoIniciado[indice]) {
    duracaoPulsoUs[indice] = agora - inicioPulsoUs[indice];
    leituraConcluida[indice] = true;
    pulsoIniciado[indice] = false;
  }
}

void IRAM_ATTR eco11Mudou() {
  registrarBorda(0, gpio_get_level(static_cast<gpio_num_t>(
      ConfigPinos::Teste::Ultrassonicos::ECHO1)));
}
void IRAM_ATTR eco12Mudou() {
  registrarBorda(1, gpio_get_level(static_cast<gpio_num_t>(
      ConfigPinos::Teste::Ultrassonicos::ECHO2)));
}
void IRAM_ATTR eco13Mudou() {
  registrarBorda(2, gpio_get_level(static_cast<gpio_num_t>(
      ConfigPinos::Teste::Ultrassonicos::ECHO3)));
}
void IRAM_ATTR eco14Mudou() {
  registrarBorda(3, gpio_get_level(static_cast<gpio_num_t>(
      ConfigPinos::Teste::Ultrassonicos::ECHO4)));
}
}  // namespace

const uint8_t Ultrassonicos::PINOS_ECHO[Ultrassonicos::QUANTIDADE] = {
    ConfigPinos::Teste::Ultrassonicos::ECHO[0],
    ConfigPinos::Teste::Ultrassonicos::ECHO[1],
    ConfigPinos::Teste::Ultrassonicos::ECHO[2],
    ConfigPinos::Teste::Ultrassonicos::ECHO[3]};

void Ultrassonicos::begin() {
  pinMode(PINO_TRIG, OUTPUT);
  digitalWrite(PINO_TRIG, LOW);
  for (uint8_t pin : PINOS_ECHO) pinMode(pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(PINOS_ECHO[0]), eco11Mudou, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINOS_ECHO[1]), eco12Mudou, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINOS_ECHO[2]), eco13Mudou, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINOS_ECHO[3]), eco14Mudou, CHANGE);
}

void Ultrassonicos::ler(float distanciasCm[QUANTIDADE], bool leiturasValidas[QUANTIDADE]) {
  noInterrupts();
  for (uint8_t i = 0; i < QUANTIDADE; ++i) {
    inicioPulsoUs[i] = 0;
    duracaoPulsoUs[i] = 0;
    pulsoIniciado[i] = false;
    leituraConcluida[i] = false;
  }
  interrupts();

  digitalWrite(PINO_TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(PINO_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PINO_TRIG, LOW);

  const uint32_t inicioEspera = micros();
  while ((uint32_t)(micros() - inicioEspera) < TIMEOUT_ECHO_US) {
    bool todosConcluidos = true;
    for (uint8_t i = 0; i < QUANTIDADE; ++i) {
      if (!leituraConcluida[i]) {
        todosConcluidos = false;
        break;
      }
    }
    if (todosConcluidos) break;
    delay(1);  // Cede tempo para as demais tarefas do ESP32.
  }

  noInterrupts();
  for (uint8_t i = 0; i < QUANTIDADE; ++i) {
    distanciasCm[i] = duracaoPulsoUs[i] * 0.0343f / 2.0f;
    leiturasValidas[i] = leituraConcluida[i]
        && distanciasCm[i] >= 2.0f
        && distanciasCm[i] <= 430.0f;
  }
  interrupts();
}
