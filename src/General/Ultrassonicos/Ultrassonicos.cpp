#include <Ultrassonicos.h>
#include <driver/gpio.h>

namespace {
constexpr uint32_t TIMEOUT_ECHO_US = 5000;

volatile uint32_t inicioPulsoUs[Ultrassonicos::QUANTIDADE] = {};
volatile uint32_t duracaoPulsoUs[Ultrassonicos::QUANTIDADE] = {};
volatile bool pulsoIniciado[Ultrassonicos::QUANTIDADE] = {};
volatile bool leituraConcluida[Ultrassonicos::QUANTIDADE] = {};
bool leituraEmAndamento = false;
uint32_t inicioLeituraUs = 0;

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

void IRAM_ATTR eco1Mudou() {
  registrarBorda(0, gpio_get_level(static_cast<gpio_num_t>(
      ConfigPinos::Head::Ultrassonicos::ECHO[0])));
}

void IRAM_ATTR eco2Mudou() {
  registrarBorda(1, gpio_get_level(static_cast<gpio_num_t>(
      ConfigPinos::Head::Ultrassonicos::ECHO[1])));
}

void IRAM_ATTR eco3Mudou() {
  registrarBorda(2, gpio_get_level(static_cast<gpio_num_t>(
      ConfigPinos::Head::Ultrassonicos::ECHO[2])));
}

void IRAM_ATTR eco4Mudou() {
  registrarBorda(3, gpio_get_level(static_cast<gpio_num_t>(
      ConfigPinos::Head::Ultrassonicos::ECHO[3])));
}
}  // namespace

const uint8_t Ultrassonicos::PINOS_ECHO[Ultrassonicos::QUANTIDADE] = {
    ConfigPinos::Head::Ultrassonicos::ECHO[0],
    ConfigPinos::Head::Ultrassonicos::ECHO[1],
    ConfigPinos::Head::Ultrassonicos::ECHO[2],
    ConfigPinos::Head::Ultrassonicos::ECHO[3]};

void Ultrassonicos::begin() {
  pinMode(PINO_TRIG, OUTPUT);
  digitalWrite(PINO_TRIG, LOW);
  for (uint8_t pin : PINOS_ECHO) pinMode(pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(PINOS_ECHO[0]), eco1Mudou, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINOS_ECHO[1]), eco2Mudou, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINOS_ECHO[2]), eco3Mudou, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINOS_ECHO[3]), eco4Mudou, CHANGE);
}

void Ultrassonicos::iniciarLeitura() {
  if (leituraEmAndamento) return;

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
  inicioLeituraUs = micros();
  leituraEmAndamento = true;
}

bool Ultrassonicos::atualizar(float distanciasCm[QUANTIDADE],
                              bool leiturasValidas[QUANTIDADE]) {
  if (!leituraEmAndamento) return false;

  bool todosConcluidos = true;
  noInterrupts();
  for (uint8_t i = 0; i < QUANTIDADE; ++i) {
    if (!leituraConcluida[i]) {
      todosConcluidos = false;
      break;
    }
  }
  interrupts();

  if (!todosConcluidos &&
      (uint32_t)(micros() - inicioLeituraUs) < TIMEOUT_ECHO_US) {
    return false;
  }

  noInterrupts();
  for (uint8_t i = 0; i < QUANTIDADE; ++i) {
    distanciasCm[i] = duracaoPulsoUs[i] * 0.0343f / 2.0f;
    leiturasValidas[i] = leituraConcluida[i] &&
        distanciasCm[i] >= 2.0f && distanciasCm[i] <= 430.0f;
    leituraConcluida[i] = false;
    pulsoIniciado[i] = false;
  }
  interrupts();
  leituraEmAndamento = false;
  return true;
}

// Compatibilidade para chamadores antigos; o Head usa a API nao bloqueante.
void Ultrassonicos::ler(float distanciasCm[QUANTIDADE],
                        bool leiturasValidas[QUANTIDADE]) {
  iniciarLeitura();
  while (!atualizar(distanciasCm, leiturasValidas)) {
    delay(20);
  }
}
