#include <Arduino.h>
#include <Kicker.h>
#include <KickerLogica.h>
#include <ConfigPinos.h>
#include <IRProtocol.h>
#include <esp_timer.h>
#include <assert.h>
#include <math.h>

namespace {
uint32_t agora = 0;
int valores[49] = {};
int modos[49] = {};
void (*callback)(void *) = nullptr;
uint64_t duracaoUs = 0;
int erroTimer = 0;
}

SerialStub Serial;
void pinMode(int pin, int mode) { modos[pin] = mode; }
void digitalWrite(int pin, int value) { valores[pin] = value; }
int digitalRead(int pin) { return valores[pin]; }
uint32_t millis() { return agora; }
esp_err_t esp_timer_create(const esp_timer_create_args_t *args,
                           esp_timer_handle_t *handle) {
  callback = args->callback;
  *handle = &duracaoUs;
  return ESP_OK;
}
esp_err_t esp_timer_start_once(esp_timer_handle_t, uint64_t duracao) {
  duracaoUs = duracao;
  return erroTimer;
}
const char *esp_err_to_name(esp_err_t) { return "erro"; }

int main() {
  using namespace KickerLogica;
  using namespace ConfigPinos::Body::Kicker;

  assert(selecionar(80.01f, 64.99f, 65.0f, true) == Lado::Esquerda);
  assert(selecionar(81.0f, 65.0f, 64.99f, true) == Lado::Direita);
  assert(selecionar(81.0f, 65.0f, 65.0f, true) == Lado::Nenhum);
  assert(selecionar(81.0f, 64.0f, 64.0f, true) == Lado::Nenhum);
  assert(selecionar(80.0f, 100.0f, 50.0f, true) == Lado::Direita);
  assert(selecionar(79.0f, 50.0f, 100.0f, true) == Lado::Esquerda);
  assert(selecionar(80.0f, 50.0f, 50.0f, true) == Lado::Nenhum);
  assert(selecionar(NAN, 50.0f, 100.0f, true) == Lado::Nenhum);
  assert(selecionar(80.0f, 0.0f, 100.0f, true) == Lado::Nenhum);
  assert(selecionar(80.0f, 50.0f, 100.0f, false) == Lado::Nenhum);
  assert(!bateriaPermite(false, 100.0f));
  assert(!bateriaPermite(true, 39.99f));
  assert(!bateriaPermite(true, NAN));
  assert(bateriaPermite(true, 40.0f));
  assert(cooldownConcluido(false, 0, 0));
  assert(!cooldownConcluido(true, UINT32_MAX - 100, 498));
  assert(cooldownConcluido(true, UINT32_MAX - 100, 499));

  Kicker::begin();
  assert(modos[IN_ESQUERDA] == OUTPUT && modos[IN_DIREITA] == OUTPUT);
  assert(modos[FIM_CURSO_1] == INPUT_PULLDOWN);
  float distancias[4] = {70.0f, 81.0f, 50.0f, 100.0f};
  const uint8_t mascara = (1U << ULTRA_INDEX_FRONT) |
                          (1U << ULTRA_INDEX_LEFT) |
                          (1U << ULTRA_INDEX_RIGHT);
  Kicker::atualizar(true, true, true, 50.0f, true, distancias, mascara);
  assert(valores[IN_ESQUERDA] == LOW && valores[IN_DIREITA] == LOW);
  valores[FIM_CURSO_2] = HIGH;
  Kicker::atualizar(true, true, true, 39.9f, true, distancias, mascara);
  assert(valores[IN_ESQUERDA] == LOW);
  Kicker::atualizar(true, true, true, 40.0f, true, distancias, mascara);
  assert(valores[IN_ESQUERDA] == HIGH && valores[IN_DIREITA] == LOW);
  assert(duracaoUs == 150000);
  agora = 150;
  callback(nullptr);
  assert(valores[IN_ESQUERDA] == LOW && valores[IN_DIREITA] == LOW);
  agora = 749;
  Kicker::atualizar(true, true, true, 50.0f, true, distancias, mascara);
  assert(valores[IN_ESQUERDA] == LOW);
  agora = 750;
  Kicker::atualizar(true, true, true, 50.0f, true, distancias, mascara);
  assert(valores[IN_ESQUERDA] == HIGH);
  Kicker::atualizar(true, true, true, 39.0f, true, distancias, mascara);
  assert(valores[IN_ESQUERDA] == LOW);
  agora = 900;
  callback(nullptr);
  agora = 1500;
  distancias[ULTRA_INDEX_FRONT] = 80.0f;
  Kicker::atualizar(true, true, true, 50.0f, true, distancias, mascara);
  assert(valores[IN_DIREITA] == LOW && valores[IN_ESQUERDA] == HIGH);
  Kicker::atualizar(false, true, true, 50.0f, true, distancias, mascara);
  assert(valores[IN_ESQUERDA] == LOW);
  agora = 1650;
  callback(nullptr);
  agora = 2250;
  Kicker::atualizar(true, true, true, 50.0f, true, distancias, mascara & ~1U);
  assert(valores[IN_ESQUERDA] == LOW);
  Kicker::atualizar(true, false, true, 50.0f, true, distancias, mascara);
  assert(valores[IN_ESQUERDA] == LOW);
  Kicker::atualizar(true, true, false, 50.0f, true, distancias, mascara);
  assert(valores[IN_ESQUERDA] == LOW);
  Kicker::atualizar(true, true, true, 50.0f, false, distancias, mascara);
  assert(valores[IN_ESQUERDA] == LOW);
  distancias[ULTRA_INDEX_LEFT] = 100.0f;
  Kicker::atualizar(true, true, true, 50.0f, true, distancias, mascara);
  assert(valores[IN_DIREITA] == HIGH && valores[IN_ESQUERDA] == LOW);
  agora = 2400;
  callback(nullptr);
  agora = 3000;
  erroTimer = 1;
  Kicker::atualizar(true, true, true, 50.0f, true, distancias, mascara);
  assert(valores[IN_ESQUERDA] == LOW && valores[IN_DIREITA] == LOW);
  erroTimer = 0;
  Kicker::atualizar(true, true, true, 50.0f, true, distancias, mascara);
  assert(valores[IN_ESQUERDA] == LOW && valores[IN_DIREITA] == LOW);
}
