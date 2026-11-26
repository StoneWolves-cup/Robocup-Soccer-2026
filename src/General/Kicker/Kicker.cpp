#include <Kicker.h>
#include <KickerLogica.h>
#include <ConfigPinos.h>
#include <IRProtocol.h>
#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_timer.h>

namespace {
using namespace ConfigPinos::Body::Kicker;

constexpr uint8_t NIVEL_KICKER_ATIVADO = LOW;
constexpr uint8_t NIVEL_KICKER_DESATIVADO = HIGH;

esp_timer_handle_t temporizador = nullptr;
bool temporizadorFalhou = false;
portMUX_TYPE trava = portMUX_INITIALIZER_UNLOCKED;
bool disparoPendente = false;
bool disparoAutomatico = false;
bool jaDisparou = false;
uint32_t fimDisparoMs = 0;
const char *ultimoBloqueio = nullptr;

void informarBloqueio(const char *motivo) {
  if (motivo == ultimoBloqueio) return;
  ultimoBloqueio = motivo;
  if (motivo) Serial.printf("Kicker bloqueado: %s\n", motivo);
}

void encerrarDisparo(void *) {
  portENTER_CRITICAL(&trava);
  digitalWrite(IN_ESQUERDA, NIVEL_KICKER_DESATIVADO);
  digitalWrite(IN_DIREITA, NIVEL_KICKER_DESATIVADO);
  fimDisparoMs = millis();
  jaDisparou = true;
  disparoPendente = false;
  disparoAutomatico = false;
  portEXIT_CRITICAL(&trava);
}

bool pinosValidos() {
  const int pinos[] = {IN_ESQUERDA, IN_DIREITA, FIM_CURSO};
  for (unsigned i = 0; i < 3; ++i) {
    if (!GPIO_IS_VALID_GPIO(pinos[i]) ||
        (i < 2 && !GPIO_IS_VALID_OUTPUT_GPIO(pinos[i]))) return false;
    for (unsigned j = 0; j < i; ++j) {
      if (pinos[i] == pinos[j]) return false;
    }
  }
  return true;
}
bool disparar(KickerLogica::Lado lado, bool automatico) {
  portENTER_CRITICAL(&trava);
  digitalWrite(IN_ESQUERDA, NIVEL_KICKER_DESATIVADO);
  digitalWrite(IN_DIREITA, NIVEL_KICKER_DESATIVADO);
  disparoPendente = true;
  disparoAutomatico = automatico;
  portEXIT_CRITICAL(&trava);
  const esp_err_t resultado = esp_timer_start_once(
      temporizador, KickerLogica::TEMPO_DISPARO_MS * 1000ULL);
  if (resultado != ESP_OK) {
    encerrarDisparo(nullptr);
    temporizadorFalhou = true;
    Serial.printf("ERRO: disparo do kicker cancelado: %s\n",
                  esp_err_to_name(resultado));
    return false;
  }
  portENTER_CRITICAL(&trava);
  const bool acionado = disparoPendente;
  if (acionado) {
    if (lado == KickerLogica::Lado::Esquerda ||
        lado == KickerLogica::Lado::Ambos) {
      digitalWrite(IN_ESQUERDA, NIVEL_KICKER_ATIVADO);
    }
    if (lado == KickerLogica::Lado::Direita ||
        lado == KickerLogica::Lado::Ambos) {
      digitalWrite(IN_DIREITA, NIVEL_KICKER_ATIVADO);
    }
  }
  portEXIT_CRITICAL(&trava);
  if (!acionado) {
    Serial.println("ERRO: temporizador expirou antes do acionamento do kicker.");
    return false;
  }
  const char *ladoTexto =
      lado == KickerLogica::Lado::Esquerda ? "esquerdo"
      : lado == KickerLogica::Lado::Direita ? "direito"
      : "duplo";
  Serial.printf("Kicker: disparo %s.\n", ladoTexto);
  return true;
}
}  // namespace

void Kicker::begin() {
  if (!pinosValidos()) {
    Serial.println("Kicker desabilitado: defina tres GPIOs validos e distintos.");
    return;
  }
  const esp_err_t resultadoEsquerda = gpio_set_level(
      static_cast<gpio_num_t>(IN_ESQUERDA), NIVEL_KICKER_DESATIVADO);
  const esp_err_t resultadoDireita = gpio_set_level(
      static_cast<gpio_num_t>(IN_DIREITA), NIVEL_KICKER_DESATIVADO);
  if (resultadoEsquerda != ESP_OK || resultadoDireita != ESP_OK) {
    Serial.printf("ERRO: nao foi possivel inicializar saidas do kicker: %s / %s\n",
                  esp_err_to_name(resultadoEsquerda),
                  esp_err_to_name(resultadoDireita));
    return;
  }
  pinMode(IN_ESQUERDA, OUTPUT);
  pinMode(IN_DIREITA, OUTPUT);
  pinMode(FIM_CURSO, INPUT_PULLDOWN);

  esp_timer_create_args_t argumentos{};
  argumentos.callback = encerrarDisparo;
  argumentos.name = "kicker";
  const esp_err_t resultado = esp_timer_create(&argumentos, &temporizador);
  if (resultado != ESP_OK) {
    temporizador = nullptr;
    Serial.printf("ERRO: nao foi possivel criar temporizador do kicker: %s\n",
                  esp_err_to_name(resultado));
  }
}

void Kicker::atualizar(ModoAutomatico modo, bool comunicacaoValida,
                       bool bateriaValida, float bateriaPercentual,
                       bool ultraValido, const float distancias[4],
                       uint8_t mascaraUltra) {
  if (!temporizador || temporizadorFalhou) return;
  const bool bateriaOk =
      KickerLogica::bateriaPermite(bateriaValida, bateriaPercentual);
  const bool modoAtaquePersonalizado =
      modo == ModoAutomatico::AtaqueUltrassomDireito;
  const uint8_t mascaraNecessaria = modoAtaquePersonalizado
      ? (1U << ULTRA_INDEX_RIGHT)
      : (1U << ULTRA_INDEX_FRONT) | (1U << ULTRA_INDEX_LEFT) |
            (1U << ULTRA_INDEX_RIGHT);
  const bool ultrasOk = ultraValido &&
      (mascaraUltra & mascaraNecessaria) == mascaraNecessaria &&
      KickerLogica::distanciaValida(distancias[ULTRA_INDEX_RIGHT]) &&
      (modoAtaquePersonalizado ||
       (KickerLogica::distanciaValida(distancias[ULTRA_INDEX_FRONT]) &&
        KickerLogica::distanciaValida(distancias[ULTRA_INDEX_LEFT])));
  const bool modoAtivo = modo != ModoAutomatico::Desativado;
  const bool precisaUltrassom = modo == ModoAutomatico::ComUltrassom ||
      modoAtaquePersonalizado;

  portENTER_CRITICAL(&trava);
  const bool pendente = disparoPendente;
  const bool disparou = jaDisparou;
  const uint32_t fimMs = fimDisparoMs;
  if (disparoAutomatico &&
      (!modoAtivo || !bateriaOk ||
       (precisaUltrassom && (!comunicacaoValida || !ultrasOk)))) {
    digitalWrite(IN_ESQUERDA, NIVEL_KICKER_DESATIVADO);
    digitalWrite(IN_DIREITA, NIVEL_KICKER_DESATIVADO);
  }
  portEXIT_CRITICAL(&trava);

  if (!modoAtivo) { informarBloqueio("modo automatico inativo"); return; }
  if (precisaUltrassom && !comunicacaoValida) {
    informarBloqueio("UART desconectada"); return;
  }
  if (!bateriaOk) { informarBloqueio("bateria invalida ou abaixo de 40%"); return; }
  if (precisaUltrassom && !ultrasOk) {
    informarBloqueio("ultrassonicos invalidos ou expirados"); return;
  }
  informarBloqueio(nullptr);
  if (pendente || !KickerLogica::cooldownConcluido(disparou, fimMs, millis()))
    return;
  if (!modoAtaquePersonalizado && digitalRead(FIM_CURSO) != HIGH) return;
  KickerLogica::Lado lado = KickerLogica::Lado::Ambos;
  if (modoAtaquePersonalizado) {
    const bool leituraDireitaValida = ultraValido &&
        (mascaraUltra & (1U << ULTRA_INDEX_RIGHT)) != 0;
    lado = KickerLogica::selecionarAtaque(
        distancias[ULTRA_INDEX_RIGHT], leituraDireitaValida);
  } else if (modo == ModoAutomatico::ComUltrassom) {
    lado = KickerLogica::selecionar(
        distancias[ULTRA_INDEX_FRONT], distancias[ULTRA_INDEX_LEFT],
        distancias[ULTRA_INDEX_RIGHT], true);
  }
  if (lado != KickerLogica::Lado::Nenhum) disparar(lado, true);
}

// Disparo manual pela IHM: ignora a estrategia e o fim de curso,
// mas respeita disparo em andamento e cooldown.
bool Kicker::dispararManual(bool direita) {
  if (!temporizador || temporizadorFalhou) return false;
  portENTER_CRITICAL(&trava);
  const bool pendente = disparoPendente;
  const bool disparou = jaDisparou;
  const uint32_t fimMs = fimDisparoMs;
  portEXIT_CRITICAL(&trava);
  if (pendente || !KickerLogica::cooldownConcluido(disparou, fimMs, millis()))
    return false;
  return disparar(direita ? KickerLogica::Lado::Direita
                          : KickerLogica::Lado::Esquerda, false);
}
