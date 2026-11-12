#include <Arduino.h>
#include <ConfigPinos.h>
#include <math.h>

// ============================================================
// Detecção direcional de bola por coroa de 16 sensores TSSP4038
// Plataforma: ESP32 DevKit V1
//
// Convenção angular:
//   0°   = frente
//   90°  = esquerda
//   180° = traseira
//   -90° = direita
//
// Os índices dos sensores aumentam no sentido horário, olhando
// o robô de cima; ângulos positivos continuam no sentido anti-horário.
// ============================================================

constexpr uint8_t NUM_SENSORES = 16;

// GPIO associado a cada posição da coroa.
// O índice de cada pino deve corresponder ao mesmo índice
// da tabela angulosSensores.
const uint8_t pinos[NUM_SENSORES] = {
  ConfigPinos::Head::TSSP::PINOS[0],
  ConfigPinos::Head::TSSP::PINOS[1],
  ConfigPinos::Head::TSSP::PINOS[2],
  ConfigPinos::Head::TSSP::PINOS[3],
  ConfigPinos::Head::TSSP::PINOS[4],
  ConfigPinos::Head::TSSP::PINOS[5],
  ConfigPinos::Head::TSSP::PINOS[6],
  ConfigPinos::Head::TSSP::PINOS[7],
  ConfigPinos::Head::TSSP::PINOS[8],
  ConfigPinos::Head::TSSP::PINOS[9],
  ConfigPinos::Head::TSSP::PINOS[10],
  ConfigPinos::Head::TSSP::PINOS[11],
  ConfigPinos::Head::TSSP::PINOS[12],
  ConfigPinos::Head::TSSP::PINOS[13],
  ConfigPinos::Head::TSSP::PINOS[14],
  ConfigPinos::Head::TSSP::PINOS[15]
};

// Ângulo nominal de cada sensor em relação à frente do robô.
// Estes valores pressupõem espaçamento uniforme de 22,5°.
const float angulosSensores[NUM_SENSORES] = {
  0.0f, 22.5f, 45.0f, 67.5f, 90.0f, 112.5f, 135.0f, 157.5f,
  180.0f, 202.5f, 225.0f, 247.5f, 270.0f, 292.5f, 315.0f, 337.5f
};

// Duração de cada janela usada para calcular a atividade dos sensores.
constexpr uint32_t JANELA_US = 5000;

// Intervalo entre atualizações enviadas pela porta serial.
constexpr uint32_t SERIAL_MS = 100;

// Coeficiente do filtro passa-baixas exponencial aplicado aos vetores.
// Valores próximos de 1 acompanham mudanças mais rapidamente;
// valores menores produzem maior suavização e mais atraso.
constexpr float ALPHA = 0.25f;

// Atividade abaixo deste valor é tratada como fundo/ruído.
// A atividade de cada sensor é expressa em porcentagem.
constexpr float LIMIAR_ATIVIDADE = 2.0f;

// Coerência mínima da resultante vetorial para aceitar uma direção.
// A confiança é uma medida de concentração direcional, não uma
// probabilidade estatística de a bola estar naquela direção.
constexpr float CONFIANCA_MIN = 0.15f;

// Contadores usados para estimar a porcentagem de tempo em que
// cada saída permaneceu em nível lógico LOW durante a janela atual.
uint32_t totalAmostras = 0;
uint32_t lowCount[NUM_SENSORES] = {};

// Atividade percentual medida e peso efetivo de cada sensor.
// O peso é a atividade após a subtração do limiar de fundo.
float atividade[NUM_SENSORES] = {};
float peso[NUM_SENSORES] = {};

// Componentes filtrados da resultante direcional.
float vetorXFiltrado = 0.0f;
float vetorYFiltrado = 0.0f;

// Soma filtrada dos pesos, usada para calcular a confiança
// de forma compatível com o vetor filtrado.
float somaPesosFiltrada = 0.0f;

// Indica se o filtro já recebeu sua primeira amostra.
bool filtroIniciado = false;

// Referências temporais para a janela de aquisição e a serialização.
uint32_t inicioJanela;
uint32_t ultimaSerial = 0;

// Resultados calculados para a janela mais recente.
float confianca = 0.0f;
bool bolaDetectada = false;

/**
 * Calcula a direção da bola usando uma média circular ponderada.
 *
 * Cada sensor contribui com um vetor unitário orientado segundo
 * seu ângulo. A magnitude de sua contribuição é proporcional à
 * atividade medida acima do limiar.
 *
 * A soma dos vetores é filtrada temporalmente para reduzir
 * oscilações entre janelas consecutivas.
 */
void calcularDirecao() {
  float x = 0.0f;
  float y = 0.0f;
  float somaPesos = 0.0f;

  // Converte as atividades em pesos e acumula as componentes
  // cartesianas da resultante direcional.
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    peso[i] = fmaxf(0.0f, atividade[i] - LIMIAR_ATIVIDADE);

    const float rad = angulosSensores[i] * DEG_TO_RAD;
    x += peso[i] * cosf(rad);
    y += peso[i] * sinf(rad);
    somaPesos += peso[i];
  }

  // Inicializa o filtro diretamente com a primeira resultante,
  // evitando uma partida gradual a partir do vetor nulo.
  if (!filtroIniciado) {
    vetorXFiltrado = x;
    vetorYFiltrado = y;
    somaPesosFiltrada = somaPesos;
    filtroIniciado = true;
  } else {
    // Filtro exponencial aplicado às componentes X e Y.
    vetorXFiltrado =
      ALPHA * x + (1.0f - ALPHA) * vetorXFiltrado;

    vetorYFiltrado =
      ALPHA * y + (1.0f - ALPHA) * vetorYFiltrado;

    // Filtra também a soma dos pesos para que o cálculo da
    // confiança use grandezas temporalmente consistentes.
    somaPesosFiltrada =
      ALPHA * somaPesos +
      (1.0f - ALPHA) * somaPesosFiltrada;
  }

  // Sem atividade válida, não há direção a reportar.
  if (somaPesos <= 0.0f || somaPesosFiltrada <= 0.0f) {
    bolaDetectada = false;
    confianca = 0.0f;
    return;
  }

  // A confiança representa a razão entre a magnitude da
  // resultante e a soma total dos pesos. Valores maiores indicam
  // que a atividade está mais concentrada em uma direção.
  const float moduloFiltrado = sqrtf(
    vetorXFiltrado * vetorXFiltrado +
    vetorYFiltrado * vetorYFiltrado
  );

  confianca = moduloFiltrado / somaPesosFiltrada;

  // Atividade distribuída de forma muito uniforme gera uma
  // direção ambígua; nesse caso, a detecção é rejeitada.
  if (confianca < CONFIANCA_MIN) {
    bolaDetectada = false;
    return;
  }

  bolaDetectada = true;
}

/**
 * Finaliza a janela de aquisição atual.
 *
 * Para cada sensor, calcula a porcentagem de amostras em LOW,
 * zera os contadores da janela e atualiza a estimativa direcional.
 */
void fecharJanela() {
  if (totalAmostras == 0) {
    return;
  }

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    atividade[i] =
      100.0f * lowCount[i] / totalAmostras;

    lowCount[i] = 0;
  }

  totalAmostras = 0;
  calcularDirecao();
}

/**
 * Envia as atividades dos sensores e o estado de deteccao
 * ao monitor serial.
 */
void imprimirDados() {
  Serial.print("LOW%: ");

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    Serial.print(atividade[i], 1);

    if (i < NUM_SENSORES - 1) {
      Serial.print(',');
    }
  }

  Serial.print(" | Bola: ");
  Serial.print(bolaDetectada ? "SIM" : "NAO");

  Serial.println();
}

void setup() {
  Serial.begin(115200);

  // Configura as entradas. Nos GPIOs 34 a 39, o ESP32 não
  // disponibiliza pull-up interno; o circuito deve fornecer
  // o nível elétrico adequado externamente, se necessário.
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    if (pinos[i] >= 34 && pinos[i] <= 39) {
      pinMode(pinos[i], INPUT);
    } else {
      pinMode(pinos[i], INPUT_PULLUP);
    }
  }

  inicioJanela = micros();

  Serial.println("Coroa TSSP4038 - 16 sensores");
  Serial.println("Verifique os GPIOs e os pull-ups externos.");
}

void loop() {
  // Amostra todos os sensores. Uma saída LOW incrementa o
  // contador correspondente à janela de aquisição.
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    if (digitalRead(pinos[i]) == LOW) {
      lowCount[i]++;
    }
  }

  totalAmostras++;

  // Fecha a janela quando sua duração atingir JANELA_US.
  // A subtração unsigned também funciona em caso de rollover
  // do contador de micros().
  const uint32_t agoraUs = micros();

  if ((uint32_t)(agoraUs - inicioJanela) >= JANELA_US) {
    fecharJanela();
    inicioJanela = agoraUs;
  }

  // Atualiza a saída serial sem usar delay(), mantendo a
  // aquisição dos sensores em execução contínua.
  const uint32_t agoraMs = millis();

  if ((uint32_t)(agoraMs - ultimaSerial) >= SERIAL_MS) {
    ultimaSerial = agoraMs;
    imprimirDados();
  }
}