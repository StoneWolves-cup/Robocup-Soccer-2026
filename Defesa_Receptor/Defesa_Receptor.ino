#include <WiFi.h>
#include <esp_now.h>

#define DEBUG true

#define VEL_DEFESA          240
#define VEL_AJUSTE_FRENTE   170
#define VEL_AJUSTE_TRAS     170
#define VEL_FUGA_LIMITE     170

#define LIMITE_VELOCIDADE   245

#define DIST_MIN_TRAS 30
#define DIST_MAX_TRAS 38

#define DIST_MIN_ESQUERDA 22
#define DIST_MIN_DIREITA  22

#define KP_BNO 0.9
#define LIMITE_CORRECAO 25
#define ZONA_MORTA_BNO 6

typedef struct {
  float anguloRobo;
  int irDirecao;
  int irForca;
  bool viuBola;
  int ultraFrente;
  int ultraDireita;
  int ultraEsquerda;
  int ultraTras;
  int comando;
  int velocidade;
} DadosRobo;

DadosRobo dados;

void iniciarESPNow();
void iniciarMotores();
void parar();
void moverRobo(int frente, int lateral, int vel, int correcao);

int calcularCorrecaoBNO();
bool fugirLimiteUltrassonico();
void defenderBola();

void setup() {
  Serial.begin(115200);
  delay(1000);

  iniciarMotores();
  iniciarESPNow();

  Serial.println("ROBO DEFESA PRONTO - SEM START");
}

void loop() {
  if (fugirLimiteUltrassonico()) {
    return;
  }

  defenderBola();

  if (DEBUG) {
    Serial.print("IR: ");
    Serial.print(dados.irDirecao);

    Serial.print(" | Ultra Tras: ");
    Serial.print(dados.ultraTras);

    Serial.print(" | Ultra E: ");
    Serial.print(dados.ultraEsquerda);

    Serial.print(" | Ultra D: ");
    Serial.print(dados.ultraDireita);

    Serial.print(" | BNO: ");
    Serial.print(dados.anguloRobo);

    Serial.print(" | Correcao: ");
    Serial.println(calcularCorrecaoBNO());
  }

  delay(15);
}

float erroAngulo() {
  float erro = 0 - dados.anguloRobo;

  if (erro > 180) erro -= 360;
  if (erro < -180) erro += 360;

  return erro;
}

int calcularCorrecaoBNO() {
  float erro = erroAngulo();

  if (abs(erro) <= ZONA_MORTA_BNO) {
    return 0;
  }

  int c = erro * KP_BNO;
  c = constrain(c, -LIMITE_CORRECAO, LIMITE_CORRECAO);

  return c;
}

bool fugirLimiteUltrassonico() {
  int c = calcularCorrecaoBNO();

  if (dados.ultraDireita > 0 && dados.ultraDireita < DIST_MIN_DIREITA) {
    Serial.println("LIMITE DIREITO - INDO PARA ESQUERDA");
    moverRobo(0, -1, VEL_FUGA_LIMITE, c);
    delay(150);
    parar();
    return true;
  }

  if (dados.ultraEsquerda > 0 && dados.ultraEsquerda < DIST_MIN_ESQUERDA) {
    Serial.println("LIMITE ESQUERDO - INDO PARA DIREITA");
    moverRobo(0, 1, VEL_FUGA_LIMITE, c);
    delay(150);
    parar();
    return true;
  }

  return false;
}

void defenderBola() {
  int d = dados.irDirecao;
  int c = calcularCorrecaoBNO();

  if (dados.ultraTras > 0 && dados.ultraTras > DIST_MAX_TRAS) {
    moverRobo(-1, 0, VEL_AJUSTE_TRAS, c);
    return;
  }

  if (dados.ultraTras > 0 && dados.ultraTras < DIST_MIN_TRAS) {
    moverRobo(1, 0, VEL_AJUSTE_FRENTE, c);
    return;
  }

  if (dados.viuBola && d > 0) {
    if (d == 1 || d == 2 || d == 3 || d == 4) {
      moverRobo(0, -1, VEL_DEFESA, c);
      return;
    }

    if (d == 6 || d == 7 || d == 8 || d == 9) {
      moverRobo(0, 1, VEL_DEFESA, c);
      return;
    }

    if (d == 5) {
      moverRobo(0, 0, 0, c);
      return;
    }
  }

  moverRobo(0, 0, 0, c);
}