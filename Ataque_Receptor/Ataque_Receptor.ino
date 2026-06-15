#include <WiFi.h>
#include <esp_now.h>

#define DEBUG true

#define START 18

extern bool recebeuPrimeiroPacote;
extern unsigned long ultimoPacoteRecebido;

// ==========================
// VELOCIDADES
// ==========================
#define VEL_PADRAO      240
#define VEL_CHUTE       255
#define VEL_FUGA_LINHA  185

#define LIMITE_VELOCIDADE 255

// ==========================
// BNO
// ==========================
#define KP_BNO 2.3
#define LIMITE_CORRECAO 60
#define ZONA_MORTA_BNO 1

// ==========================
// FORÇA PARA CONSIDERAR BOLA PERTO
// ==========================
#define FORCA_PERTO_D2 50
#define FORCA_PERTO_D3 45
#define FORCA_PERTO_D4 110
#define FORCA_PERTO_D5 70
#define FORCA_PERTO_D6 110
#define FORCA_PERTO_D7 45
#define FORCA_PERTO_D8 50

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
void iniciarLinha();

void parar();
void moverRobo(int frente, int lateral, int vel, int correcao);

bool linhaFrente();
bool linhaDireita();
bool linhaTras();
bool linhaEsquerda();

int calcularCorrecaoBNO();
bool fugirLinha();
void atacarBola();

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(START, INPUT_PULLDOWN);

  iniciarMotores();
  iniciarLinha();
  iniciarESPNow();

  Serial.println("ROBO PRONTO - START GPIO 18");
}

void loop() {
  if (digitalRead(START) == LOW) {
    parar();
    return;
  }

  if (!recebeuPrimeiroPacote) {
    parar();
    return;
  }

  if (fugirLinha()) {
    return;
  }

  atacarBola();

  if (DEBUG) {
    int c = calcularCorrecaoBNO();

    Serial.print("START: ");
    Serial.print(digitalRead(START) == HIGH ? "HIGH" : "LOW");

    Serial.print(" | Bola: ");
    Serial.print(dados.viuBola ? "SIM" : "NAO");

    Serial.print(" | Dir: ");
    Serial.print(dados.irDirecao);

    Serial.print(" | Forca: ");
    Serial.print(dados.irForca);

    Serial.print(" | BNO: ");
    Serial.print(dados.anguloRobo);

    Serial.print(" | Correcao: ");
    Serial.println(c);
  }

  delay(30);
}

// ==========================
// BNO
// ==========================
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

// ==========================
// ATAQUE
// ==========================
void atacarBola() {
  if (digitalRead(START) == LOW) {
    parar();
    return;
  }

  int d = dados.irDirecao;
  int f = dados.irForca;
  int c = calcularCorrecaoBNO();

  bool pertoD2 = f >= FORCA_PERTO_D2;
  bool pertoD3 = f >= FORCA_PERTO_D3;
  bool pertoD4 = f >= FORCA_PERTO_D4;
  bool pertoD5 = f >= FORCA_PERTO_D5;
  bool pertoD6 = f >= FORCA_PERTO_D6;
  bool pertoD7 = f >= FORCA_PERTO_D7;
  bool pertoD8 = f >= FORCA_PERTO_D8;

  if (!dados.viuBola) {
    moverRobo(-1, 0, VEL_PADRAO, c);
  }

  // Direção 2
  else if (d == 2 && !pertoD2) {
    moverRobo(-1, -1, VEL_PADRAO, c);  // diagonal trás esquerda
  }
  else if (d == 2 && pertoD2) {
    moverRobo(-1, 0, VEL_PADRAO, c);   // trás
  }

  // Direção 3
  else if (d == 3 && !pertoD3) {
    moverRobo(0, -1, VEL_PADRAO, c);   // esquerda
  }
  else if (d == 3 && pertoD3) {
    moverRobo(-1, -1, VEL_PADRAO, c);  // diagonal trás esquerda
  }

  // Direção 4
  else if (d == 4 && !pertoD4) {
    moverRobo(1, -1, VEL_PADRAO, c);   // diagonal frente esquerda
  }
  else if (d == 4 && pertoD4) {
    moverRobo(1, 0, VEL_PADRAO, c);    // frente
  }

  // Direção 5
  else if (d == 5 && !pertoD5) {
    moverRobo(1, 0, VEL_PADRAO, c);    // frente
  }
  else if (d == 5 && pertoD5) {
    moverRobo(1, 0, VEL_CHUTE, c);     // chute
  }

  // Direção 6
  else if (d == 6 && !pertoD6) {
    moverRobo(1, 1, VEL_PADRAO, c);    // diagonal frente direita
  }
  else if (d == 6 && pertoD6) {
    moverRobo(1, 0, VEL_PADRAO, c);    // frente
  }

  // Direção 7
  else if (d == 7 && !pertoD7) {
    moverRobo(0, 1, VEL_PADRAO, c);    // direita
  }
  else if (d == 7 && pertoD7) {
    moverRobo(-1, 1, VEL_PADRAO, c);   // diagonal trás direita
  }

  // Direção 8
  else if (d == 8 && !pertoD8) {
    moverRobo(-1, 1, VEL_PADRAO, c);   // diagonal trás direita
  }
  else if (d == 8 && pertoD8) {
    moverRobo(-1, 0, VEL_PADRAO, c);   // trás
  }

  else {
    moverRobo(-1, 0, VEL_PADRAO, c);
  }
}

// ==========================
// LINHA
// ==========================
bool fugirLinha() {
  if (digitalRead(START) == LOW) {
    parar();
    return true;
  }

  int c = calcularCorrecaoBNO();

  if (linhaFrente()) {
    Serial.println("LINHA FRENTE");
    moverRobo(-1, 0, VEL_FUGA_LINHA, c);
    delay(400);
    parar();
    return true;
  }

  if (linhaTras()) {
    Serial.println("LINHA TRAS");
    moverRobo(1, 0, VEL_FUGA_LINHA, c);
    delay(400);
    parar();
    return true;
  }

  if (linhaDireita()) {
    Serial.println("LINHA DIREITA");
    moverRobo(0, -1, VEL_FUGA_LINHA, c);
    delay(400);
    parar();
    return true;
  }

  if (linhaEsquerda()) {
    Serial.println("LINHA ESQUERDA");
    moverRobo(0, 1, VEL_FUGA_LINHA, c);
    delay(400);
    parar();
    return true;
  }

  return false;
}