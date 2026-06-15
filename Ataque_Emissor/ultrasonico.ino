// ===============================
// ULTRASSONICOS - ROBO DEFESA
// ===============================

// Direita
#define TRIG_DIREITA  26
#define ECHO_DIREITA  27

// Esquerda
#define TRIG_ESQUERDA 23
#define ECHO_ESQUERDA 19

// Trás
#define TRIG_TRAS     32
#define ECHO_TRAS     33

void iniciarUltrassonicos() {
  pinMode(TRIG_DIREITA, OUTPUT);
  pinMode(ECHO_DIREITA, INPUT);

  pinMode(TRIG_ESQUERDA, OUTPUT);
  pinMode(ECHO_ESQUERDA, INPUT);

  pinMode(TRIG_TRAS, OUTPUT);
  pinMode(ECHO_TRAS, INPUT);

  Serial.println("Ultrassonicos defesa prontos!");
}

int medirDistancia(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duracao = pulseIn(echo, HIGH, 25000);

  if (duracao == 0) {
    return -1;
  }

  int distancia = duracao * 0.034 / 2;
  return distancia;
}

int lerUltraDireita() {
  delay(30);
  return medirDistancia(TRIG_DIREITA, ECHO_DIREITA);
}

int lerUltraEsquerda() {
  delay(30);
  return medirDistancia(TRIG_ESQUERDA, ECHO_ESQUERDA);
}

int lerUltraTras() {
  delay(30);
  return medirDistancia(TRIG_TRAS, ECHO_TRAS);
}