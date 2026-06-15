#define M1_IN1 27
#define M1_IN2 26
#define M1_EN  25

#define M2_IN1 2
#define M2_IN2 4
#define M2_EN  15

#define M3_IN1 12
#define M3_IN2 14
#define M3_EN  13

#define M4_IN1 16
#define M4_IN2 17
#define M4_EN  5

#define PWM_FREQ 2000
#define PWM_RES 8

int invM1 = 1;
int invM2 = 1;
int invM3 = 1;
int invM4 = 1;

void iniciarMotores() {
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);

  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);

  pinMode(M4_IN1, OUTPUT);
  pinMode(M4_IN2, OUTPUT);

  ledcAttach(M1_EN, PWM_FREQ, PWM_RES);
  ledcAttach(M2_EN, PWM_FREQ, PWM_RES);
  ledcAttach(M3_EN, PWM_FREQ, PWM_RES);
  ledcAttach(M4_EN, PWM_FREQ, PWM_RES);

  parar();

  Serial.println("Motores prontos!");
}

void motor(int in1, int in2, int en, int vel, int inv) {
  vel = constrain(vel, -255, 255);
  vel = vel * inv;

  if (vel > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(en, vel);
  } 
  else if (vel < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(en, abs(vel));
  } 
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(en, 0);
  }
}

void moverMotores(int m1, int m2, int m3, int m4) {
  m1 = constrain(m1, -LIMITE_VELOCIDADE, LIMITE_VELOCIDADE);
  m2 = constrain(m2, -LIMITE_VELOCIDADE, LIMITE_VELOCIDADE);
  m3 = constrain(m3, -LIMITE_VELOCIDADE, LIMITE_VELOCIDADE);
  m4 = constrain(m4, -LIMITE_VELOCIDADE, LIMITE_VELOCIDADE);

  motor(M1_IN1, M1_IN2, M1_EN, m1, invM1);
  motor(M2_IN1, M2_IN2, M2_EN, m2, invM2);
  motor(M3_IN1, M3_IN2, M3_EN, m3, invM3);
  motor(M4_IN1, M4_IN2, M4_EN, m4, invM4);
}

void parar() {
  moverMotores(0, 0, 0, 0);
}

// frente: 1 frente | -1 trás
// lateral: 1 direita | -1 esquerda
void moverRobo(int frente, int lateral, int vel, int correcao) {
  float x = lateral;
  float y = frente;

  float giro = correcao / 255.0;

  float m1 = y + x + giro;
  float m2 = y - x + giro;
  float m3 = y - x - giro;
  float m4 = y + x - giro;

  float maior = max(max(abs(m1), abs(m2)), max(abs(m3), abs(m4)));

  if (maior > 1.0) {
    m1 /= maior;
    m2 /= maior;
    m3 /= maior;
    m4 /= maior;
  }

  moverMotores(
    m1 * vel,
    m2 * vel,
    m3 * vel,
    m4 * vel
  );
}
