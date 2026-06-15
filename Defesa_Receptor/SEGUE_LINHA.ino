#define SENSOR_FRENTE   35
#define SENSOR_DIREITA  33
#define SENSOR_TRAS     32
#define SENSOR_ESQUERDA 34

void iniciarLinha() {
  pinMode(SENSOR_FRENTE, INPUT);
  pinMode(SENSOR_DIREITA, INPUT);
  pinMode(SENSOR_TRAS, INPUT);
  pinMode(SENSOR_ESQUERDA, INPUT);

  Serial.println("Sensores de linha prontos!");
}

// VERDE = 1
// BRANCO = 0

bool linhaFrente() {
  return digitalRead(SENSOR_FRENTE) == LOW;
}

bool linhaDireita() {
  return digitalRead(SENSOR_DIREITA) == LOW;
}

bool linhaTras() {
  return digitalRead(SENSOR_TRAS) == LOW;
}

bool linhaEsquerda() {
  return digitalRead(SENSOR_ESQUERDA) == LOW;
}
