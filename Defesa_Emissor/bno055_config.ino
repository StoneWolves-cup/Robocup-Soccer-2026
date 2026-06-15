class BNO055 {
public:
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
  sensors_event_t event;

  int16_t offset_x = 0;

  void begin() {
    Serial.println("Iniciando BNO055...");

    if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
      Serial.println("ERRO: BNO055 nao encontrado");
      while (1);
    }

    delay(100);
    bno.setExtCrystalUse(true);

    resetZero();

    Serial.println("BNO055 pronto");
  }

  void resetZero() {
    bno.getEvent(&event);
    offset_x = event.orientation.x;

    Serial.print("Zero BNO: ");
    Serial.println(offset_x);
  }

  int16_t getYawAngle() {
    bno.getEvent(&event);
    return converterAngulo(event.orientation.x, offset_x);
  }

  int16_t converterAngulo(int16_t valor, int16_t offset) {
    int16_t angulo = valor - offset;

    if (angulo > 180) angulo -= 360;
    if (angulo < -180) angulo += 360;

    return angulo;
  }
};

BNO055 gyro;

void iniciarBNO() {
  gyro.begin();
}

float lerAnguloBNO() {
  return gyro.getYawAngle();
}