#define IR_SEEKER_ADDR 0x08

#define IR_REG_DIRECAO_AC 0x49
#define IR_REG_FORCA_AC_1 0x4A
#define IR_REG_FORCA_AC_2 0x4B
#define IR_REG_FORCA_AC_3 0x4C
#define IR_REG_FORCA_AC_4 0x4D
#define IR_REG_FORCA_AC_5 0x4E

#define FORCA_MINIMA_BOLA 10

void iniciarIRSeeker() {
  Serial.println("Iniciando HiTechnic IR Seeker...");

  Wire.beginTransmission(IR_SEEKER_ADDR);
  byte erro = Wire.endTransmission();

  if (erro == 0) {
    Serial.println("HiTechnic encontrado");
  } else {
    Serial.println("ERRO: HiTechnic nao encontrado");
  }
}

byte lerRegistroIR(byte reg) {
  Wire.beginTransmission(IR_SEEKER_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return 0;
  }

  Wire.requestFrom(IR_SEEKER_ADDR, 1);

  if (Wire.available()) {
    return Wire.read();
  }

  return 0;
}

void lerIRSeeker(int &direcao, int &forca, bool &viuBola) {
  int dir = lerRegistroIR(IR_REG_DIRECAO_AC);

  int s1 = lerRegistroIR(IR_REG_FORCA_AC_1);
  int s2 = lerRegistroIR(IR_REG_FORCA_AC_2);
  int s3 = lerRegistroIR(IR_REG_FORCA_AC_3);
  int s4 = lerRegistroIR(IR_REG_FORCA_AC_4);
  int s5 = lerRegistroIR(IR_REG_FORCA_AC_5);

  int maiorForca = max(s1, max(s2, max(s3, max(s4, s5))));

  direcao = dir;
  forca = maiorForca;

  if (direcao >= 1 && direcao <= 9 && forca >= FORCA_MINIMA_BOLA) {
    viuBola = true;
  } else {
    viuBola = false;
    direcao = 0;
    forca = 0;
  }
}