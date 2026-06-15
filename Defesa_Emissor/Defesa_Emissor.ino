#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define SDA_ESP32 21
#define SCL_ESP32 22

uint8_t enderecoReceptor[] = {0xB0, 0xCB, 0xD8, 0xC9, 0x84, 0x2C};

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
void enviarDados();

void iniciarBNO();
float lerAnguloBNO();

void iniciarIRSeeker();
void lerIRSeeker(int &direcao, int &forca, bool &viuBola);

void iniciarUltrassonicos();
int lerUltraDireita();
int lerUltraEsquerda();
int lerUltraTras();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_ESP32, SCL_ESP32);

  iniciarBNO();
  iniciarIRSeeker();
  iniciarUltrassonicos();
  iniciarESPNow();

  Serial.println("ESP EMISSOR DEFESA PRONTO");
}

void loop() {
  dados.anguloRobo = lerAnguloBNO();

  lerIRSeeker(
    dados.irDirecao,
    dados.irForca,
    dados.viuBola
  );

  dados.ultraFrente = 0;
  dados.ultraDireita = lerUltraDireita();
  dados.ultraEsquerda = lerUltraEsquerda();
  dados.ultraTras = lerUltraTras();

  dados.comando = 0;
  dados.velocidade = 0;

  enviarDados();

  Serial.print("BNO: ");
  Serial.print(dados.anguloRobo);

  Serial.print(" | IR Dir: ");
  Serial.print(dados.irDirecao);

  Serial.print(" | Forca: ");
  Serial.print(dados.irForca);

  Serial.print(" | Bola: ");
  Serial.print(dados.viuBola ? "SIM" : "NAO");

  Serial.print(" | Ultra D: ");
  Serial.print(dados.ultraDireita);

  Serial.print(" | Ultra E: ");
  Serial.print(dados.ultraEsquerda);

  Serial.print(" | Ultra T: ");
  Serial.println(dados.ultraTras);

  delay(50);
}