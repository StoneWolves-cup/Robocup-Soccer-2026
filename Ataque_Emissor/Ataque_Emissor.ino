#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define SDA_ESP32 21
#define SCL_ESP32 22

uint8_t enderecoReceptor[] = {  0xA4, 0xF0, 0x0F, 0x77, 0x10, 0x84};

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

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_ESP32, SCL_ESP32);

  iniciarBNO();
  iniciarIRSeeker();
  iniciarESPNow();

  Serial.println("ESP CEREBRO PRONTO");
}

void loop() {
  dados.anguloRobo = lerAnguloBNO();

  lerIRSeeker(
    dados.irDirecao,
    dados.irForca,
    dados.viuBola
  );

  dados.ultraFrente = 0;
  dados.ultraDireita = 0;
  dados.ultraEsquerda = 0;
  dados.ultraTras = 0;

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
  Serial.println(dados.viuBola ? "SIM" : "NAO");

  delay(50);
}