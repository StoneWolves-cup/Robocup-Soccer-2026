// Firmware do Head compartilhado pelos dois robos: concentra aquisicao e
// publicacao dos sensores, enquanto o Body escolhe e executa a estrategia.
#include <Arduino.h>
#include <math.h>
#include <IRProtocol.h>
#include <BNO055.h>
#include <TSSP.h>
#include <Ultrassonicos.h>
#include <ConfigPinos.h>

namespace {
constexpr uint8_t NUM_SENSORES = IR_SENSOR_COUNT;
// Ritmos independentes evitam que leitura do sonar limite o fluxo IR mais rapido.
constexpr uint32_t SERIAL_MS = 100;
constexpr uint32_t INTERVALO_ENVIO_MS = 20;
constexpr uint32_t INTERVALO_BNO_MS = 10;
constexpr uint32_t INTERVALO_TENTATIVA_BNO_MS = 50;
constexpr uint32_t INTERVALO_ULTRA_MS = 100;
constexpr uint32_t USB_SERIAL_BAUD = 921600;
// ALPHA controla suavizacao da direcao; limiar elimina atividade de fundo antes
// da soma vetorial e confianca rejeita deteccoes difusas no anel.
constexpr float ALPHA = 0.25f;
constexpr float LIMIAR_ATIVIDADE = 2.0f;
constexpr float CONFIANCA_MIN = 0.15f;
constexpr int UART_RX_PIN = ConfigPinos::Geral::UART1::RX;
constexpr int UART_TX_PIN = ConfigPinos::Geral::UART1::TX;
constexpr uint8_t BNO_SDA_PIN = ConfigPinos::Head::BNO055::SDA;
constexpr uint8_t BNO_SCL_PIN = ConfigPinos::Head::BNO055::SCL;

const float angulosSensores[NUM_SENSORES] = {
  0.0f, 22.5f, 45.0f, 67.5f, 90.0f, 112.5f, 135.0f, 157.5f,
  180.0f, 202.5f, 225.0f, 247.5f, 270.0f, 292.5f, 315.0f, 337.5f
};

float atividade[NUM_SENSORES] = {};
float peso[NUM_SENSORES] = {};
float vetorXFiltrado = 0.0f;
float vetorYFiltrado = 0.0f;
float somaPesosFiltrada = 0.0f;
bool filtroIniciado = false;
uint32_t ultimaSerial = 0;
uint32_t ultimoEnvio = 0;
uint16_t sequenciaIR = 0;
uint16_t sequenciaBNO = 0;
uint16_t sequenciaUltra = 0;
float direcaoGraus = 0.0f;
float confianca = 0.0f;
bool bolaDetectada = false;
BNO055 sensorBNO;
TSSP sensorTSSP;
Ultrassonicos sensorUltrassonicos;
bool bnoDisponivel = false;
uint32_t ultimaAtualizacaoBNO = 0;
uint32_t ultimaTentativaBNO = 0;
uint32_t ultimaLeituraUltra = 0;
bool leituraUltraEmAndamento = false;

// Soma as atividades acima do ruido como vetores na circunferencia. O filtro
// exponencial estabiliza a direcao; a razao modulo/soma estima a concentracao
// espacial e rejeita leituras distribuidas que nao indicam um rumo confiavel.
void calcularDirecao() {
  float x = 0.0f;
  float y = 0.0f;
  float somaPesos = 0.0f;

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    peso[i] = fmaxf(0.0f, atividade[i] - LIMIAR_ATIVIDADE);
    const float rad = angulosSensores[i] * DEG_TO_RAD;
    x += peso[i] * cosf(rad);
    y += peso[i] * sinf(rad);
    somaPesos += peso[i];
  }

  if (!filtroIniciado) {
    vetorXFiltrado = x;
    vetorYFiltrado = y;
    somaPesosFiltrada = somaPesos;
    filtroIniciado = true;
  } else {
    vetorXFiltrado = ALPHA * x + (1.0f - ALPHA) * vetorXFiltrado;
    vetorYFiltrado = ALPHA * y + (1.0f - ALPHA) * vetorYFiltrado;
    somaPesosFiltrada =
        ALPHA * somaPesos + (1.0f - ALPHA) * somaPesosFiltrada;
  }

  if (somaPesos <= 0.0f || somaPesosFiltrada <= 0.0f) {
    bolaDetectada = false;
    confianca = 0.0f;
    return;
  }

  const float moduloFiltrado = sqrtf(
      vetorXFiltrado * vetorXFiltrado + vetorYFiltrado * vetorYFiltrado);
  confianca = moduloFiltrado / somaPesosFiltrada;
  if (confianca < CONFIANCA_MIN) {
    bolaDetectada = false;
    return;
  }

  direcaoGraus = atan2f(vetorYFiltrado, vetorXFiltrado) * RAD_TO_DEG;
  if (direcaoGraus < 0.0f) direcaoGraus += 360.0f;
  if (direcaoGraus > 180.0f) direcaoGraus -= 360.0f;
  bolaDetectada = true;
}

// Copia o snapshot da ultima janela TSSP antes de calcular o rumo que sera
// publicado; a janela so muda quando o driver informa que ela foi fechada.
void atualizarAtividadeTSSP() {
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    atividade[i] = sensorTSSP.atividadePercentual(i);
  }

  calcularDirecao();
}

// Serializa atividade e decisao IR no formato comum do protocolo; o checksum
// permite ao Body descartar quadros corrompidos antes de usar a direcao.
void transmitirIR() {
  IRActivityPacket pacote{};
  pacote.magic = IR_PACKET_MAGIC;
  pacote.version = IR_PACKET_VERSION;
  pacote.sequence = sequenciaIR++;
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    pacote.activity[i] = static_cast<uint8_t>(atividade[i] + 0.5f);
  }
  pacote.directionTenthsDegrees = bolaDetectada
      ? static_cast<int16_t>(lroundf(direcaoGraus * 10.0f)) : 0;
  int confiancaPercentual = static_cast<int>(lroundf(confianca * 100.0f));
  if (confiancaPercentual < 0) confiancaPercentual = 0;
  if (confiancaPercentual > 100) confiancaPercentual = 100;
  pacote.confidencePercent = static_cast<uint8_t>(confiancaPercentual);
  pacote.ballDetected = bolaDetectada ? 1 : 0;
  pacote.checksum = irPacketChecksum(pacote);
  Serial1.write(reinterpret_cast<const uint8_t *>(&pacote), sizeof(pacote));
}

// So publica telemetria BNO depois da inicializacao bem-sucedida, evitando
// enviar zeros que poderiam parecer uma orientacao valida ao Body.
void transmitirBNO() {
  if (!bnoDisponivel) return;

  BNODataPacket pacote{};
  pacote.magic = BNO_PACKET_MAGIC;
  pacote.version = BNO_PACKET_VERSION;
  pacote.sequence = sequenciaBNO++;
  pacote.headingDegrees = sensorBNO.getHeading();
  pacote.gyroZDegreesPerSecond = sensorBNO.getGyroZ();
  pacote.checksum = bnoPacketChecksum(pacote);
  Serial1.write(reinterpret_cast<const uint8_t *>(&pacote), sizeof(pacote));
}

void transmitirUltra(const float distanciasCm[Ultrassonicos::QUANTIDADE],
                     const bool leiturasValidas[Ultrassonicos::QUANTIDADE]) {
  UltraDataPacket pacote{};
  pacote.magic = ULTRA_PACKET_MAGIC;
  pacote.version = ULTRA_PACKET_VERSION;
  pacote.sequence = sequenciaUltra++;
  for (uint8_t i = 0; i < Ultrassonicos::QUANTIDADE; ++i) {
    pacote.distanceCm[i] = distanciasCm[i];
    if (leiturasValidas[i]) pacote.validMask |= 1U << i;
  }
  pacote.checksum = ultraPacketChecksum(pacote);
  Serial1.write(reinterpret_cast<const uint8_t *>(&pacote), sizeof(pacote));
}

// Tenta iniciar o sensor e registra o horario mesmo em caso de falha para
// limitar a frequencia de novas tentativas e manter o loop responsivo.
void iniciarBNO() {
  Serial.println("Tentando inicializar BNO055...");
  bnoDisponivel = sensorBNO.begin(BNO_SDA_PIN, BNO_SCL_PIN);
  ultimaTentativaBNO = millis();

  if (bnoDisponivel) {
    ultimaAtualizacaoBNO = ultimaTentativaBNO;
    Serial.println("BNO055 inicializado; telemetria sera enviada ao Body.");
  } else {
    Serial.println(
        "ERRO: BNO055 nao encontrado em SDA=8/SCL=9; nova tentativa em 50 ms. "
        "Body deve manter os motores parados.");
  }
}

// Emite diagnostico legivel para bancada sem alterar os quadros enviados ao Body.
void imprimirDados() {
  Serial.print("LOW%: ");
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    Serial.print(atividade[i], 1);
    if (i < NUM_SENSORES - 1) Serial.print(',');
  }

  Serial.print(" | Bola: ");
  Serial.print(bolaDetectada ? "SIM" : "NAO");
  if (bolaDetectada) {
    Serial.print(" | Direcao: ");
    Serial.print(direcaoGraus, 1);
    Serial.print(" graus");
  }
  Serial.print(" | Confianca: ");
  Serial.println(confianca, 2);
}
} // namespace

// Configura canais seriais e inicializa sensores antes de habilitar a telemetria.
void setup() {
  Serial.begin(USB_SERIAL_BAUD);
  Serial1.begin(HEAD_BODY_UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  iniciarBNO();
  sensorTSSP.begin();
  sensorUltrassonicos.begin();
  Serial.println("Coroa TSSP4038 - 16 sensores");
  Serial.printf(
      "Head UART1: %lu baud, 8N1, RX=18 TX=17; TSSP ativo. "
      "Telemetria BNO055 sera enviada apos inicializacao do sensor.\n",
      static_cast<unsigned long>(HEAD_BODY_UART_BAUD));
}

// Agenda amostragem, atualizacao e transmissao por intervalos independentes
// para que a leitura IR continue responsiva.
void loop() {
  if (sensorTSSP.atualizar()) atualizarAtividadeTSSP();

  const uint32_t agoraMs = millis();
  if (!bnoDisponivel &&
      (uint32_t)(agoraMs - ultimaTentativaBNO) >=
          INTERVALO_TENTATIVA_BNO_MS) {
    iniciarBNO();
  }
  if (bnoDisponivel &&
      (uint32_t)(agoraMs - ultimaAtualizacaoBNO) >= INTERVALO_BNO_MS) {
    ultimaAtualizacaoBNO = agoraMs;
    sensorBNO.update();
  }
  if ((uint32_t)(agoraMs - ultimaLeituraUltra) >= INTERVALO_ULTRA_MS) {
    ultimaLeituraUltra = agoraMs;
    sensorUltrassonicos.iniciarLeitura();
    leituraUltraEmAndamento = true;
  }
  if (leituraUltraEmAndamento) {
    float distanciasCm[Ultrassonicos::QUANTIDADE] = {};
    bool leiturasValidas[Ultrassonicos::QUANTIDADE] = {};
    if (sensorUltrassonicos.atualizar(distanciasCm, leiturasValidas)) {
      transmitirUltra(distanciasCm, leiturasValidas);
      leituraUltraEmAndamento = false;
    }
  }
  if ((uint32_t)(agoraMs - ultimoEnvio) >= INTERVALO_ENVIO_MS) {
    ultimoEnvio = agoraMs;
    transmitirIR();
    transmitirBNO();
  }
  if ((uint32_t)(agoraMs - ultimaSerial) >= SERIAL_MS) {
    ultimaSerial = agoraMs;
    imprimirDados();
  }
}
