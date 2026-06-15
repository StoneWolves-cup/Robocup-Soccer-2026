bool recebeuPrimeiroPacote = false;
unsigned long ultimoPacoteRecebido = 0;

void receberDados(const esp_now_recv_info_t *info,
                  const uint8_t *data,
                  int len)
{
  memcpy(&dados, data, sizeof(dados));

  recebeuPrimeiroPacote = true;
  ultimoPacoteRecebido = millis();
}

void iniciarESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("MAC DO ESP MOTORES: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERRO ao iniciar ESP-NOW");
    while (true);
  }

  esp_now_register_recv_cb(receberDados);

  Serial.println("ESP-NOW receptor pronto!");
}