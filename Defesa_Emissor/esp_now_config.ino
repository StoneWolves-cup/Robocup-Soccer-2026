void envioConfirmado(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("ESP-NOW Status: ");

  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("SUCESSO");
  } else {
    Serial.println("FALHOU");
  }
}

void iniciarESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("MAC DO ESP CEREBRO: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERRO ao iniciar ESP-NOW");
    while (true);
  }

  esp_now_register_send_cb(envioConfirmado);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, enderecoReceptor, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERRO ao adicionar ESP receptor");
    while (true);
  }

  Serial.println("ESP-NOW iniciado!");
}

void enviarDados() {
  esp_err_t resultado = esp_now_send(
    enderecoReceptor,
    (uint8_t *)&dados,
    sizeof(dados)
  );

  if (resultado != ESP_OK) {
    Serial.print("Erro ao enviar dados: ");
    Serial.println(resultado);
  }
}