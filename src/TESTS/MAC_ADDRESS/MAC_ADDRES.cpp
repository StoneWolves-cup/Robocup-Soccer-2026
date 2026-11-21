#include <WiFi.h>
#include <esp_wifi.h>

void setup() {
  Serial.begin(115200);
  delay(1000); // Allow time for USB CDC Serial to initialize on ESP32-S3

  WiFi.mode(WIFI_STA);
  WiFi.STA.begin();

  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  
  if (ret == ESP_OK) {
    Serial.printf("ESP32-S3 MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

void loop() {}