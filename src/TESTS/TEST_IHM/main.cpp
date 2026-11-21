#include <ESPNowLink.h>

void IHM_begin();
void IHM_update();

void setup() {
  IHM_begin();
  ESPNowLink::begin();
}

void loop() {
  IHM_update();
  ESPNowLink::update();
}
