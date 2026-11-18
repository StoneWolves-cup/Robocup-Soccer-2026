#include <BatteryEstimator.h>

BatteryEstimator battery;

void setup() {
    Serial.begin(115200);
    battery.begin();
}

void loop() {
    battery.update(UPDATE_PERIOD_MS);
    delay(UPDATE_PERIOD_MS);
}
