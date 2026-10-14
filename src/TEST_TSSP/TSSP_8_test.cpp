#include <Arduino.h>
#include <math.h>
#include <PID.h>

const uint8_t SENSOR_COUNT = 8;
const uint8_t sensorPins[SENSOR_COUNT] = {33, 32, 12, 13, 14, 27, 36, 25};

const uint32_t WINDOW_US = 5000;
const uint32_t UPDATE_PERIOD_MS = 20;
const uint8_t FRAME_COUNT = 12;
const float FILTER_ALPHA = 0.2f;

const float TURN_ON_THRESHOLD = 3.0f;
const float TURN_OFF_THRESHOLD = 1.5f;
const uint8_t CONFIRM_TURN_ON = 3;
const uint8_t CONFIRM_TURN_OFF = 5;

const float TARGET_DIRECTION_DEGREES = 0.0f;
const float KP = 1.0f;
const float KI = 0.05f;
const float KD = 0.0f;
const float PID_OUTPUT_MIN = -100.0f;
const float PID_OUTPUT_MAX = 100.0f;
const float PID_INTEGRAL_LIMIT = 50.0f;
PID directionPid(KP, KI, KD, PID_OUTPUT_MIN, PID_OUTPUT_MAX);

uint32_t lowSamples[SENSOR_COUNT] = {0};
float filteredActivity[SENSOR_COUNT] = {0};
float activityFrames[SENSOR_COUNT][FRAME_COUNT] = {{0}};

uint32_t totalSamples = 0;
uint32_t windowStartUs = 0;
uint32_t lastUpdateMs = 0;
uint8_t frameIndex = 0;
uint8_t frameCount = 0;
uint8_t turnOnCount = 0;
uint8_t turnOffCount = 0;
bool firstFrame = true;
bool ballDetected = false;

void sampleSensors() {
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    if (digitalRead(sensorPins[sensor]) == LOW) {
      lowSamples[sensor]++;
    }
  }
  totalSamples++;
}

void updateActivityFrame() {
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    float activity = totalSamples > 0
        ? 100.0f * lowSamples[sensor] / totalSamples
        : 0.0f;

    if (firstFrame) {
      filteredActivity[sensor] = activity;
    } else {
      filteredActivity[sensor] =
          FILTER_ALPHA * activity +
          (1.0f - FILTER_ALPHA) * filteredActivity[sensor];
    }

    activityFrames[sensor][frameIndex] = filteredActivity[sensor];
    lowSamples[sensor] = 0;
  }

  firstFrame = false;
  totalSamples = 0;
  frameIndex = (frameIndex + 1) % FRAME_COUNT;
  if (frameCount < FRAME_COUNT) {
    frameCount++;
  }
}

float averageActivity(uint8_t sensor) {
  float sum = 0.0f;
  for (uint8_t frame = 0; frame < frameCount; frame++) {
    sum += activityFrames[sensor][frame];
  }
  return frameCount > 0 ? sum / frameCount : 0.0f;
}

void updateDetection(const float activity[SENSOR_COUNT], float &maxActivity) {
  maxActivity = 0.0f;
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    if (activity[sensor] > maxActivity) {
      maxActivity = activity[sensor];
    }
  }

  if (!ballDetected) {
    turnOffCount = 0;
    if (maxActivity >= TURN_ON_THRESHOLD) {
      if (turnOnCount < CONFIRM_TURN_ON) {
        turnOnCount++;
      }
    } else {
      turnOnCount = 0;
    }

    if (turnOnCount >= CONFIRM_TURN_ON) {
      ballDetected = true;
      turnOnCount = 0;
    }
  } else {
    turnOnCount = 0;
    if (maxActivity <= TURN_OFF_THRESHOLD) {
      if (turnOffCount < CONFIRM_TURN_OFF) {
        turnOffCount++;
      }
    } else {
      turnOffCount = 0;
    }

    if (turnOffCount >= CONFIRM_TURN_OFF) {
      ballDetected = false;
      turnOffCount = 0;
    }
  }
}

bool calculateDirection(const float activity[SENSOR_COUNT], float &angleDegrees) {
  float vectorX = 0.0f;
  float vectorY = 0.0f;

  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    float weight = activity[sensor] - TURN_OFF_THRESHOLD;
    if (weight <= 0.0f) {
      continue;
    }

    float angleRadians = sensor * (PI / 4.0f);
    vectorX += weight * cosf(angleRadians);
    vectorY += weight * sinf(angleRadians);
  }

  if (sqrtf(vectorX * vectorX + vectorY * vectorY) < 0.01f) {
    return false;
  }

  angleDegrees = atan2f(vectorY, vectorX) * (180.0f / PI);
  if (angleDegrees < 0.0f) {
    angleDegrees += 360.0f;
  }
  return true;
}

float calculateAngularError(float targetDegrees, float currentDegrees) {
  float error = targetDegrees - currentDegrees;
  while (error > 180.0f) {
    error -= 360.0f;
  }
  while (error < -180.0f) {
    error += 360.0f;
  }
  return error;
}

void printStatus(float dt) {
  float activity[SENSOR_COUNT];
  float maxActivity;
  float directionDegrees;
  float angularError = 0.0f;
  float pidOutput = 0.0f;

  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    activity[sensor] = averageActivity(sensor);
  }

  updateDetection(activity, maxActivity);
  bool directionValid = calculateDirection(activity, directionDegrees);
  if (ballDetected && directionValid) {
    angularError = calculateAngularError(
        TARGET_DIRECTION_DEGREES,
        directionDegrees
    );
    pidOutput = directionPid.calculate(angularError, 0.0f, dt);
  } else {
    directionPid.reset();
  }

  Serial.print("Atividade: ");
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    Serial.print("GPIO");
    Serial.print(sensorPins[sensor]);
    Serial.print('=');
    Serial.print(activity[sensor], 1);
    Serial.print('%');
    if (sensor + 1 < SENSOR_COUNT) {
      Serial.print(" | ");
    }
  }

  Serial.print(" | Bola: ");
  Serial.print(ballDetected ? "DETECTADA" : "AUSENTE");
  Serial.print(" | Direcao: ");
  if (ballDetected && directionValid) {
    Serial.print(directionDegrees, 1);
    Serial.print(" graus | Erro: ");
    Serial.print(angularError, 1);
    Serial.print(" graus | PID: ");
    Serial.print(pidOutput, 1);
    Serial.println(" (comando de giro)");
  } else {
    Serial.println("indisponivel | PID: 0.0");
  }
}

void setup() {
  Serial.begin(115200);
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    pinMode(sensorPins[sensor], INPUT);
  }

  directionPid.setIntegralLimit(PID_INTEGRAL_LIMIT);
  windowStartUs = micros();
  lastUpdateMs = millis();
  Serial.println("Teste de 8 sensores TSSP4038 iniciado");
  Serial.println("Ordem horaria: GPIO33, 32, 12, 13, 14, 27, 36, 25");
}

void loop() {
  sampleSensors();

  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - windowStartUs) >= WINDOW_US) {
    updateActivityFrame();
    windowStartUs = nowUs;
  }

  uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastUpdateMs) >= UPDATE_PERIOD_MS) {
    uint32_t elapsedMs = nowMs - lastUpdateMs;
    lastUpdateMs = nowMs;
    printStatus(elapsedMs / 1000.0f);
  }
}