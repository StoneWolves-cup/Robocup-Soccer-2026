#include <FuzzyPID.h>
#include <math.h>

namespace {
// Linhas: erro pequeno/medio/grande. Colunas: velocidade angular
// pequena/media/grande. Multiplicadores obtidos por inferencia fuzzy
// Sugeno de ordem zero (regras ponderadas pelas pertinencias).
constexpr float KP_RULES[3][3] = {
    {0.70f, 0.75f, 0.80f},
    {1.00f, 1.05f, 1.10f},
    {1.35f, 1.40f, 1.45f}};
constexpr float KI_RULES[3][3] = {
    {1.00f, 0.75f, 0.40f},
    {0.80f, 0.50f, 0.25f},
    {0.50f, 0.25f, 0.10f}};
constexpr float KD_RULES[3][3] = {
    {0.70f, 1.00f, 1.30f},
    {0.85f, 1.15f, 1.50f},
    {1.00f, 1.35f, 1.70f}};

float inferGain(const float rules[3][3], const float errorSet[3],
                const float gyroSet[3]) {
  float weighted = 0.0f;
  float totalWeight = 0.0f;
  for (uint8_t e = 0; e < 3; ++e) {
    for (uint8_t g = 0; g < 3; ++g) {
      const float weight = errorSet[e] * gyroSet[g];
      weighted += weight * rules[e][g];
      totalWeight += weight;
    }
  }
  return totalWeight > 0.0f ? weighted / totalWeight : 1.0f;
}
}

FuzzyPID::FuzzyPID(float kp, float ki, float kd, float outputMin,
                   float outputMax, float errorScaleDegrees,
                   float gyroScaleDegreesPerSecond)
    : kp_(kp), ki_(ki), kd_(kd), outputMin_(outputMin), outputMax_(outputMax),
      integralLimit_(100.0f), integral_(0.0f),
      errorScale_(fmaxf(errorScaleDegrees, 0.001f)),
      gyroScale_(fmaxf(gyroScaleDegreesPerSecond, 0.001f)) {}

void FuzzyPID::reset() { integral_ = 0.0f; }

void FuzzyPID::setIntegralLimit(float limit) {
  integralLimit_ = fabsf(limit);
}

void FuzzyPID::memberships(float value, float &low, float &medium, float &high) {
  value = fminf(1.0f, fmaxf(0.0f, value));
  low = fmaxf(0.0f, 1.0f - 2.0f * value);
  medium = 1.0f - fabsf(2.0f * value - 1.0f);
  high = fmaxf(0.0f, 2.0f * value - 1.0f);
}

float FuzzyPID::calculate(float error, float gyroZDegreesPerSecond, float dt) {
  if (dt <= 0.0f) return 0.0f;

  float eLow, eMedium, eHigh;
  float gLow, gMedium, gHigh;
  memberships(fabsf(error) / errorScale_, eLow, eMedium, eHigh);
  memberships(fabsf(gyroZDegreesPerSecond) / gyroScale_,
              gLow, gMedium, gHigh);
  const float errorSet[3] = {eLow, eMedium, eHigh};
  const float gyroSet[3] = {gLow, gMedium, gHigh};

  const float kp = kp_ * inferGain(KP_RULES, errorSet, gyroSet);
  const float ki = ki_ * inferGain(KI_RULES, errorSet, gyroSet);
  const float kd = kd_ * inferGain(KD_RULES, errorSet, gyroSet);

  integral_ += error * dt;
  integral_ = fminf(integralLimit_, fmaxf(-integralLimit_, integral_));

  float output = kp * error + ki * integral_ - kd * gyroZDegreesPerSecond;
  return fminf(outputMax_, fmaxf(outputMin_, output));
}
