#ifndef FUZZY_PID_H
#define FUZZY_PID_H

#include <Arduino.h>

// PID com ajuste fuzzy dos ganhos usando erro angular e velocidade do giroscopio.
// Os ganhos base continuam sendo os valores calibraveis do controlador.
class FuzzyPID {
public:
  FuzzyPID(float kp, float ki, float kd, float outputMin, float outputMax,
           float errorScaleDegrees = 30.0f,
           float gyroScaleDegreesPerSecond = 120.0f);

  void reset();
  void setIntegralLimit(float limit);
  float calculate(float error, float gyroZDegreesPerSecond, float dt);

private:
  float kp_;
  float ki_;
  float kd_;
  float outputMin_;
  float outputMax_;
  float integralLimit_;
  float integral_;
  float errorScale_;
  float gyroScale_;

  static void memberships(float value, float &low, float &medium, float &high);
};

#endif
