#ifndef FUZZY_PID_H
#define FUZZY_PID_H

#include <Arduino.h>

// PID com ajuste fuzzy dos ganhos usando erro angular e velocidade do giroscopio.
// Os ganhos base continuam sendo os valores calibraveis do controlador.
class FuzzyPID {
public:
  // Mantem ganhos de base calibraveis e define limites da saida de rotacao.
  FuzzyPID(float kp, float ki, float kd, float outputMin, float outputMax,
           float errorScaleDegrees = 30.0f,
           float gyroScaleDegreesPerSecond = 120.0f);

  // Limpa a memoria integral ao iniciar ou interromper uma correcao.
  void reset();
  // Limita o acumulador integral para reduzir wind-up durante saturacao.
  void setIntegralLimit(float limit);
  // Calcula rotacao usando erro de rumo, gyro Z e intervalo em segundos.
  // dt nao positivo e rejeitado para nao integrar com tempo invalido.
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

  // Gera pertinencias low/medium/high para interpolar as regras fuzzy.
  static void memberships(float value, float &low, float &medium, float &high);
};

#endif
