#pragma once

#include <Arduino.h>

#include "_config.h"

class PID{

private:
  float kP, kI, kD;
  float P, I, D;
  float newInput;
  float output;
  float outMin, outMax;
  float setPoint;
  float error, lastError;
  unsigned long lastTime;

  float dT;


public:

  PID(float _kP = 0.0, float _kI = 0.0, float _kD = 0.0);
  PID(float _kP, float _kI, float _kD, float _outMin, float _outMax);

  void setSetPoint(float _setPoint);

  void setKP(float _kP);
  void setKI(float _kI);
  void setKD(float _kD);  

  void setParameters(float _kP, float _kI, float _kD);
  void setLimits(int _outMin, int _outMax);

  void addInput(float _newInput);

  float compute();
  float compute(float dTime);

  void reset();

  float getKP();
  float getKI();
  float getKD();
  float getSetPoint();

  void debug();

};
