#include "PID.h"

PID::PID(float _kP, float _kI, float _kD){
    setParameters(_kP, _kI, _kD);
    setSetPoint(0);

    outMax = 255;
    outMin = 0;

}

PID::PID(float _kP, float _kI, float _kD, float _outMin, float _outMax){
    setParameters(_kP, _kI, _kD);

    outMax = _outMax;
    outMin = _outMin;
}

void PID::setSetPoint(float _setPoint){
	setPoint = _setPoint;
}

void PID::setKP(float _kP){  kP = _kP; }
void PID::setKI(float _kI){  kI = _kI; }
void PID::setKD(float _kD){  kD = _kD; }  

void PID::setParameters(float _kP, float _kI, float _kD){
	setKP(_kP);
	setKI(_kI);
	setKD(_kD);
}

void PID::setLimits(int _outMin, int _outMax){
	outMin = _outMin;
	outMax = _outMax;
}

void PID::addInput(float _newInput){
	newInput = _newInput;
}

float PID::compute(){

	unsigned long time = (millis() - lastTime); 
	lastTime = millis();
	float dTime = time/1000.0f;

	compute(dTime);

}

float PID::compute(float dTime){

	dT = dTime;

	// calculate error
	error = (setPoint - newInput);

	// Implementation of P
	P = kP * error;

	// Implementation of I
	// I += kI * (lastError + error) * dTime / 2;
	I += kI * error * dTime;
	I = constrain(I, outMin, outMax);

	// Implementation of D
	D = kD * (error - lastError) / dTime;

	// Calculate output PID
	output = P + I + D;
	output = constrain(output, outMin, outMax);

	lastError = error;

	return output;
}

void PID::reset(){
	P = I = D = output = 0;
}

float PID::getKP(){return kP;}
float PID::getKI(){return kI;}
float PID::getKD(){return kD;}
float PID::getSetPoint(){return setPoint;}

void PID::debug(){
	LOG(P);			LOG(TAB);
	LOG(I);			LOG(TAB);
	LOG(D);			LOG(TAB);
	LOG(output);	LOG(TAB);
	LOG(dT, 4);		LOG(TAB);
	LOG(kP, 3);		LOG(TAB);
	LOG(kI, 3);		LOG(TAB);
	LOG(kD, 3);		LOG(NEW_LINE);
}