#include "Stabilizer.h"

Fuzzy* Stabilizer::fuzzyPitch;
Fuzzy* Stabilizer::fuzzyRoll;

FuzzyPID Stabilizer::pitch;
FuzzyPID Stabilizer::roll;

PID Stabilizer::throttle;
PID Stabilizer::yaw;

unsigned long Stabilizer::lastComputeTime;

void Stabilizer::begin(){
	LOG("\nInitializing Stabilizer...\n");

	throttle.setLimits(MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
	yaw.setLimits(-(MAX_DUTY_CYCLE - MIN_DUTY_CYCLE), (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE));

	roll.outMin = pitch.outMin = -(MAX_DUTY_CYCLE - MIN_DUTY_CYCLE);
	roll.outMax = pitch.outMax = (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE);

	// throttle.setParameters(0, 0, 0);
	// yaw.setParameters(0, 0, 0);
	pitch.kP = 0;	pitch.kI = 0;	pitch.kD = 0;
	roll.kP = 0;	roll.kI = 0;	roll.kD = 0;

	fuzzyPitchInit();
	fuzzyRollInit();

	Motors::stop();

	lastComputeTime = millis();
}

void Stabilizer::throttleUpdateSetPoint(float setPoint){
	throttle.setSetPoint(setPoint);
}

void Stabilizer::throttleUpdateInput(float input){
	throttle.addInput(input);
}

void Stabilizer::yawUpdateSetPoint(float setPoint){
	yaw.setSetPoint(setPoint);
}

void Stabilizer::yawUpdateInput(float input){
	yaw.addInput(input);
}

void Stabilizer::pitchUpdateSetPoint(float setPoint){
	pitch.setPoint = setPoint;
}

void Stabilizer::pitchUpdateInput(float input){
	pitch.input = input;
}

void Stabilizer::rollUpdateSetPoint(float setPoint){
	roll.setPoint = setPoint;
}

void Stabilizer::rollUpdateInput(float input){
	roll.input = input;
}

void Stabilizer::reset(){
	throttle.reset();
	yaw.reset();
	pitch.P = pitch.I = pitch.D = pitch.lastError = 0;
	roll.P = roll.I = roll.D = roll.lastError = 0;
}

void Stabilizer::stabilize(){

	unsigned long time = (millis() - lastComputeTime); 
	lastComputeTime = millis();
	float dt = time/1000.0f;

	float outThrottle = throttle.getSetPoint();
	float outYaw = yaw.compute(dt);
	float outPitch = computesFuzzyPID(fuzzyPitch, &pitch, dt);
	float outRoll = computesFuzzyPID(fuzzyRoll, &roll, dt);

	uint32_t m1 = outThrottle - outRoll;
	uint32_t m2 = outThrottle;
	uint32_t m3 = outThrottle + outRoll;
	uint32_t m4 = outThrottle;

	uint32_t powers[] = {
		m1, 0, m3, 0
	};

	Motors::setPower(powers);


	// float error = pitchSetPoint - pitchInput; 

	// fuzzyPitch->setInput(1, error);
	// fuzzyPitch->fuzzify();
	// float outPitchFuzzy = fuzzyPitch->defuzzify(1) * pidRoll.getKP();

	// float outPitchPID = pidPitch.compute();
 
	// uint32_t powers[] = {
	// 	0,
	// 	outThrottle + outPitchFuzzy - outPitchPID,
	// 	0,
	// 	outThrottle - outPitchFuzzy + outPitchPID
	// };

	// Motors::setPower(powers);

}

float Stabilizer::computesFuzzyPID(Fuzzy* fuzzy, FuzzyPID* fuzzyPID, float dTime){

	// Error
	float error = fuzzyPID->setPoint - fuzzyPID->input;

	// Fuzzy
	fuzzy->setInput(1, error);
	fuzzy->fuzzify();
	float outFuzzy = fuzzy->defuzzify(1);

	// Proportional
	fuzzyPID->P = outFuzzy * fuzzyPID->kP;

	// Integrative
	fuzzyPID->I += fuzzyPID->kI * error * dTime;
	fuzzyPID->I = constrain(fuzzyPID->I, fuzzyPID->outMin, fuzzyPID->outMax);

	// Derivative
	fuzzyPID->D = fuzzyPID->kD * (error - fuzzyPID->lastError) / dTime;
	fuzzyPID->lastError = error;

	// Output
	float output = fuzzyPID->P - fuzzyPID->I - fuzzyPID->D;
	output = constrain(output, fuzzyPID->outMin, fuzzyPID->outMax);

	return output;

}

void Stabilizer::fuzzyPitchInit(){

	fuzzyPitch = new Fuzzy();

	FuzzyInput* angle = new FuzzyInput(1);

	FuzzySet* negativeAngleHigh = new FuzzySet(-1476, -1476, -1148, -983.7);
	angle->addFuzzySet(negativeAngleHigh);
	FuzzySet* negativeAngleMiddle = new FuzzySet(-1148, -983.7, -492, -328);
	angle->addFuzzySet(negativeAngleMiddle);
	FuzzySet* negativeAngleLow = new FuzzySet(-574, -410, -246, 0);
	angle->addFuzzySet(negativeAngleLow);
	FuzzySet* angleZero = new FuzzySet(-114.8, -32.8, 32.8, 114.8);
	angle->addFuzzySet(angleZero);
	FuzzySet* angleLow = new FuzzySet(0, 246, 410, 574);
	angle->addFuzzySet(angleLow);
	FuzzySet* angleMiddle = new FuzzySet(328, 492, 983.7, 1148);
	angle -> addFuzzySet(angleMiddle);
	FuzzySet* angleHigh = new FuzzySet(983.7, 1148, 1476, 1476);
	angle->addFuzzySet(angleHigh);

	fuzzyPitch->addFuzzyInput(angle);


	FuzzyOutput* pwm = new FuzzyOutput(1);

	FuzzySet* negativePwmHigh = new FuzzySet(-205, -205, -160, -120);
	pwm-> addFuzzySet( negativePwmHigh);
	FuzzySet* negativePwmMiddle = new FuzzySet(-140, -120, -90, -60);
	pwm-> addFuzzySet( negativePwmMiddle);
	FuzzySet* negativePwmLow = new FuzzySet(-90, -60, -30, -10);
	pwm-> addFuzzySet( negativePwmLow);
	FuzzySet* pwmZero = new FuzzySet(-15, -5, 5, 15);
	pwm-> addFuzzySet(pwmZero);
	FuzzySet* pwmLow = new FuzzySet(10, 30, 60, 90);
	pwm-> addFuzzySet(pwmLow);
	FuzzySet* pwmMiddle = new FuzzySet(60, 90, 120, 140);
	pwm-> addFuzzySet(pwmMiddle);
	FuzzySet* pwmHigh = new FuzzySet(120, 160, 205, 205);
	pwm-> addFuzzySet(pwmHigh);

	fuzzyPitch->addFuzzyOutput(pwm);

	////////////////Rule01/////////////////////////////
	FuzzyRuleAntecedent* ifNegativeAngleHigh = new FuzzyRuleAntecedent();
	ifNegativeAngleHigh->joinSingle(negativeAngleHigh);
	FuzzyRuleConsequent* thenPwmHigh = new FuzzyRuleConsequent();
	thenPwmHigh->addOutput(pwmHigh);
	FuzzyRule* fuzzyRule01 = new FuzzyRule(1, ifNegativeAngleHigh, thenPwmHigh);

	fuzzyPitch->addFuzzyRule(fuzzyRule01);

	////////////////Rule02/////////////////////////////
	FuzzyRuleAntecedent* ifNegativeAngleMiddle = new FuzzyRuleAntecedent();
	ifNegativeAngleMiddle->joinSingle(negativeAngleMiddle);
	FuzzyRuleConsequent* thenPwmMiddle = new FuzzyRuleConsequent();
	thenPwmMiddle->addOutput(pwmMiddle);
	FuzzyRule* fuzzyRule02 = new FuzzyRule(2, ifNegativeAngleMiddle, thenPwmMiddle);

	fuzzyPitch->addFuzzyRule(fuzzyRule02);

	////////////////Rule03/////////////////////////////
	FuzzyRuleAntecedent* ifNegativeAngleLow = new FuzzyRuleAntecedent();
	ifNegativeAngleLow->joinSingle(negativeAngleLow);
	FuzzyRuleConsequent* thenPwmLow = new FuzzyRuleConsequent();
	thenPwmLow->addOutput(pwmLow);
	FuzzyRule* fuzzyRule03 = new FuzzyRule(3, ifNegativeAngleLow, thenPwmLow);

	fuzzyPitch->addFuzzyRule(fuzzyRule03);

	////////////////Rule04/////////////////////////////
	FuzzyRuleAntecedent* ifAngleZero= new FuzzyRuleAntecedent();
	ifAngleZero->joinSingle(angleZero);
	FuzzyRuleConsequent* thenPwmZero = new FuzzyRuleConsequent();
	thenPwmZero->addOutput(pwmZero);
	FuzzyRule* fuzzyRule04 = new FuzzyRule(4, ifAngleZero, thenPwmZero);

	fuzzyPitch->addFuzzyRule(fuzzyRule04);

	////////////////Rule05/////////////////////////////
	FuzzyRuleAntecedent* ifAngleLow = new FuzzyRuleAntecedent();
	ifAngleLow->joinSingle(angleLow);
	FuzzyRuleConsequent* thenNegativePwmLow = new FuzzyRuleConsequent();
	thenNegativePwmLow->addOutput(negativePwmLow);
	FuzzyRule* fuzzyRule05 = new FuzzyRule(5, ifAngleLow, thenNegativePwmLow);

	fuzzyPitch->addFuzzyRule(fuzzyRule05);

	////////////////Rule06/////////////////////////////
	FuzzyRuleAntecedent* ifAngleMiddle = new FuzzyRuleAntecedent();
	ifAngleMiddle->joinSingle(angleMiddle);
	FuzzyRuleConsequent* thenNegativePwmMiddle = new FuzzyRuleConsequent();
	thenNegativePwmMiddle->addOutput(negativePwmMiddle);
	FuzzyRule* fuzzyRule06 = new FuzzyRule(6, ifAngleMiddle, thenNegativePwmMiddle);

	fuzzyPitch->addFuzzyRule(fuzzyRule06);

	////////////////Rule07/////////////////////////////
	FuzzyRuleAntecedent* ifAngleHigh = new FuzzyRuleAntecedent();
	ifAngleHigh->joinSingle(angleHigh);
	FuzzyRuleConsequent* thenNegativePwmHigh = new FuzzyRuleConsequent();
	thenNegativePwmHigh->addOutput(negativePwmHigh);
	FuzzyRule* fuzzyRule07 = new FuzzyRule(7, ifAngleHigh, thenNegativePwmHigh);

	fuzzyPitch->addFuzzyRule(fuzzyRule07);
}

void Stabilizer::fuzzyRollInit(){

	fuzzyRoll = new Fuzzy();

	FuzzyInput* angle = new FuzzyInput(1);

	FuzzySet* negativeAngleHigh = new FuzzySet(-1476, -1476, -1148, -983.7);
	angle->addFuzzySet(negativeAngleHigh);
	FuzzySet* negativeAngleMiddle = new FuzzySet(-1148, -983.7, -492, -328);
	angle->addFuzzySet(negativeAngleMiddle);
	FuzzySet* negativeAngleLow = new FuzzySet(-574, -410, -246, 0);
	angle->addFuzzySet(negativeAngleLow);
	FuzzySet* angleZero = new FuzzySet(-114.8, -32.8, 32.8, 114.8);
	angle->addFuzzySet(angleZero);
	FuzzySet* angleLow = new FuzzySet(0, 246, 410, 574);
	angle->addFuzzySet(angleLow);
	FuzzySet* angleMiddle = new FuzzySet(328, 492, 983.7, 1148);
	angle -> addFuzzySet(angleMiddle);
	FuzzySet* angleHigh = new FuzzySet(983.7, 1148, 1476, 1476);
	angle->addFuzzySet(angleHigh);

	fuzzyRoll->addFuzzyInput(angle);


	FuzzyOutput* pwm = new FuzzyOutput(1);

	FuzzySet* negativePwmHigh = new FuzzySet(-205, -205, -160, -120);
	pwm-> addFuzzySet( negativePwmHigh);
	FuzzySet* negativePwmMiddle = new FuzzySet(-140, -120, -90, -60);
	pwm-> addFuzzySet( negativePwmMiddle);
	FuzzySet* negativePwmLow = new FuzzySet(-90, -60, -30, -10);
	pwm-> addFuzzySet( negativePwmLow);
	FuzzySet* pwmZero = new FuzzySet(-15, -5, 5, 15);
	pwm-> addFuzzySet(pwmZero);
	FuzzySet* pwmLow = new FuzzySet(10, 30, 60, 90);
	pwm-> addFuzzySet(pwmLow);
	FuzzySet* pwmMiddle = new FuzzySet(60, 90, 120, 140);
	pwm-> addFuzzySet(pwmMiddle);
	FuzzySet* pwmHigh = new FuzzySet(120, 160, 205, 205);
	pwm-> addFuzzySet(pwmHigh);

	fuzzyRoll->addFuzzyOutput(pwm);

	////////////////Rule01/////////////////////////////
	FuzzyRuleAntecedent* ifNegativeAngleHigh = new FuzzyRuleAntecedent();
	ifNegativeAngleHigh->joinSingle(negativeAngleHigh);
	FuzzyRuleConsequent* thenPwmHigh = new FuzzyRuleConsequent();
	thenPwmHigh->addOutput(pwmHigh);
	FuzzyRule* fuzzyRule01 = new FuzzyRule(1, ifNegativeAngleHigh, thenPwmHigh);

	fuzzyRoll->addFuzzyRule(fuzzyRule01);

	////////////////Rule02/////////////////////////////
	FuzzyRuleAntecedent* ifNegativeAngleMiddle = new FuzzyRuleAntecedent();
	ifNegativeAngleMiddle->joinSingle(negativeAngleMiddle);
	FuzzyRuleConsequent* thenPwmMiddle = new FuzzyRuleConsequent();
	thenPwmMiddle->addOutput(pwmMiddle);
	FuzzyRule* fuzzyRule02 = new FuzzyRule(2, ifNegativeAngleMiddle, thenPwmMiddle);

	fuzzyRoll->addFuzzyRule(fuzzyRule02);

	////////////////Rule03/////////////////////////////
	FuzzyRuleAntecedent* ifNegativeAngleLow = new FuzzyRuleAntecedent();
	ifNegativeAngleLow->joinSingle(negativeAngleLow);
	FuzzyRuleConsequent* thenPwmLow = new FuzzyRuleConsequent();
	thenPwmLow->addOutput(pwmLow);
	FuzzyRule* fuzzyRule03 = new FuzzyRule(3, ifNegativeAngleLow, thenPwmLow);

	fuzzyRoll->addFuzzyRule(fuzzyRule03);

	////////////////Rule04/////////////////////////////
	FuzzyRuleAntecedent* ifAngleZero= new FuzzyRuleAntecedent();
	ifAngleZero->joinSingle(angleZero);
	FuzzyRuleConsequent* thenPwmZero = new FuzzyRuleConsequent();
	thenPwmZero->addOutput(pwmZero);
	FuzzyRule* fuzzyRule04 = new FuzzyRule(4, ifAngleZero, thenPwmZero);

	fuzzyRoll->addFuzzyRule(fuzzyRule04);

	////////////////Rule05/////////////////////////////
	FuzzyRuleAntecedent* ifAngleLow = new FuzzyRuleAntecedent();
	ifAngleLow->joinSingle(angleLow);
	FuzzyRuleConsequent* thenNegativePwmLow = new FuzzyRuleConsequent();
	thenNegativePwmLow->addOutput(negativePwmLow);
	FuzzyRule* fuzzyRule05 = new FuzzyRule(5, ifAngleLow, thenNegativePwmLow);

	fuzzyRoll->addFuzzyRule(fuzzyRule05);

	////////////////Rule06/////////////////////////////
	FuzzyRuleAntecedent* ifAngleMiddle = new FuzzyRuleAntecedent();
	ifAngleMiddle->joinSingle(angleMiddle);
	FuzzyRuleConsequent* thenNegativePwmMiddle = new FuzzyRuleConsequent();
	thenNegativePwmMiddle->addOutput(negativePwmMiddle);
	FuzzyRule* fuzzyRule06 = new FuzzyRule(6, ifAngleMiddle, thenNegativePwmMiddle);

	fuzzyRoll->addFuzzyRule(fuzzyRule06);

	////////////////Rule07/////////////////////////////
	FuzzyRuleAntecedent* ifAngleHigh = new FuzzyRuleAntecedent();
	ifAngleHigh->joinSingle(angleHigh);
	FuzzyRuleConsequent* thenNegativePwmHigh = new FuzzyRuleConsequent();
	thenNegativePwmHigh->addOutput(negativePwmHigh);
	FuzzyRule* fuzzyRule07 = new FuzzyRule(7, ifAngleHigh, thenNegativePwmHigh);

	fuzzyRoll->addFuzzyRule(fuzzyRule07);
}