#include "Stabilizer.h"

PID Stabilizer::pidThrottle;
PID Stabilizer::pidYaw;
PID Stabilizer::pidPitch;
PID Stabilizer::pidRoll;

unsigned long Stabilizer::lastComputeTime;

void Stabilizer::begin(){
	LOG("\nInitializing Stabilizer...\n");

	pidThrottle.setLimits(MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
	pidYaw.setLimits(-(MAX_DUTY_CYCLE - MIN_DUTY_CYCLE), (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE));
	pidPitch.setLimits(-(MAX_DUTY_CYCLE - MIN_DUTY_CYCLE), (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE));
	pidRoll.setLimits(-(MAX_DUTY_CYCLE - MIN_DUTY_CYCLE), (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE));

	Motors::stop();

	lastComputeTime = millis();
}

void Stabilizer::throttleUpdateSetPoint(float setPoint){
	pidThrottle.setSetPoint(setPoint);
}

void Stabilizer::throttleUpdateInput(float input){
	pidThrottle.addInput(input);
}

void Stabilizer::yawUpdateSetPoint(float setPoint){
	pidYaw.setSetPoint(setPoint);
}

void Stabilizer::yawUpdateInput(float input){
	pidYaw.addInput(input);
}

void Stabilizer::pitchUpdateSetPoint(float setPoint){
	pidPitch.setSetPoint(setPoint);
}

void Stabilizer::pitchUpdateInput(float input){
	pidPitch.addInput(input);
}

void Stabilizer::rollUpdateSetPoint(float setPoint){
	pidRoll.setSetPoint(setPoint);
}

void Stabilizer::rollUpdateInput(float input){
	pidRoll.addInput(input);
}

void Stabilizer::resetPID(){
	pidThrottle.reset();
	pidYaw.reset();
	pidPitch.reset();
	pidRoll.reset();
}

void Stabilizer::stabilize(){

	unsigned long time = (millis() - lastComputeTime); 
	lastComputeTime = millis();
	float dt = time/1000.0f;

	float outThrottle = mapFloat(pidThrottle.getSetPoint(), THROTTLE_MIN, THROTTLE_MAX, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
	float outYaw = 0;
	float outPitch = pidPitch.compute(dt);
	float outRoll = pidRoll.compute(dt);


	uint32_t powers[] = {
		outThrottle + outRoll,
		0,
		outThrottle - outRoll,
		0
		// outThrottle + outYaw + outPitch + outRoll,		// MOTOR 1
		// outThrottle - outYaw + outPitch - outRoll,		// MOTOR 2
		// outThrottle + outYaw - outPitch - outRoll,		// MOTOR 3
		// outThrottle - outYaw - outPitch + outRoll		// MOTOR 4
	};

	Motors::setPower(powers);

}