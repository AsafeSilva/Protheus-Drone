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

	// pidPitch.setParameters(0.05f, 0.01f, 0.01f);
	// pidRoll.setParameters(0.05f, 0.01f, 0.01f);
	// pidYaw.setParameters(0.300f, 0.025f, 0.025f);

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

void Stabilizer::reset(){
	pidThrottle.reset();
	pidYaw.reset();
	pidPitch.reset();
	pidRoll.reset();
}

void Stabilizer::stabilize(){

	unsigned long time = (millis() - lastComputeTime); 
	lastComputeTime = millis();
	float dt = time/1000.0f;

	float outThrottle = pidThrottle.getSetPoint();
	float outYaw = pidYaw.compute(dt);
	float outPitch = pidPitch.compute(dt);
	float outRoll = pidRoll.compute(dt);

	uint32_t m1 = outThrottle + outPitch + outYaw;
	uint32_t m2 = outThrottle - outRoll - outYaw;
	uint32_t m3 = outThrottle - outPitch + outYaw;
	uint32_t m4 = outThrottle + outRoll - outYaw;

	uint32_t powers[] = {
		m1, m2, m3, m4
	};

	Motors::setPower(powers);

}