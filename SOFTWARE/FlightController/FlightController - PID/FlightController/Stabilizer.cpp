#include "Stabilizer.h"

PID Stabilizer::pidThrottle;
PID Stabilizer::pidYaw;
PID Stabilizer::pidPitch;
PID Stabilizer::pidRoll;
float Stabilizer::outThrottle = 0;
float Stabilizer::outYaw = 0;
float Stabilizer::outPitch = 0;
float Stabilizer::outRoll = 0;

unsigned long Stabilizer::lastComputeTime;

void Stabilizer::begin(){
	LOG("\nInitializing Stabilizer...\n");

	pidThrottle.setLimits(MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
	pidYaw.setLimits(-(MAX_DUTY_CYCLE - MIN_DUTY_CYCLE), (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE));
	pidPitch.setLimits(-(MAX_DUTY_CYCLE - MIN_DUTY_CYCLE), (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE));
	pidRoll.setLimits(-(MAX_DUTY_CYCLE - MIN_DUTY_CYCLE), (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE));

#ifdef PITCH_KP
	pidPitch.setParameters(PITCH_KP, PITCH_KI, PITCH_KD);
#endif

#ifdef ROLL_KP
	pidRoll.setParameters(ROLL_KP, ROLL_KI, ROLL_KD);
#endif

#ifdef YAW_KP
	pidYaw.setParameters(YAW_KP, YAW_KI, YAW_KD);
#endif

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

void Stabilizer::stabilize(bool throttleInZero){

	unsigned long time = (millis() - lastComputeTime); 
	lastComputeTime = millis();
	float dt = time/1000.0f;

	outThrottle = pidThrottle.getSetPoint();
	outYaw = pidYaw.compute(dt);
	outPitch = pidPitch.compute(dt);
	outRoll = pidRoll.compute(dt);

	if(throttleInZero){
		outYaw = outPitch = outRoll = 0;
		pidYaw.reset();
		pidPitch.reset();
		pidRoll.reset();
	}

	uint32_t m1 = outThrottle - outPitch + outRoll - outYaw;
	uint32_t m2 = outThrottle - outPitch - outRoll + outYaw;
	uint32_t m3 = outThrottle + outPitch - outRoll - outYaw;
	uint32_t m4 = outThrottle + outPitch + outRoll + outYaw;

	uint32_t powers[] = {
		m1, m2, m3, m4
	};

	Motors::setPower(powers);

}