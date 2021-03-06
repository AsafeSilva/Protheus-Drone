#pragma once


#include "_config.h"

#include "Motors.h"
#include "PID.h"

class Stabilizer{

private:

	static unsigned long lastComputeTime;

	static float outThrottle, outYaw, outPitch, outRoll;

public:
	static PID pidThrottle, pidYaw, pidPitch, pidRoll;

	static void begin();

	static void throttleUpdateSetPoint(float setPoint);
	static void throttleUpdateInput(float input);

	static void yawUpdateSetPoint(float setPoint);
	static void yawUpdateInput(float input);

	static void pitchUpdateSetPoint(float setPoint);
	static void pitchUpdateInput(float input);

	static void rollUpdateSetPoint(float setPoint);
	static void rollUpdateInput(float input);

	static void reset();

	static void stabilize(bool throttleInZero);
};