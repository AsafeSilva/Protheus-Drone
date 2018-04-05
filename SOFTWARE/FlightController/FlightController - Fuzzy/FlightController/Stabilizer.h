#pragma once


#include "_config.h"

#include "AHRS.h"
#include "Motors.h"
#include "PID.h"

#include <Fuzzy.h>
#include <FuzzyComposition.h>
#include <FuzzyInput.h>
#include <FuzzyIO.h>
#include <FuzzyOutput.h>
#include <FuzzyRule.h>
#include <FuzzyRuleAntecedent.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzySet.h>


struct FuzzyPID{
	float kP, kI, kD;
	float P, I, D;
	float input, setPoint;
	float outMin, outMax;
	float lastError;
};


class Stabilizer{

private:

	static void fuzzyPitchInit();
	static void fuzzyRollInit();

	static float computesFuzzyPID(Fuzzy* fuzzy, FuzzyPID* fuzzyPID, float dTime);

	static Fuzzy* fuzzyPitch;
	static Fuzzy* fuzzyRoll;

	static unsigned long lastComputeTime;

public:

	static FuzzyPID pitch, roll;
	static PID throttle, yaw;

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

	static void stabilize();
};