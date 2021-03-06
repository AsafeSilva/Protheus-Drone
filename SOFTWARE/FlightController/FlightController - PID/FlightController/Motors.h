#pragma once

#include <Arduino.h>

#include "_config.h"
#include "_utils.h"

// 
// PWM PROPERTIES
// 
// OBS: 1bit of Duty cycle = 4,88us
#define MIN_DUTY_CYCLE	204			// ~1000 us of duty cycle - 204
#define MAX_DUTY_CYCLE	409			// ~2000 us of duty cycle - 409
#define MIN_DUTY_2FLY	MIN_DUTY_CYCLE 
#define RESOLUTION	12
#define FREQUENCY	50

#define MAX_DUTY_CYCLE_PWM 4095

class Motors{

private:

	static uint32_t powers[MOTORS_COUNT];

public:

	// Initializes PWM channels
	static void begin();

	// Sets DutyCycle of the PWM channels	RANGE: [MIN_DUTY_CYCLE ~ MAX_DUTY_CYCLE]
	static void setPower(uint32_t powers[MOTORS_COUNT]);

	// Get the percentage of motors speed
	static uint32_t* getPercentPower();

	// Stop all motors
	static void stop();
};