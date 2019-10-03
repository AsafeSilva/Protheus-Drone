#pragma once

#include <Arduino.h>

#include "_config.h"
#include "_utils.h"


#define DISARMED 0
#define ARMED 1
#define WAIT_ACTIVATION 2
#define ESC_CALIBRATION 3
#define ERROR_MPU_CONNECTION 4
#define ERROR_MPU_GYRO 5


class System{

private:

	static bool ledErrorState;
	static bool ledDebugState;
	static bool ledRightState;
	static bool ledLeftState;
	static unsigned long lastTimeBlink;
	static unsigned long blinkInterval;

public:

	static int DroneState;

	static void ledDebug();

	static void waitActivation(volatile uint32_t *roll, volatile uint32_t *pitch, volatile uint32_t *throttle, volatile uint32_t *yaw);

	static void manageDroneState(uint32_t roll, uint32_t pitch, uint32_t throttle, uint32_t yaw);
	
};