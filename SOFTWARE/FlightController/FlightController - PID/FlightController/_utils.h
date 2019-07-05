#pragma once

#include <Arduino.h>

#include "_config.h"

// ***************************************************************************************
enum STATE{
	DISARMED,
	ARMED,
	WAIT_ACTIVATION,
	ESC_CALIBRATION
};

static STATE DroneState = DISARMED;
// =======================================================================================


// ***************************************************************************************
static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
// =======================================================================================

// ***************************************************************************************
static bool ledState = false;
static unsigned long lastTime;
static unsigned long interval;
static void ledDebug(){

	if(millis() - lastTime > interval){

		ledState = !ledState;
		digitalWrite(PIN_LED_DEBUG2, ledState);

		if(DroneState == ARMED)
			interval = ledState ? 100 : 1000;
		if(DroneState == DISARMED)
			interval = 500;
		if(DroneState == WAIT_ACTIVATION)
			interval = ledState ? 900 : 100;
		if(DroneState == ESC_CALIBRATION)
			interval = 0;

		lastTime = millis();
	}
}
// =======================================================================================

// ***************************************************************************************
static void waitActivation(volatile uint32_t *throttle, volatile uint32_t *yaw){

	delay(500);
	
	LOGln("Move the left stick down and to the right...");

	DroneState = WAIT_ACTIVATION;
	while(true){
		ledDebug();

		if((*throttle < 1100) && (*yaw < 1100))
			break;
	}
	DroneState = DISARMED;
}
// =======================================================================================

// ***************************************************************************************
static void droneChangeState(uint32_t roll, uint32_t pitch, uint32_t throttle, uint32_t yaw){
	if(DroneState == DISARMED){

		if((throttle < 1100) && (yaw < 1100) && (roll < 1100) && (pitch > 1900)){
			LOGln(DroneState != ARMED ? "STATE: ARMED!" : "");
			DroneState = ARMED;
		}else if((throttle > 1900) && (yaw < 1100)){
			LOGln(DroneState != ESC_CALIBRATION ? "STATE: ESC CALIBRATION" : "");
			DroneState = ESC_CALIBRATION;
		}

	}else if(DroneState != DISARMED){

		if((throttle < 1100) && (yaw > 1900)){
			LOGln(DroneState != DISARMED ? "STATE: DISARMED!" : "");
			DroneState = DISARMED;
		}
	}

	ledDebug();
}
// =======================================================================================