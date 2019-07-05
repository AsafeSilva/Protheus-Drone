#pragma once

#include <Arduino.h>

#include "_config.h"

// ***************************************************************************************
enum STATE{
	DISARMED,
	ARMED,
	WAIT_ACTIVATION,
	ESC_CALIBRATION,
	ERROR
};

static STATE DroneState = DISARMED;
// =======================================================================================

// ***************************************************************************************
static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
// =======================================================================================

// ***************************************************************************************
static bool ledDebugState1 = false;
static bool ledDebugState2 = false;
static bool ledRightState = false;
static bool ledLeftState = false;
static unsigned long lastTimeDebug;
static unsigned long intervalDebug;

static void ledDebug(){

	if(millis() - lastTimeDebug > intervalDebug){

		ledDebugState2 = !ledDebugState2;
		ledDebugState1 = !ledDebugState1;
		
		if(DroneState == ARMED){
			intervalDebug = ledDebugState2 ? 100 : 1000;

			digitalWrite(PIN_LED_DEBUG2, ledDebugState2);
			digitalWrite(PIN_LED_RIGHT, !ledDebugState2);
			digitalWrite(PIN_LED_LEFT, !ledDebugState2);

		}else if(DroneState == DISARMED){
			intervalDebug = 500;

			digitalWrite(PIN_LED_DEBUG2, ledDebugState2);
			digitalWrite(PIN_LED_RIGHT, ledDebugState2);
			digitalWrite(PIN_LED_LEFT, !ledDebugState2);

		}else if(DroneState == WAIT_ACTIVATION){
			intervalDebug = ledDebugState2 ? 900 : 100;

			digitalWrite(PIN_LED_DEBUG2, ledDebugState2);
			digitalWrite(PIN_LED_RIGHT, 0);
			digitalWrite(PIN_LED_LEFT, 0);

		}else if(DroneState == ESC_CALIBRATION){
			intervalDebug = 0;

			digitalWrite(PIN_LED_DEBUG2, ledDebugState2);
			digitalWrite(PIN_LED_RIGHT, 0);
			digitalWrite(PIN_LED_LEFT, 0);
		}else if (DroneState == ERROR){
			intervalDebug = 200;

			digitalWrite(PIN_LED_DEBUG1, ledDebugState1);
			digitalWrite(PIN_LED_DEBUG2, 0);
			digitalWrite(PIN_LED_RIGHT, 0);
			digitalWrite(PIN_LED_LEFT, 0);
		}

		lastTimeDebug = millis();
	}
}
// =======================================================================================

// ***************************************************************************************
static void waitActivation(volatile uint32_t *throttle, volatile uint32_t *yaw){
	
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