#pragma once

#include <Arduino.h>

#include "_config.h"

// ***************************************************************************************
enum STATE{
	DISARMED,
	ARMED,
	WAIT_ACTIVATION,
	ESC_CALIBRATION,
	ERROR_MPU_CONNECTION,
	ERROR_MPU_GYRO
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

			digitalWrite(PIN_LED_DEBUG, ledDebugState2);
			digitalWrite(PIN_LED_RIGHT, !ledDebugState2);
			digitalWrite(PIN_LED_LEFT, !ledDebugState2);

		}else if(DroneState == DISARMED){
			intervalDebug = 500;

			digitalWrite(PIN_LED_DEBUG, ledDebugState2);
			digitalWrite(PIN_LED_RIGHT, ledDebugState2);
			digitalWrite(PIN_LED_LEFT, !ledDebugState2);

		}else if(DroneState == WAIT_ACTIVATION){
			intervalDebug = ledDebugState2 ? 900 : 100;

			digitalWrite(PIN_LED_DEBUG, ledDebugState2);
			digitalWrite(PIN_LED_RIGHT, 0);
			digitalWrite(PIN_LED_LEFT, 0);

		}else if(DroneState == ESC_CALIBRATION){
			intervalDebug = 0;

			digitalWrite(PIN_LED_DEBUG, ledDebugState2);
			digitalWrite(PIN_LED_RIGHT, 0);
			digitalWrite(PIN_LED_LEFT, 0);
		}else if (DroneState == ERROR_MPU_CONNECTION){
			intervalDebug = 100;

			digitalWrite(PIN_LED_ERROR, ledDebugState1);
			digitalWrite(PIN_LED_DEBUG, 0);
			digitalWrite(PIN_LED_RIGHT, 0);
			digitalWrite(PIN_LED_LEFT, 0);
		}else if(DroneState == ERROR_MPU_GYRO){
			intervalDebug = 500;

			digitalWrite(PIN_LED_ERROR, ledDebugState1);
			digitalWrite(PIN_LED_DEBUG, 0);
			digitalWrite(PIN_LED_RIGHT, 0);
			digitalWrite(PIN_LED_LEFT, 0);
		}

		lastTimeDebug = millis();
	}
}
// =======================================================================================

// ***************************************************************************************
static void waitActivation(volatile uint32_t *roll, volatile uint32_t *pitch, volatile uint32_t *throttle, volatile uint32_t *yaw){

	delay(1000);
	
	LOGln("[Left Stick: Up & Left] + [Right Stick: Up & Right] to continue...");

	DroneState = WAIT_ACTIVATION;
	while(true){
		ledDebug();

		if((*roll == 0) || (*pitch == 0) || (*throttle == 0) || (*yaw == 0))
			continue;

		if((*roll > 1900) && (*pitch < 1100) && (*throttle > 1900) && (*yaw > 1900))
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

		if((roll > 1900) && (pitch > 1900) && (throttle < 1100) && (yaw > 1900)){
			LOGln(DroneState != DISARMED ? "STATE: DISARMED!" : "");
			DroneState = DISARMED;
		}
	}

	ledDebug();
}
// =======================================================================================