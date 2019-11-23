#include "System.h"

bool System::ledErrorState = false;
bool System::ledDebugState = false;
bool System::ledRightState = false;
bool System::ledLeftState = false;
unsigned long System::lastTimeBlink = 0;
unsigned long System::blinkInterval = 0;

int System::DroneState = DISARMED;

void System::ledDebug(){

	if(millis() - lastTimeBlink > blinkInterval){

		ledDebugState = !ledDebugState;
		ledErrorState = !ledErrorState;
		
		if(DroneState == ARMED){
			blinkInterval = ledDebugState ? 100 : 1000;

			digitalWrite(PIN_LED_DEBUG, ledDebugState);
			digitalWrite(PIN_LED_RIGHT, !ledDebugState);
			digitalWrite(PIN_LED_LEFT, !ledDebugState);

		}else if(DroneState == DISARMED){
			blinkInterval = 500;

			digitalWrite(PIN_LED_DEBUG, ledDebugState);
			digitalWrite(PIN_LED_RIGHT, ledDebugState);
			digitalWrite(PIN_LED_LEFT, !ledDebugState);

		}else if(DroneState == WAIT_ACTIVATION){
			blinkInterval = ledDebugState ? 900 : 100;

			digitalWrite(PIN_LED_DEBUG, ledDebugState);
			digitalWrite(PIN_LED_RIGHT, 0);
			digitalWrite(PIN_LED_LEFT, 0);

		}else if(DroneState == ESC_CALIBRATION){
			blinkInterval = 0;

			digitalWrite(PIN_LED_DEBUG, ledDebugState);
			digitalWrite(PIN_LED_RIGHT, 0);
			digitalWrite(PIN_LED_LEFT, 0);

		}else if (DroneState == ERROR_MPU_CONNECTION){
			blinkInterval = 100;

			digitalWrite(PIN_LED_ERROR, ledErrorState);
			digitalWrite(PIN_LED_DEBUG, 0);
			digitalWrite(PIN_LED_RIGHT, 0);
			digitalWrite(PIN_LED_LEFT, 0);

		}else if(DroneState == ERROR_MPU_GYRO){
			blinkInterval = 500;

			digitalWrite(PIN_LED_ERROR, ledErrorState);
			digitalWrite(PIN_LED_DEBUG, 0);
			digitalWrite(PIN_LED_RIGHT, 0);
			digitalWrite(PIN_LED_LEFT, 0);
		}

		lastTimeBlink = millis();
	}
}

void System::waitActivation(volatile uint32_t *roll, volatile uint32_t *pitch, volatile uint32_t *throttle, volatile uint32_t *yaw){

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

void System::manageDroneState(uint32_t roll, uint32_t pitch, uint32_t throttle, uint32_t yaw){
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