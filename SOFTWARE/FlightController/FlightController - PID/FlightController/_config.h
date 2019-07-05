#pragma once

#include <Arduino.h>

// 
// Project
// 
#define PROJECT_NAME	F("\n**** Protheus Drone vPID ****\n")


//
//	SERIAL COMMUNICATION
//
#define DEBUG			// un/comment to enable/disable Serial
#define SERIAL_SPEED	38400
#define NEW_LINE		"\n"
#define TAB 			"\t"
#ifdef DEBUG
	#define LOG 			Serial.print
	#define LOGln 			Serial.println
	#define SerialRead		Serial.read
	#define Serial_begin 	Serial.begin
#else
	#define LOG
	#define LOGln
	#define SerialRead
	#define Serial_begin
#endif


// 
// INTERFACE BLUETOOTH COMMUNICATION
// 
#define BT_SPEED	38400
#define BT_LOG 		Serial1.print
#define BT_Read		Serial1.read
#define BT_begin 	Serial1.begin
#define BT_Event	void serialEvent1


// 
// MOTORS
// 
/*	Motors convention for this project
			M1	   M2
			 \	   /
			  \___/
			  /   \
			 /     \
			M4     M3   
*/
#define MOTORS_COUNT	4

#define PIN_MOTOR1	6
#define PIN_MOTOR2	7
#define PIN_MOTOR3	8
#define PIN_MOTOR4	9

const uint8_t MOTOR_PINS[MOTORS_COUNT] = {
	PIN_MOTOR1,
	PIN_MOTOR2,
	PIN_MOTOR3,
	PIN_MOTOR4
};

// 
// LEDs
// 
#define PIN_LED_DEBUG1 A2
#define PIN_LED_DEBUG2 A1
#define PIN_LED_RIGHT 10
#define PIN_LED_LEFT 12

// 
// I2C COMMUNICATION
// 
#define I2C_FREQUENCY	400000

// 
// IMU CONFIGS
// 
#define ACCEL_PITCH_BIAS -3.86
#define ACCEL_ROLL_BIAS -0.14

// 
// RADIO CONTROL CALIBRATION PIN
// 
#define PIN_RADIO_CALIB 13

// 
// RADIO CONTROL CALIBRATION TIME
// 
#define TIME_CALIBRATION  10000  // ms

// 
// RADIO CONTROL CALIBRATION VALUES
// 
#define ROLL_MIN 1296
#define ROLL_MAX 1738
#define PITCH_MIN 1310
#define PITCH_MAX 1738
#define THROTTLE_MIN 1082
#define THROTTLE_MAX 1738
#define YAW_MIN 1298
#define YAW_MAX 1740
#define SWITCH_MIN 982
#define SWITCH_MAX 1966