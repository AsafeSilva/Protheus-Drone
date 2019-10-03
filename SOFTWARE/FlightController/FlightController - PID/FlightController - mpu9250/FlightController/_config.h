#pragma once

#include <Arduino.h>


// 
// Project
// 
#define PROJECT_NAME	F("\nProtheus Drone v2.0\n")


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
// LOGIC LEVEL CONVERTER
// 
#define PIN_LEVEL_CONVERTER	A4

// 
// LED FOR DEBUG
// 
#define PIN_LED_DEBUG 13

// 
// I2C COMMUNICATION
// 
#define I2C_FREQUENCY	400000

// 
// RADIO CONTROL RANGES
// 
#define YAW_MIN			1000
#define YAW_MAX			2000
#define PITCH_MIN		2000
#define PITCH_MAX		1000
#define ROLL_MIN		1000
#define ROLL_MAX		2000