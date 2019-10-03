#pragma once

#include <Arduino.h>

// 
// Project
// 
#define PROJECT_NAME	F("\n**** Protheus Drone vPID ****\n")


//
//	SERIAL COMMUNICATION
//
#define SERIAL_SPEED	115200
#define NEW_LINE		"\n"
#define TAB 			"\t"
#define LOG 			Serial.print
#define LOGln 			Serial.println
#define SerialRead		Serial.read
#define Serial_begin 	Serial.begin


// 
// INTERFACE BLUETOOTH COMMUNICATION
// 
#define BT_SPEED	38400
#define BT_LOG 		Serial3.print
#define BT_Read		Serial3.read
#define BT_begin 	Serial3.begin
#define BT_Event	void serialEvent3


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
#define PIN_LED_ERROR A2
#define PIN_LED_DEBUG A1
#define PIN_LED_RIGHT 10
#define PIN_LED_LEFT 12

// 
// I2C COMMUNICATION
// 
#define I2C_FREQUENCY	400000

// 
// IMU CONFIGS
// 
#define ACCEL_PITCH_BIAS -1.09
#define ACCEL_ROLL_BIAS -0.55

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
#define ROLL_MIN 1314
#define ROLL_MAX 1730
#define PITCH_MIN 1304
#define PITCH_MAX 1730
#define THROTTLE_MIN 1070
#define THROTTLE_MAX 1730
#define YAW_MIN 1316
#define YAW_MAX 1732
#define SWITCH_MIN 976
#define SWITCH_MAX 1958

// 
// PID Parameters 
// 
#define PITCH_KP	0.080f
#define PITCH_KI	0.001f
#define PITCH_KD	0.020f
#define ROLL_KP	0.080f
#define ROLL_KI	0.001f
#define ROLL_KD	0.020f
#define YAW_KP	0.150f
#define YAW_KI	0.012f
#define YAW_KD	0.020f