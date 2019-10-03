#pragma once

#include <Arduino.h>

#include <Wire.h>
#include <MPU9250.h>
#include <SFE_BMP180.h>

#include "Kalman.h"

#include "_config.h"
#include "_utils.h"


class AHRS{

private:

	static MPU9250 imu;
	// static SFE_BMP180 *bmp;

	// === IMU Variables
	static Kalman kalmanPitch, kalmanRoll;

	static void AccelCalibration();
	static float accPitchBias, accRollBias;

	static void processKalmanFilter();
	static void processComplementaryFilter();

	static unsigned long lastComputeTime;



	// === BMP Variables
	// static double initialPressure;

	// static double getPressure();

public:

	// Initializes communication and performs sensor configuration
	// Returns 'true' if the connection was successful
	static bool begin();

	// Performs reading of the sensors
	// Returns 'true' if the reading was successful
	static bool readData();

	// Returns angles and altitude of last reading performed
	static int32_t getYaw();
	static int32_t getPitch();
	static int32_t getRoll();
	// static double getAltitude();

	// Print angles
	static void printAngles();
};