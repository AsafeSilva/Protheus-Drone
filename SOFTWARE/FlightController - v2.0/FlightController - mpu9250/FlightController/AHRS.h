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

	static Kalman kalmanPitch, kalmanRoll;
	static void processKalmanFilter();
	static void processComplementaryFilter();

	static void calibrate();
	static float pitchBias, rollBias;

	static void AccelCalibration();
	static float accPitchBias, accRollBias;

	static float rawGyroPitch, rawGyroRoll;
	static float gyroPitch, gyroRoll;
	static float accelPitch, accelRoll;

	static float pitchAngle, rollAngle;

	static float outKalmanPitch, outKalmanRoll;
	static float outCompPitch, outCompRoll;

	static unsigned long lastComputeTime;

public:

	// Initializes communication and performs sensor configuration
	// Returns 'true' if the connection was successful
	static bool begin();

	// Performs reading of the sensors
	// Returns 'true' if the reading was successful
	static bool readData();

	// Returns angles and altitude of last reading performed
	static float getYaw();
	static float getGyroPitch();
	static float getPitch();
	static float getGyroRoll();
	static float getRoll();
	// static double getAltitude();

	// Returns raw values (Accelerometer and Gyroscope)
	static float getAx();
	static float getAy();
	static float getAz();
	static float getGx();
	static float getGy();
	static float getGz();

	// Print angles
	static void debug();
};