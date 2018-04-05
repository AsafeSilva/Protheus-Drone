#pragma once

#include <Arduino.h>

#include <Wire.h>
#include <MPU6050.h>
#include <SFE_BMP180.h>

#include "Kalman.h"

#include "_config.h"
#include "_utils.h"


class AHRS{

private:

	static MPU6050 imu;

	static Kalman kalmanPitch, kalmanRoll;
	static void processKalmanFilter();
	static void processComplementaryFilter();

	static int16_t ax, ay, az, gx, gy, gz;
	static float gyroX, gyroY, gyroZ;
	static float accelX, accelY, accelZ;

	static void AccelCalibrate();
	static float accelPitchBias, accelRollBias;
	static void GyroCalibrate();
	static float gyroXBias, gyroYBias, gyroZBias;

	static float rawGyroPitch, rawGyroRoll, rawGyroYaw;
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
	static float getPitch();
	static float getRoll();
	static float getGyroYaw();
	static float getGyroPitch();
	static float getGyroRoll();
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