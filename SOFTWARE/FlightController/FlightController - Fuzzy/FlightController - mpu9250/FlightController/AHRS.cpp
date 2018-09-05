#include "AHRS.h"


MPU9250 AHRS::imu;
Kalman AHRS::kalmanPitch;
Kalman AHRS::kalmanRoll;

float AHRS::accelPitchBias = 0;
float AHRS::accelRollBias = 0;
float AHRS::gyroXBias = 0;
float AHRS::gyroYBias = 0;
float AHRS::gyroZBias = 0;

float AHRS::rawGyroPitch = 0; 
float AHRS::rawGyroRoll = 0;
float AHRS::rawGyroYaw = 0;
float AHRS::accelPitch = 0; 
float AHRS::accelRoll = 0;

float AHRS::outKalmanPitch = 0;
float AHRS::outKalmanRoll = 0;
float AHRS::outCompPitch = 0;
float AHRS::outCompRoll = 0;

float AHRS::pitchAngle = 0; 
float AHRS::rollAngle = 0;

unsigned long AHRS::lastComputeTime;


bool AHRS::begin(){

	LOG("\nInitializing Reference System (IMU + Magnetometer + Barometer)...\n");

	Wire.setClock(I2C_FREQUENCY);
	Wire.begin();

	// This register (WHO_AM_I_MPU9250) is used to verify the identity of the device.
	// The default value of the register is 0x71.
	uint8_t addrs = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

	if (addrs != 0x71){
		LOGln("Could not connect to MPU9250 (Gyro and Accel) :(");
		return false;
	}

	// Starts self test
	imu.MPU9250SelfTest(imu.SelfTest);

	// Calibrate gyro and accelerometers, load biases in bias registers
	// imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);

	// Initialize device for active mode
	imu.initMPU9250();

	// Calibration for remove sensor offset
	AHRS::GyroCalibrate();
	AHRS::AccelCalibrate();
	accelPitchBias = 0.02;
	accelRollBias = 7.86;

	// Set starting angle
	outCompPitch = accelPitch - accelPitchBias;
	outCompRoll = accelRoll - accelRollBias;
	kalmanPitch.setAngle(outCompPitch);
	kalmanRoll.setAngle(outCompRoll);

	lastComputeTime = millis();

	LOGln("MPU9250 was successfully connected :)");

	return true;
}


bool AHRS::readData(){

	if (!(imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)){
		LOGln("... data not ready ...");
		return false;
	}

	/*------------------------------ READ RAW DATA ------------------------------*/
	// === READ ACCELEROMETER === //
	imu.readAccelData(imu.accelCount);  // Read the x/y/z adc values
	imu.getAres();

	// Now we'll calculate the accleration value into actual g's
	// This depends on scale being set
	imu.ax = (float)imu.accelCount[0]*imu.aRes;
	imu.ay = (float)imu.accelCount[1]*imu.aRes;
	imu.az = (float)imu.accelCount[2]*imu.aRes;

    // === READ GYROSCOPE === //
	imu.readGyroData(imu.gyroCount);  // Read the x/y/z adc values
	imu.getGres();

	// Calculate the gyro value into actual degrees per second
	// This depends on scale being set
	imu.gx = (float)imu.gyroCount[0]*imu.gRes;
	imu.gy = (float)imu.gyroCount[1]*imu.gRes;
	imu.gz = (float)imu.gyroCount[2]*imu.gRes;
	/*---------------------------- end READ RAW DATA ----------------------------*/

	// Complementary filter in gyroscope
	rawGyroYaw = rawGyroYaw * 0.7f + imu.gz * 0.3f;
	rawGyroPitch = rawGyroPitch * 0.7f + imu.gx * 0.3f;
	rawGyroRoll = rawGyroRoll * 0.7f + imu.gy * 0.3f;


	// Kalman filter to calculate angles with gyroscope and accelerometer
	// processKalmanFilter();
	processComplementaryFilter();

	return true;
}

void AHRS::processKalmanFilter(){
	// === CALCULATE ANGLES WITH ACCELETOMETER VALUES === //
  	accelPitch = atan2(imu.ay, sqrt(imu.ax*imu.ax + imu.az*imu.az)) * RAD_TO_DEG;
  	accelRoll = -atan2(imu.ax, sqrt(imu.ay*imu.ay + imu.az*imu.az)) * RAD_TO_DEG;

  	accelPitch -= accelPitchBias;
  	accelRoll -= accelRollBias;

  	unsigned long time = (millis() - lastComputeTime); 
	lastComputeTime = millis();
	float dt = time/1000.0f;

	outKalmanPitch = kalmanPitch.getAngle(accelPitch, imu.gx, dt);
	outKalmanRoll = kalmanRoll.getAngle(accelRoll, imu.gy, dt);

	// LOW PASS FILTER
	float rc = 1.0f / (2.0f * M_PI * 1.0f);
	float alpha = dt / (dt + rc);
	if(alpha > 1.0f) alpha = 1.0f;
	else if(alpha < 0.0f) alpha = 0.0f;

  	pitchAngle += (outKalmanPitch - pitchAngle) * alpha;
  	rollAngle += (outKalmanRoll - rollAngle) * alpha;
}

void AHRS::processComplementaryFilter(){
	unsigned long time = (millis() - lastComputeTime); 
	lastComputeTime = millis();
	float dt = time/1000.0f;

	// ANGLE WITH ACCELEROMETER
	accelPitch = atan2(imu.ay, sqrt(imu.ax*imu.ax + imu.az*imu.az)) * RAD_TO_DEG;
	accelRoll = -atan2(imu.ax, sqrt(imu.ay*imu.ay + imu.az*imu.az)) * RAD_TO_DEG;

	// Remove offset
	accelPitch -= accelPitchBias;
	accelRoll -= accelRollBias;

	// float lastCompPitch = outCompPitch;
	// float lastCompRoll = outCompRoll;

	// Angle with complementary filter (gyroscope + accelerometer)
	outCompPitch = (outCompPitch + imu.gx*dt) * 0.994f + accelPitch * 0.006f;
	outCompRoll = (outCompRoll + imu.gy*dt) * 0.994f + accelRoll * 0.006f;

	pitchAngle = outCompPitch;
	rollAngle = outCompRoll;

	// HIGH PASS FILTER
	// float rc = 1.0f / (2.0f * M_PI * 0.1f);
	// float alpha = rc / (rc + dt);
	// if(alpha > 1.0f) alpha = 1.0f;
	// else if(alpha < 0.0f) alpha = 0.0f;

	// pitchAngle = alpha * (pitchAngle + outCompPitch - lastCompPitch);
	// rollAngle = alpha * (rollAngle + outCompRoll - lastCompRoll);

}

float AHRS::getYaw(){ return 0; }

float AHRS::getPitch(){ return pitchAngle; }

float AHRS::getRoll(){ return rollAngle; }

float AHRS::getGyroYaw(){ return rawGyroYaw; }

float AHRS::getGyroPitch(){ return rawGyroPitch; }

float AHRS::getGyroRoll(){ return rawGyroRoll; }

float AHRS::getAx(){ return imu.ax; }

float AHRS::getAy(){ return imu.ay; }

float AHRS::getAz(){ return imu.az; }

float AHRS::getGx(){ return imu.gx; }

float AHRS::getGy(){ return imu.gy; }

float AHRS::getGz(){ return imu.gz; }

void AHRS::debug(){
	LOG("Y: ");	LOG(getGyroYaw());	LOG("  ");
	LOG("P: ");	LOG(getPitch());	LOG("  ");
	LOG("R: ");	LOG(getRoll());	LOG(NEW_LINE);
}

void AHRS::AccelCalibrate(){

	for (int i = 0; i < 500; i++){

		while(!(imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01));

		// === READ ACCELEROMETER === //
		imu.readAccelData(imu.accelCount);  // Read the x/y/z adc values
		imu.getAres();

		// Now we'll calculate the accleration value into actual g's
		// This depends on scale being set
		imu.ax = (float)imu.accelCount[0]*imu.aRes;
		imu.ay = (float)imu.accelCount[1]*imu.aRes;
		imu.az = (float)imu.accelCount[2]*imu.aRes;

		// === CALCULATE ANGLES WITH ACCELETOMETER VALUES === //
	  	accelPitch = atan2(imu.ay, sqrt(imu.ax*imu.ax + imu.az*imu.az)) * RAD_TO_DEG;
  		accelRoll = -atan2(imu.ax, sqrt(imu.ay*imu.ay + imu.az*imu.az)) * RAD_TO_DEG;

	  	accelPitchBias += accelPitch; 
	  	accelRollBias += accelRoll;
	}

	accelPitchBias /= 500.0f;
	accelRollBias /= 500.0f;

	LOGln("Accelerometer bias:");
	LOG("accelPitchBias = "); LOG(accelPitchBias); LOG(";"); LOG(NEW_LINE);
	LOG("accelRollBias = "); LOG(accelRollBias); LOG(";"); LOG(NEW_LINE);
}


void AHRS::GyroCalibrate(){

	for (int i = 0; i < 500; i++){

		while(!(imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01));

		// === READ GYROSCOPE === //
		imu.readGyroData(imu.gyroCount);  // Read the x/y/z adc values
		imu.getGres();

		// Calculate the gyro value into actual degrees per second
		// This depends on scale being set
		imu.gx = (float)imu.gyroCount[0]*imu.gRes;
		imu.gy = (float)imu.gyroCount[1]*imu.gRes;
		imu.gz = (float)imu.gyroCount[2]*imu.gRes;

		gyroXBias += imu.gx;
		gyroYBias += imu.gy;
		gyroZBias += imu.gz;
	}

	gyroXBias /= 500.0f;
	gyroYBias /= 500.0f;
	gyroZBias /= 500.0f;
}