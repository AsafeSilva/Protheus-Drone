#include "AHRS.h"


MPU6050 AHRS::imu;
Kalman AHRS::kalmanPitch;
Kalman AHRS::kalmanRoll;


int16_t AHRS::ax = 0;
int16_t AHRS::ay = 0;
int16_t AHRS::az = 0;
int16_t AHRS::gx = 0;
int16_t AHRS::gy = 0;
int16_t AHRS::gz = 0;

float AHRS::gyroX = 0;
float AHRS::gyroY = 0;
float AHRS::gyroZ = 0;
float AHRS::accelX = 0;
float AHRS::accelY = 0;
float AHRS::accelZ = 0;

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

	LOG("\nInitializing Reference System (IMU + Barometer)...\n");

	Wire.setClock(I2C_FREQUENCY);
	Wire.begin();


	// Initialize device
	imu.initialize();
	imu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
	imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
	imu.setDLPFMode(0x03);

	if (!imu.testConnection()){
		LOGln("Could not connect to MPU6050 (Gyro and Accel) :(");
		return false;
	}

	AHRS::GyroCalibrate();
	AHRS::AccelCalibrate();
	accelPitchBias = -0.37;
	accelRollBias = -1.72;

	// Set starting angle
	outCompPitch = accelPitch - accelPitchBias;
	outCompRoll = accelRoll - accelRollBias;
	kalmanPitch.setAngle(outCompPitch);
	kalmanRoll.setAngle(outCompRoll);

	lastComputeTime = millis();

	LOGln("MPU6050 was successfully connected :)");

	return true;
}


bool AHRS::readData(){

	if (!imu.getIntDataReadyStatus()){
		LOGln("... data not ready ...");
		return false;
	}

	/*------------------------------ READ RAW DATA ------------------------------*/
	imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	accelX = (float)ax/16384.0f;
	accelY = (float)ay/16384.0f;
	accelZ = (float)az/16384.0f;
	gyroX = (float)(gx - gyroXBias)/65.5f;
	gyroY = (float)(gy - gyroYBias)/65.5f;
	gyroZ = (float)(gz - gyroZBias)/65.5f;
	/*---------------------------- end READ RAW DATA ----------------------------*/

	// Complementary filter in gyroscope
	rawGyroYaw = rawGyroYaw * 0.7f + gyroZ * 0.3f;
	rawGyroPitch = rawGyroPitch * 0.7f + gyroX * 0.3f;
	rawGyroRoll = rawGyroRoll * 0.7f + gyroY * 0.3f;

	// Kalman filter to calculate angles with gyroscope and accelerometer
	// processKalmanFilter();
	processComplementaryFilter();

	return true;
}

void AHRS::processKalmanFilter(){
	// ANGLE WITH ACCELEROMETER
	accelPitch = atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ)) * RAD_TO_DEG;
	accelRoll = -atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ)) * RAD_TO_DEG;

	// Remove offset
	accelPitch -= accelPitchBias;
	accelRoll -= accelRollBias;

  	unsigned long time = (millis() - lastComputeTime); 
	lastComputeTime = millis();
	float dt = time/1000.0f;

	outKalmanPitch = kalmanPitch.getAngle(accelPitch, gyroX, dt);
	outKalmanRoll = kalmanRoll.getAngle(accelRoll, gyroY, dt);

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
	accelPitch = atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ)) * RAD_TO_DEG;
	accelRoll = -atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ)) * RAD_TO_DEG;

	// Remove offset
	accelPitch -= accelPitchBias;
	accelRoll -= accelRollBias;

	// float lastCompPitch = outCompPitch;
	// float lastCompRoll = outCompRoll;

	// Angle with complementary filter (gyroscope + accelerometer)
	outCompPitch = (outCompPitch + gyroX*dt) * 0.994f + accelPitch * 0.006f;
	outCompRoll = (outCompRoll + gyroY*dt) * 0.994f + accelRoll * 0.006f;

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

float AHRS::getAx(){ return accelX; }

float AHRS::getAy(){ return accelY; }

float AHRS::getAz(){ return accelZ; }

float AHRS::getGx(){ return gyroX; }

float AHRS::getGy(){ return gyroY; }

float AHRS::getGz(){ return gyroZ; }

void AHRS::debug(){
	LOG("Y: ");	LOG(getGyroYaw());	LOG("  ");
	LOG("P: ");	LOG(getPitch());	LOG("  ");
	LOG("R: ");	LOG(getRoll());	LOG(NEW_LINE);
}

void AHRS::AccelCalibrate(){

	for (int i = 0; i < 500; i++){

		imu.getAcceleration(&ax, &ay, &az);

		accelX = (float)ax/16384.0f;
		accelY = (float)ay/16384.0f;
		accelZ = (float)az/16384.0f;

		accelPitch = atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ)) * RAD_TO_DEG;
		accelRoll = -atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ)) * RAD_TO_DEG;

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

		imu.getRotation(&gx, &gy, &gz);

		gyroXBias += gx;
		gyroYBias += gy;
		gyroZBias += gz;
	}

	gyroXBias /= 500.0f;
	gyroYBias /= 500.0f;
	gyroZBias /= 500.0f;
}