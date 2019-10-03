#include "IMU.h"


MPU6050 IMU::imu;
Kalman IMU::kalmanPitch;
Kalman IMU::kalmanRoll;


int16_t IMU::ax = 0;
int16_t IMU::ay = 0;
int16_t IMU::az = 0;
int16_t IMU::gx = 0;
int16_t IMU::gy = 0;
int16_t IMU::gz = 0;

float IMU::gyroX = 0;
float IMU::gyroY = 0;
float IMU::gyroZ = 0;
float IMU::accelX = 0;
float IMU::accelY = 0;
float IMU::accelZ = 0;

float IMU::accelPitchBias = 0;
float IMU::accelRollBias = 0;
float IMU::gyroXBias = 0;
float IMU::gyroYBias = 0;
float IMU::gyroZBias = 0;

float IMU::rawGyroPitch = 0;
float IMU::rawGyroRoll = 0;
float IMU::rawGyroYaw = 0;
float IMU::accelPitch = 0;
float IMU::accelRoll = 0;

float IMU::outKalmanPitch = 0;
float IMU::outKalmanRoll = 0;
float IMU::outCompPitch = 0;
float IMU::outCompRoll = 0;

float IMU::pitchAngle = 0; 
float IMU::rollAngle = 0;

unsigned long IMU::lastComputeTime;


bool IMU::begin(){

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

	IMU::GyroCalibrate();
	IMU::AccelCalibrate();
	accelPitchBias = 0.51;
	accelRollBias = -0.96;

	// Set starting angle
	outCompPitch = accelPitch - accelPitchBias;
	outCompRoll = accelRoll - accelRollBias;
	kalmanPitch.setAngle(outCompPitch);
	kalmanRoll.setAngle(outCompRoll);

	lastComputeTime = millis();

	LOGln("MPU6050 was successfully connected :)");

	return true;
}


bool IMU::readData(){

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

void IMU::processKalmanFilter(){
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

void IMU::processComplementaryFilter(){
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


float IMU::getYaw(){ return 0; }

float IMU::getPitch(){ return pitchAngle; }

float IMU::getRoll(){ return rollAngle; }

float IMU::getGyroYaw(){ return rawGyroYaw; }

float IMU::getGyroPitch(){ return rawGyroPitch; }

float IMU::getGyroRoll(){ return rawGyroRoll; }

float IMU::getAx(){ return accelX; }

float IMU::getAy(){ return accelY; }

float IMU::getAz(){ return accelZ; }

float IMU::getGx(){ return gyroX; }

float IMU::getGy(){ return gyroY; }

float IMU::getGz(){ return gyroZ; }

void IMU::debug(){
	LOG("Y: ");	LOG(getGyroYaw());	LOG("  ");
	LOG("P: ");	LOG(getPitch());	LOG("  ");
	LOG("R: ");	LOG(getRoll());	LOG(NEW_LINE);
}

void IMU::AccelCalibrate(){

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

void IMU::GyroCalibrate(){

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