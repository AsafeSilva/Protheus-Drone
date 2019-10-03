#include "AHRS.h"


MPU9250 AHRS::imu;
// SFE_BMP180 AHRS::bmp(&Wire1);

Kalman AHRS::kalmanPitch;
Kalman AHRS::kalmanRoll;

float AHRS::accPitchBias = 0;
float AHRS::accRollBias = 0;

unsigned long AHRS::lastComputeTime;

// double AHRS::initialPressure = 0;


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
	// imu.MPU9250SelfTest(imu.SelfTest);

	// Calibrate gyro and accelerometers, load biases in bias registers
	imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);

	// Initialize device for active mode
	imu.initMPU9250();

	// // Read the WHO_AM_I register of the magnetometer
	// // The default value of the register is 0x48.
	// addrs = imu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

	// if (addrs != 0x48){
	// 	LOGln("Could not connect to MPU9250 (Magnetomer) :(");
	// 	return false;
	// }

	// // Get magnetometer calibration from AK8963 ROM
 //    // Initialize device for active mode read of magnetometer
 //    imu.initAK8963(imu.magCalibration);

	// Calibration for Acceleromer offset
	AHRS::AccelCalibration();
	
	// Initialize Kalman filter with accelerometer values 
	float accelPitch = atan2(imu.ay, sqrt(imu.ax*imu.ax + imu.az*imu.az)) * RAD_TO_DEG;
	float accelRoll = -atan2(imu.ax, sqrt(imu.ay*imu.ay + imu.az*imu.az)) * RAD_TO_DEG;

	accelPitch -= accPitchBias;
	accelRoll -= accRollBias;

	// Set starting angle
	kalmanPitch.setAngle(accelPitch);
	kalmanRoll.setAngle(accelRoll);


	// == Initialize Barometer
	// initialPressure = getPressure();


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

	// === READ MAGNETOMETER === //
	// imu.readMagData(imu.magCount);  // Read the x/y/z adc values
	// imu.getMres();
	// User environmental x-axis correction in milliGauss, should be
	// automatically calculated
	// imu.magbias[0] = +470.;
	// // User environmental x-axis correction in milliGauss TODO axis??
	// imu.magbias[1] = +120.;
	// // User environmental x-axis correction in milliGauss
	// imu.magbias[2] = +125.;

	// Calculate the magnetometer values in milliGauss
	// Include factory calibration per data sheet and user environmental
	// corrections
	// Get actual magnetometer value, this depends on scale being set
	// imu.mx = (float)imu.magCount[0]*imu.mRes*imu.magCalibration[0] -
	//            imu.magbias[0];
	// imu.my = (float)imu.magCount[1]*imu.mRes*imu.magCalibration[1] -
	//            imu.magbias[1];
	// imu.mz = (float)imu.magCount[2]*imu.mRes*imu.magCalibration[2] -
	//            imu.magbias[2];
	/*---------------------------- end READ RAW DATA ----------------------------*/

	AHRS::processKalmanFilter();

	return true;
}

int32_t AHRS::getYaw(){	
	return imu.yaw;	
}

int32_t AHRS::getPitch(){	
	return imu.pitch;	
}

int32_t AHRS::getRoll(){	
	return imu.roll;	
}

void AHRS::printAngles(){
	// LOG(imu.yaw);	LOG(TAB);
	LOG(imu.pitch);	LOG(TAB);
	LOG(imu.roll);	LOG(NEW_LINE);
}

void AHRS::processKalmanFilter(){
	// === CALCULATE ANGLES WITH ACCELETOMETER VALUES === //
  	float accelPitch = atan2(imu.ay, sqrt(imu.ax*imu.ax + imu.az*imu.az)) * RAD_TO_DEG;
  	float accelRoll = -atan2(imu.ax, sqrt(imu.ay*imu.ay + imu.az*imu.az)) * RAD_TO_DEG;

  	accelPitch -= accPitchBias;
  	accelRoll -= accRollBias;

  	unsigned long time = (millis() - lastComputeTime); 
	lastComputeTime = millis();
	float dt = time/1000.0f;

	imu.pitch = kalmanPitch.getAngle(accelPitch, imu.gx, dt);
	imu.roll = kalmanRoll.getAngle(accelRoll, imu.gy, dt);
}


void AHRS::AccelCalibration(){

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
	  	float accelPitch = atan2(imu.ay, sqrt(imu.ax*imu.ax + imu.az*imu.az)) * RAD_TO_DEG;
  		float accelRoll = -atan2(imu.ax, sqrt(imu.ay*imu.ay + imu.az*imu.az)) * RAD_TO_DEG;

	  	accPitchBias += accelPitch; 
	  	accRollBias += accelRoll;
	}

	accPitchBias /= 500;
	accRollBias /= 500;
}


// Code of Sparkfun
// double AHRS::getPressure(){
//   char status;
//   double T,P,p0,a;

//   // You must first get a temperature measurement to perform a pressure reading.
  
//   // Start a temperature measurement:
//   // If request is successful, the number of ms to wait is returned.
//   // If request is unsuccessful, 0 is returned.

//   status = pressure.startTemperature();
//   if (status != 0)
//   {
//     // Wait for the measurement to complete:

//     delay(status);

//     // Retrieve the completed temperature measurement:
//     // Note that the measurement is stored in the variable T.
//     // Use '&T' to provide the address of T to the function.
//     // Function returns 1 if successful, 0 if failure.

//     status = pressure.getTemperature(T);
//     if (status != 0)
//     {
//       // Start a pressure measurement:
//       // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
//       // If request is successful, the number of ms to wait is returned.
//       // If request is unsuccessful, 0 is returned.

//       status = pressure.startPressure(3);
//       if (status != 0)
//       {
//         // Wait for the measurement to complete:
//         delay(status);

//         // Retrieve the completed pressure measurement:
//         // Note that the measurement is stored in the variable P.
//         // Use '&P' to provide the address of P.
//         // Note also that the function requires the previous temperature measurement (T).
//         // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
//         // Function returns 1 if successful, 0 if failure.

//         status = pressure.getPressure(P,T);
//         if (status != 0)
//         {
//           return(P);
//         }
//         else LOGln("error retrieving pressure measurement\n");
//       }
//       else LOGln("error starting pressure measurement\n");
//     }
//     else LOGln("error retrieving temperature measurement\n");
//   }
//   else LOGln("error starting temperature measurement\n");
// }