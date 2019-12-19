/*************************************************************

 --- Protheus Drone ---

> Link do projeto no GitHub:
github.com/AsafeSilva/Protheus-Drone

> Autores:

.Asafe dos Santos Silva
.Daniel Queiroz Moraes Resende
.Élton Franklin de Lima

> Data de Criação:	18/08/2017

> Última modificação:	23/11/2019

> Descrição do projeto:

	<> Hardware
		- Arduino DUE [ARM Cortex-M3]
		- MPU6050 [Acelerômetro + Giroscópio]
		- BMP 180 [Barômetro]
		- Rádio [Receptor]
		- Bluetooth
		- Electronic Speed Control (ESC)
	<> Software
		- 1° Passo		"Leitura do rádio controle"
		- 2° Passo		"Leitura da orientação (Yaw, Pitch e Roll) do Drone"
		- 3° Passo		"Cálculo dos PID's (Yaw - Pitch - Roll)"
		- 4° Passo		"Atualizar velocidades dos motores"
		- 5° Passo		"Comunicação com o sintonizador PID"
	<> Dependences
		- I2CDev [https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev]
		- MPU6050 [github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050]
		- Kalman [github.com/TKJElectronics/KalmanFilter]	

> Informações adicionais:

Projeto desenvolvido em Santiago - Chile, através da parceria do
IFPE Campus Caruaru, com a Universidad de Chile, INACAP.

**************************************************************/

// -- Misc stuffs
#include "_config.h"
#include "_utils.h"

// -- Libraries
#include <Wire.h>
#include <MPU6050.h>
#include <Kalman.h>

// -- Modules
#include "System.h"
#include "InterfaceComm.h"
#include "RadioControl.h"
#include "Motors.h"
#include "IMU.h"
#include "PID.h"
#include "Stabilizer.h"


void setup() {

	// 
	// Initialize leds for debug
	// 
	pinMode(PIN_LED_ERROR, OUTPUT); digitalWrite(PIN_LED_ERROR, 0);
	pinMode(PIN_LED_DEBUG, OUTPUT); digitalWrite(PIN_LED_DEBUG, 0);
	pinMode(PIN_LED_RIGHT, OUTPUT);	digitalWrite(PIN_LED_RIGHT, 0);
	pinMode(PIN_LED_LEFT, OUTPUT); digitalWrite(PIN_LED_LEFT, 0);

	// 
	// Setup Serial Communication
	// 
	Serial_begin(SERIAL_SPEED);
	BT_begin(BT_SPEED);
	LOG(PROJECT_NAME);

	// 
	// Setup Motors
	// 
	Motors::begin();

	// 
	// Setup Radio Control
	// 
	RadioControl::begin();

	// 
	// Setup IMU
	// 
	System::waitActivation(RadioControl::RollChannel.getInterval(), RadioControl::PitchChannel.getInterval(),
					 RadioControl::ThrottleChannel.getInterval(), RadioControl::YawChannel.getInterval());

	if(!IMU::begin()){
		while(true)
			System::ledDebug();
	}

	// 
	// Setup Stabilizer
	// 
	Stabilizer::begin();

	LOGln();
	LOGln(F("ARM:             ---- [Left Stick: Donw & Right] + [Right Stick: Down & Left]"));
	LOGln(F("DISARM:          ---- [Left Stick: Down & Left]"));
	LOGln(F("ESC_CALIBRATION: ---- [Left Stick: Up & Right]"));
	LOGln();
	LOGln(F("\n------------ Let's fly! ------------\n"));

}


void loop() {

	System::manageDroneState(RadioControl::RollChannel.read(), RadioControl::PitchChannel.read(), RadioControl::ThrottleChannel.read(), RadioControl::YawChannel.read());

	// If drone DISARMED, Stop motors and resetPID
	if(System::DroneState == DISARMED){
		Stabilizer::reset();

		Motors::stop();
	}else if(System::DroneState == ESC_CALIBRATION){
		float throttle = mapFloat(RadioControl::ThrottleChannel.read(), MIN_RC_VALUE, MAX_RC_VALUE, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
		uint32_t powers[] = {throttle, throttle, throttle, throttle};
		Motors::setPower(powers);

		LOGln(RadioControl::ThrottleChannel.read());

		return;
	}

	// If the data is ready...
	if(IMU::readData()){

		// Read Radio Control
		float throttleSetPoint = mapFloat(RadioControl::ThrottleChannel.read(), MIN_RC_VALUE, MAX_RC_VALUE, MIN_DUTY_2FLY, MAX_DUTY_CYCLE);
		float yawControl = constrain(RadioControl::YawChannel.read(), MIN_RC_VALUE, MAX_RC_VALUE);
		float pitchControl = constrain(RadioControl::PitchChannel.read(), MIN_RC_VALUE, MAX_RC_VALUE);
		float rollControl = constrain(RadioControl::RollChannel.read(), MIN_RC_VALUE, MAX_RC_VALUE);

		float yawSetPoint = 0, pitchSetPoint = 0, rollSetPoint = 0;

		if(RadioControl::ThrottleChannel.read() > 1100){
			// dead band for better results
			if(yawControl < (MID_RC_VALUE - RC_DEAD_BAND)) yawSetPoint = yawControl - (MID_RC_VALUE - RC_DEAD_BAND);
			else if(yawControl > (MID_RC_VALUE + RC_DEAD_BAND)) yawSetPoint = yawControl - (MID_RC_VALUE + RC_DEAD_BAND);
		}

		// dead band for better results
		if(pitchControl < (MID_RC_VALUE - RC_DEAD_BAND)) pitchSetPoint = pitchControl - (MID_RC_VALUE - RC_DEAD_BAND);
		else if(pitchControl > (MID_RC_VALUE + RC_DEAD_BAND)) pitchSetPoint = pitchControl - (MID_RC_VALUE + RC_DEAD_BAND);

		// dead band for better results
		if(rollControl < (MID_RC_VALUE - RC_DEAD_BAND)) rollSetPoint = rollControl - (MID_RC_VALUE - RC_DEAD_BAND);
		else if(rollControl > (MID_RC_VALUE + RC_DEAD_BAND)) rollSetPoint = rollControl - (MID_RC_VALUE + RC_DEAD_BAND);

		// Calculate SETPOINT (deg/sec)
		pitchSetPoint -= IMU::getPitch() * (((MID_RC_VALUE - MIN_RC_VALUE) - RC_DEAD_BAND)/MAX_ANGLE);
		pitchSetPoint /= ((MID_RC_VALUE - MIN_RC_VALUE) - RC_DEAD_BAND)/MAX_ANGULAR_VELOCITY;

		rollSetPoint -= IMU::getRoll() * (((MID_RC_VALUE - MIN_RC_VALUE) - RC_DEAD_BAND)/MAX_ANGLE);
		rollSetPoint /= ((MID_RC_VALUE - MIN_RC_VALUE) - RC_DEAD_BAND)/MAX_ANGULAR_VELOCITY;

		yawSetPoint /= ((MID_RC_VALUE - MIN_RC_VALUE) - RC_DEAD_BAND)/MAX_ANGULAR_VELOCITY;



		if((System::DroneState == ARMED)){

			// === CALCULATE PID
			Stabilizer::throttleUpdateSetPoint(throttleSetPoint);
			
			Stabilizer::yawUpdateSetPoint(yawSetPoint);
			Stabilizer::yawUpdateInput(IMU::getGyroYaw());

			Stabilizer::pitchUpdateSetPoint(pitchSetPoint);
			Stabilizer::pitchUpdateInput(IMU::getGyroPitch());

			Stabilizer::rollUpdateSetPoint(rollSetPoint);
			Stabilizer::rollUpdateInput(IMU::getGyroRoll());
			
			bool throttleInZero = RadioControl::ThrottleChannel.read() < 1020;
			Stabilizer::stabilize(throttleInZero);

		}


		// === LOAD DATA TO INTERFACE
		DataToSend[SendID::YAW_IN] = IMU::getGyroYaw();
		DataToSend[SendID::YAW_SET] = yawSetPoint;
		DataToSend[SendID::PITCH_IN] = IMU::getPitch();
		DataToSend[SendID::PITCH_SET] = mapFloat(pitchControl, MIN_RC_VALUE, MAX_RC_VALUE, -MAX_ANGLE, MAX_ANGLE);
		DataToSend[SendID::ROLL_IN] = IMU::getRoll();
		DataToSend[SendID::ROLL_SET] = mapFloat(rollControl, MIN_RC_VALUE, MAX_RC_VALUE, -MAX_ANGLE, MAX_ANGLE);
		DataToSend[SendID::M1_VEL] = *(Motors::getPercentPower());
		DataToSend[SendID::M2_VEL] = *(Motors::getPercentPower()+1);
		DataToSend[SendID::M3_VEL] = *(Motors::getPercentPower()+2);
		DataToSend[SendID::M4_VEL] = *(Motors::getPercentPower()+3);


		// === SEND DATA TO INTERFACE
		SendDataToInterface();
	}
}


// === RECEIVE INTERFACE DATA
BT_Event(){

	if(ReceiveInterfaceData()){
			if(ID == ReceiveID::KP_YAW)				Stabilizer::pidYaw.setKP(DataReceived[ReceiveID::KP_YAW]);
			else if(ID == ReceiveID::KI_YAW)		Stabilizer::pidYaw.setKI(DataReceived[ReceiveID::KI_YAW]);
			else if(ID == ReceiveID::KD_YAW)		Stabilizer::pidYaw.setKD(DataReceived[ReceiveID::KD_YAW]);
			else if(ID == ReceiveID::KP_PITCH)		Stabilizer::pidPitch.setKP(DataReceived[ReceiveID::KP_PITCH]);
			else if(ID == ReceiveID::KI_PITCH)		Stabilizer::pidPitch.setKI(DataReceived[ReceiveID::KI_PITCH]);
			else if(ID == ReceiveID::KD_PITCH)		Stabilizer::pidPitch.setKD(DataReceived[ReceiveID::KD_PITCH]);
			else if(ID == ReceiveID::KP_ROLL)		Stabilizer::pidRoll.setKP(DataReceived[ReceiveID::KP_ROLL]);
			else if(ID == ReceiveID::KI_ROLL)		Stabilizer::pidRoll.setKI(DataReceived[ReceiveID::KI_ROLL]);
			else if(ID == ReceiveID::KD_ROLL)		Stabilizer::pidRoll.setKD(DataReceived[ReceiveID::KD_ROLL]);
	}

}
