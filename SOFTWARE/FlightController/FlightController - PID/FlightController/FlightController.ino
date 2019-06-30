/*************************************************************

 --- Protheus Drone ---

> Link do projeto no GitHub:
github.com/AsafeSilva/Protheus-Drone

> Autores:

.Asafe dos Santos Silva
.Daniel Queiroz Moraes Resende
.Élton Franklin de Lima

> Data de Criação:	18/08/2017

> Última modificação:	20/02/2019

> Descrição do projeto:

	<> Hardware
		- Arduino DUE [ARM Cortex-M3]
		- MPU6050 [Acelerômetro + Giroscópio]
		- BMP 180 [Barômetro]
		- Rádio [Receptor]
		- Bluetooth
		- Conversor de Nível [3V3 <-> 5V] !!! NECESSÁRIO POIS O ARDUINO DUE TRABALHA COM 3V3 !!!
		- Electronic Speed Control (ESC)
	<> Software
		- 1° Passo		"Leitura do rádio controle"
		- 2° Passo		"Leitura da orientação (Yaw, Pitch e Roll) do Drone"
		- 3° Passo		"Cálculo dos PID's (Yaw - Pitch - Roll)"
		- 4° Passo		"Atualizar velocidades dos motores"
		- 5° Passo		"Comunicação com o sintonizador PID"
	<> Dependences
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
#include "InterfaceComm.h"
#include "RadioControl.h"
#include "Motors.h"
#include "IMU.h"
#include "PID.h"
#include "Stabilizer.h"


void setup() {

	// 
	// Initialize Logic level converter
	// 
	pinMode(PIN_LEVEL_CONVERTER, OUTPUT);
	digitalWrite(PIN_LEVEL_CONVERTER, 0);
	delay(100);
	digitalWrite(PIN_LEVEL_CONVERTER, 1);

	// 
	// Initialize led for debug
	// 
	pinMode(PIN_LED_DEBUG, OUTPUT);
	digitalWrite(PIN_LED_DEBUG, 0);

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
	waitActivation(RadioControl::ThrottleChannel.getInterval(), RadioControl::YawChannel.getInterval());
	if(!IMU::begin())	while(true);

	// 
	// Setup Stabilizer
	// 
	Stabilizer::begin();

	LOGln();
	LOGln(F("ARM:             --- Left Stick: Donw & Right + Right Stick: Down & Left"));
	LOGln(F("DISARM:          --- Left Stick: Down & Left"));
	LOGln(F("ESC_CALIBRATION: --- Left Stick: Up & Right"));
	LOGln();
	LOGln(F("\n------------ Let's fly! ------------\n"));

}


void loop() {

	droneChangeState(RadioControl::RollChannel.read(), RadioControl::PitchChannel.read(), RadioControl::ThrottleChannel.read(), RadioControl::YawChannel.read());

	// If drone DISARMED, Stop motors and resetPID
	if(DroneState == DISARMED){
		Stabilizer::reset();

		Motors::stop();
	}else if(DroneState == ESC_CALIBRATION){
		float throttle = mapFloat(RadioControl::ThrottleChannel.read(), 1000, 2000, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
		uint32_t powers[] = {throttle, throttle, throttle, throttle};
		Motors::setPower(powers);

		LOGln(RadioControl::ThrottleChannel.read());

		return;
	}


	// If the data is ready...
	if(IMU::readData()){

		// Read Radio Control
		float throttleSetPoint = mapFloat(RadioControl::ThrottleChannel.read(), 1000, 2000, MIN_DUTY_2FLY, MAX_DUTY_CYCLE);
		float yawControl = RadioControl::YawChannel.read();
		float pitchControl = RadioControl::PitchChannel.read();
		float rollControl = RadioControl::RollChannel.read();

		pitchControl -= 0; //RadioControl::PitchChannel.getOffset();
		rollControl -= 0; //RadioControl::RollChannel.getOffset();

		pitchControl = constrain(pitchControl, 1000, 2000);
		rollControl = constrain(rollControl, 1000, 2000);

		// Dead band in pitch and roll (+/- 8)
		float yawSetPoint = 0, pitchSetPoint = 0, rollSetPoint = 0;

		if(RadioControl::ThrottleChannel.read() > 1400){
			if(yawControl < 1492) yawSetPoint = yawControl - 1492;
			else if(yawControl > 1508) yawSetPoint = yawControl - 1508;
		}

		if(pitchControl < 1492) pitchSetPoint = pitchControl - 1492;
		else if(pitchControl > 1508) pitchSetPoint = pitchControl - 1508;

		if(rollControl < 1492) rollSetPoint = rollControl - 1492;
		else if(rollControl > 1508) rollSetPoint = rollControl - 1508;

		// Calculate SETPOINT
		pitchSetPoint = (pitchSetPoint - IMU::getPitch() * 49.2f) / 3.0f; 
		rollSetPoint = (rollSetPoint - IMU::getRoll() * 49.2f) / 3.0f; 
		yawSetPoint /= 3.0f;


		// If drone is armed AND throttle is greater than 2%...
		if((DroneState == ARMED) /*&& (RadioControl::ThrottleChannel.read() > 1020)*/){

			// === CALCULATE PID
			Stabilizer::throttleUpdateSetPoint(throttleSetPoint);

			Stabilizer::yawUpdateSetPoint(yawSetPoint);
			Stabilizer::yawUpdateInput(IMU::getGyroYaw());
	
			Stabilizer::pitchUpdateSetPoint(pitchSetPoint);
			Stabilizer::pitchUpdateInput(IMU::getGyroPitch());
	
			Stabilizer::rollUpdateSetPoint(rollSetPoint);
			Stabilizer::rollUpdateInput(IMU::getGyroRoll());
	
			Stabilizer::stabilize();

		}/*else if(DroneState == ARMED){
			Stabilizer::reset();
			
			uint32_t powers[] = {MIN_DUTY_2FLY, MIN_DUTY_2FLY, MIN_DUTY_2FLY, MIN_DUTY_2FLY};
			Motors::setPower(powers);
		}*/


		// === LOAD DATA TO INTERFACE
		DataToSend[SendID::YAW_IN] = IMU::getGyroYaw();
		DataToSend[SendID::YAW_SET] = yawSetPoint;
		DataToSend[SendID::PITCH_IN] = IMU::getPitch();
		DataToSend[SendID::PITCH_SET] = mapFloat(pitchControl, 1000, 2000, -10, 10);
		DataToSend[SendID::ROLL_IN] = IMU::getRoll();
		DataToSend[SendID::ROLL_SET] = mapFloat(rollControl, 1000, 2000, -10, 10);
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
