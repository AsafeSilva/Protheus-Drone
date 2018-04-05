/*************************************************************

 --- Protheus Drone ---

> Link do projeto no GitHub:
github.com/AsafeSilva/...

> Autores:

.Asafe dos Santos Silva
.Daniel Queiroz Moraes Resende
.Élton Franklin de Lima

> Data de Criação:	18/08/2017

> Última modificação:	05/03/2018

> Descrição do projeto:

	<> Hardware
		- Arduino DUE [ARM Cortex-M3]
		- MPU9250 [Acelerômetro + Giroscópio + Magnetrômetro]
		- BMP 180 [Barômetro]
		- Rádio [Receptor]
		- Bluetooth
		- Conversor de Nível [3V3 <-> 5V] !!! NECESSÁRIO POIS O ARDUINO DUE TRABALHA COM 3V3 !!!
		- Electronic Speed Control (ESC)
	<> Software
		- 1° Passo		"Leitura do rádio controle"
		- 2° Passo		"Leitura do AHRS (Attitude and Heading Reference System)"
		- 3° Passo		"Cálculo do Fuzzy (Yaw - Pitch - Roll)"
		- 4° Passo		"Atualizar velocidades dos motores"
		- 5° Passo		"Comunicação com o sintonizador"

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
#include <Fuzzy.h>

// -- Modules
#include "InterfaceComm.h"
#include "RadioControl.h"
#include "Motors.h"
#include "Kalman.h"
#include "AHRS.h"
#include "Stabilizer.h"


void setup() {

	// 
	// Initialize Logic level converter
	// 
	pinMode(PIN_LEVEL_CONVERTER, OUTPUT);
	digitalWrite(PIN_LEVEL_CONVERTER, 0);
	delay(400);
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
	RadioControl_begin();

	// 
	// Setup AHRS
	// 
	waitActivation(&ThrottleChannel.timer, ThrottleChannel.MIN+1000, &YawChannel.timer, YawChannel.MAX-1000);
	if(!AHRS::begin())	while(true);

	// 
	// Setup Stabilizer
	// 
	Stabilizer::begin();

	LOGln();
	LOGln("ARM:             --- Move the left stick down and to the right...");
	LOGln("DISARM:          --- Move the left stick down and to the left...");
	LOGln("ESC_CALIBRATION: --- Move the left stick up and to the right...");
	LOGln();
	LOGln("\n------------ Let's fly! ------------\n");

}


void loop() {

	droneChangeState(ThrottleChannel.timer, ThrottleChannel.MIN, ThrottleChannel.MAX, YawChannel.timer, YawChannel.MIN, YawChannel.MAX);

	// If drone DISARMED, Stop motors
	if(DroneState == DISARMED){
		Stabilizer::reset();
		Motors::stop();
	}else if(DroneState == ESC_CALIBRATION){
		float throttle = mapFloat(ThrottleChannel.timer, ThrottleChannel.MIN, ThrottleChannel.MAX, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
		uint32_t powers[] = {throttle, throttle, throttle, throttle};
		Motors::setPower(powers);

		return;
	}

	// If the data is ready...
	if(AHRS::readData()){

		// Read Radio Control
		float throttleSetPoint = mapFloat(ThrottleChannel.timer, ThrottleChannel.MIN, ThrottleChannel.MAX, MIN_DUTY_2FLY, MIN_DUTY_2FLY + (MAX_DUTY_CYCLE-MIN_DUTY_2FLY)*0.6f);
		float yawControl = mapFloat(YawChannel.timer, YawChannel.MIN, YawChannel.MAX, YAW_MIN, YAW_MAX);
		float pitchControl = mapFloat(PitchChannel.timer, PitchChannel.MIN, PitchChannel.MAX, PITCH_MIN, PITCH_MAX);
		float rollControl = mapFloat(RollChannel.timer, RollChannel.MIN, RollChannel.MAX, ROLL_MIN, ROLL_MAX);

		pitchControl -= PitchChannel.offset;
		rollControl -= RollChannel.offset;

		pitchControl = constrain(pitchControl, 1000, 2000);
		rollControl = constrain(rollControl, 1000, 2000);

		// Dead band in pitch and roll (+/- 8)
		float yawSetPoint = 0, pitchSetPoint = 0, rollSetPoint = 0;

		if(ThrottleChannel.timer > (ThrottleChannel.MIN + (ThrottleChannel.MAX - ThrottleChannel.MIN)*0.2)){
			if(yawControl < 1492) yawSetPoint = yawControl - 1492;
			else if(yawControl > 1508) yawSetPoint = yawControl - 1508;
		}

		if(pitchControl < 1492) pitchSetPoint = pitchControl - 1492;
		else if(pitchControl > 1508) pitchSetPoint = pitchControl - 1508;

		if(rollControl < 1492) rollSetPoint = rollControl - 1492;
		else if(rollControl > 1508) rollSetPoint = rollControl - 1508;

		// Calculate SETPOINT
		pitchSetPoint = (pitchSetPoint - AHRS::getPitch() * 49.2f) / 3.0f; 
		rollSetPoint = (rollSetPoint - AHRS::getRoll() * 49.2f) / 3.0f; 
		yawSetPoint /= 3.0f;


		// === LOAD DATA TO INTERFACE
		DataToSend[SendID::YAW_IN] = AHRS::getGyroYaw();
		DataToSend[SendID::YAW_SET] = yawSetPoint;
		DataToSend[SendID::PITCH_IN] = AHRS::getGyroPitch();
		DataToSend[SendID::PITCH_SET] = pitchSetPoint;
		DataToSend[SendID::ROLL_IN] = AHRS::getGyroRoll();
		DataToSend[SendID::ROLL_SET] = rollSetPoint;


		// If drone is armed AND throttle is greater than 2%...
		if((DroneState == ARMED) && 
			(ThrottleChannel.timer > ThrottleChannel.MIN + (ThrottleChannel.MAX - ThrottleChannel.MIN)*0.02)){
		
			// === CALCULATE PID
			Stabilizer::throttleUpdateSetPoint(throttleSetPoint);

			Stabilizer::yawUpdateSetPoint(yawSetPoint);
			Stabilizer::yawUpdateInput(AHRS::getGyroYaw());
	
			Stabilizer::pitchUpdateSetPoint(pitchSetPoint);
			Stabilizer::pitchUpdateInput(AHRS::getGyroPitch());
	
			Stabilizer::rollUpdateSetPoint(rollSetPoint);
			Stabilizer::rollUpdateInput(AHRS::getGyroRoll());
	
			Stabilizer::stabilize();

			// === LOAD DATA TO INTERFACE
			DataToSend[SendID::M1_VEL] = *(Motors::getPercentPower());
			DataToSend[SendID::M2_VEL] = *(Motors::getPercentPower()+1);
			DataToSend[SendID::M3_VEL] = *(Motors::getPercentPower()+2);
			DataToSend[SendID::M4_VEL] = *(Motors::getPercentPower()+3);
		}else{
			Stabilizer::reset();
			Motors::stop();
		}


		// === SEND DATA TO INTERFACE
		SendDataToInterface();
	}
}


// === RECEIVE INTERFACE DATA
BT_Event(){

	if(ReceiveInterfaceData()){

		if(ID == ReceiveID::KP_YAW)				Stabilizer::yaw.setKP(DataReceived[ReceiveID::KP_YAW]);
		else if(ID == ReceiveID::KI_YAW)		Stabilizer::yaw.setKI(DataReceived[ReceiveID::KI_YAW]);
		else if(ID == ReceiveID::KD_YAW)		Stabilizer::yaw.setKD(DataReceived[ReceiveID::KD_YAW]);
		else if(ID == ReceiveID::KP_PITCH)		Stabilizer::pitch.kP = DataReceived[ReceiveID::KP_PITCH];
		else if(ID == ReceiveID::KI_PITCH)		Stabilizer::pitch.kI = DataReceived[ReceiveID::KI_PITCH];
		else if(ID == ReceiveID::KD_PITCH)		Stabilizer::pitch.kD = DataReceived[ReceiveID::KD_PITCH];
		else if(ID == ReceiveID::KP_ROLL)		Stabilizer::roll.kP = DataReceived[ReceiveID::KP_ROLL];
		else if(ID == ReceiveID::KI_ROLL)		Stabilizer::roll.kI = DataReceived[ReceiveID::KI_ROLL];
		else if(ID == ReceiveID::KD_ROLL)		Stabilizer::roll.kD = DataReceived[ReceiveID::KD_ROLL];
		
	}

}