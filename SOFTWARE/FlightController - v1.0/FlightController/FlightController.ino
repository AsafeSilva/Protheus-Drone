/*************************************************************

 --- Protheus Drone ---

> Link do projeto no GitHub:
github.com/AsafeSilva/...

> Autores:

.Asafe dos Santos Silva
.Daniel Queiroz Moraes Resende
.Élton Franklin de Lima

> Data de Criação:	18/08/2017

> Última modificação:	28/11/2017

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
		- 3° Passo		"Cálculo dos PID's (Throttle - Yaw - Pitch - Roll)"
		- 4° Passo		"Atualizar velocidades dos motores"
		- 5° Passo		"Comunicação com o sintonizador PID"

> Informações adicionais:

Projeto desenvolvido em Santiago - Chile, através da parceria do
IFPE Campus Caruaru, com a Universidad de Chile, INACAP.

**************************************************************/

// -- Misc stuffs
#include "_config.h"
#include "_utils.h"

// -- Libraries
#include <Wire.h>
#include <MPU9250.h>
#include <SFE_BMP180.h>

// -- Modules
#include "InterfaceComm.h"
#include "RadioControl.h"
#include "Motors.h"
#include "Kalman.h"
#include "AHRS.h"
#include "PID.h"
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

	// 
	// Setup Serial Communication
	// 
	Serial_begin(SERIAL_SPEED);
	BT_begin(BT_SPEED);
	LOG(PROJECT_NAME);

	//
 	// Setup Radio Control
	// 
	RadioControl_begin();
 
	// 
	// Setup Motors
	// 
	Motors::begin();
	Motors::enable(&RadioChannel3.timer, RadioChannel3.MIN+1000, &RadioChannel4.timer, RadioChannel4.MAX-1000);

	// 
	// Setup AHRS
	// 
	if(!AHRS::begin())	while(true);

	// 
	// Setup Stabilizer
	// 
	Stabilizer::begin();

	// 
	// Timer setting for control loop
	// 
	// configTimer(TC0, 2, TC2_IRQn, SAMPLE_RATE_LOOP);
	

	LOGln();
	LOGln("ARM:    --- Move the left stick down and to the right...");
	LOGln("DISARM: --- Move the left stick down and to the left...");
	LOGln();
	LOGln("\n------------ Let's fly! ------------");

}

void loop() {

	droneChangeState(RadioChannel3.timer, RadioChannel3.MIN, RadioChannel4.timer, RadioChannel4.MIN, RadioChannel4.MAX);
	ledDebug();

	if(DroneState != ARMED){
		Stabilizer::resetPID();
		Motors::stop();
		return;
	}


	if(AHRS::readData()){

		float throttleSetPoint = map(RadioChannel3.timer, RadioChannel3.MIN, RadioChannel3.MAX, THROTTLE_MIN, THROTTLE_MAX);
		// float yawSetPoint = map(RadioChannel4.timer, RadioChannel4.MIN, RadioChannel4.MAX, YAW_MIN, YAW_MAX);
		// float pitchSetPoint = map(RadioChannel2.timer, RadioChannel2.MIN, RadioChannel2.MAX, PITCH_MIN, PITCH_MAX);
		float rollSetPoint = map(RadioChannel1.timer, RadioChannel1.MIN, RadioChannel1.MAX, ROLL_MIN, ROLL_MAX);

		float yawSetPoint = 0;
		float pitchSetPoint = 0;

		// === CALCULATE PID
		Stabilizer::throttleUpdateSetPoint(throttleSetPoint);
		// Stabilizer::pitchUpdateInput(AHRS::getAltitude());

		Stabilizer::pitchUpdateSetPoint(pitchSetPoint);
		Stabilizer::pitchUpdateInput(AHRS::getPitch());

		Stabilizer::rollUpdateSetPoint(rollSetPoint);
		Stabilizer::rollUpdateInput(AHRS::getRoll());

		Stabilizer::stabilize();
		

		// === LOAD DATA TO INTERFACE
		DataToSend[SendID::YAW_IN] = AHRS::getYaw();
		DataToSend[SendID::YAW_SET] = yawSetPoint;
		DataToSend[SendID::PITCH_IN] = AHRS::getPitch();
		DataToSend[SendID::PITCH_SET] = pitchSetPoint;
		DataToSend[SendID::ROLL_IN] = AHRS::getRoll();
		DataToSend[SendID::ROLL_SET] = rollSetPoint;
		DataToSend[SendID::M1_VEL] = *(Motors::getPercentPower());
		DataToSend[SendID::M2_VEL] = *(Motors::getPercentPower()+1);
		DataToSend[SendID::M3_VEL] = *(Motors::getPercentPower()+2);
		DataToSend[SendID::M4_VEL] = *(Motors::getPercentPower()+3);
		// === SEND DATA TO INTERFACE
		SendDataToInterface();
	}

}


/*void TC2_Handler(void){
	TC_GetStatus(TC0, 2);

	if(DroneState != ARMED)
		return;


	if(AHRS::readData()){

		float throttleSetPoint = map(RadioChannel3.timer, RadioChannel3.MIN, RadioChannel3.MAX, THROTTLE_MIN, THROTTLE_MAX);
		// float yawSetPoint = map(RadioChannel4.timer, RadioChannel4.MIN, RadioChannel4.MAX, YAW_MIN, YAW_MAX);
		// float pitchSetPoint = map(RadioChannel2.timer, RadioChannel2.MIN, RadioChannel2.MAX, PITCH_MIN, PITCH_MAX);
		// float rollSetPoint = map(RadioChannel1.timer, RadioChannel1.MIN, RadioChannel1.MAX, ROLL_MIN, ROLL_MAX);

		float yawSetPoint = 0;
		float pitchSetPoint = 0;
		float rollSetPoint = 0;

		// === CALCULATE PID
		Stabilizer::throttleUpdateSetPoint(throttleSetPoint);

		Stabilizer::pitchUpdateSetPoint(pitchSetPoint);
		Stabilizer::pitchUpdateInput(AHRS::getPitch());

		Stabilizer::rollUpdateSetPoint(rollSetPoint);
		Stabilizer::rollUpdateInput(AHRS::getRoll());

		Stabilizer::stabilize();


		// === LOAD DATA TO INTERFACE
		DataToSend[SendID::YAW_IN] = AHRS::getYaw();
		DataToSend[SendID::YAW_SET] = yawSetPoint;
		DataToSend[SendID::PITCH_IN] = AHRS::getPitch();
		DataToSend[SendID::PITCH_SET] = pitchSetPoint;
		DataToSend[SendID::ROLL_IN] = AHRS::getRoll();
		DataToSend[SendID::ROLL_SET] = rollSetPoint;
		DataToSend[SendID::M1_VEL] = *(Motors::getPercentPower());
		DataToSend[SendID::M2_VEL] = *(Motors::getPercentPower()+1);
		DataToSend[SendID::M3_VEL] = *(Motors::getPercentPower()+2);
		DataToSend[SendID::M4_VEL] = *(Motors::getPercentPower()+3);
	}
}*/


// === RECEIVE INTERFACE DATA
BT_Event(){

	if(ReceiveInterfaceData()){
		Stabilizer::pidYaw.setKP(DataReceived[ReceiveID::KP_YAW]);
		Stabilizer::pidYaw.setKI(DataReceived[ReceiveID::KI_YAW]);
		Stabilizer::pidYaw.setKD(DataReceived[ReceiveID::KD_YAW]);

		Stabilizer::pidPitch.setKP(DataReceived[ReceiveID::KP_PITCH]);
		Stabilizer::pidPitch.setKI(DataReceived[ReceiveID::KI_PITCH]);
		Stabilizer::pidPitch.setKD(DataReceived[ReceiveID::KD_PITCH]);

		Stabilizer::pidRoll.setKP(DataReceived[ReceiveID::KP_ROLL]);
		Stabilizer::pidRoll.setKI(DataReceived[ReceiveID::KI_ROLL]);
		Stabilizer::pidRoll.setKD(DataReceived[ReceiveID::KD_ROLL]);
	}

}