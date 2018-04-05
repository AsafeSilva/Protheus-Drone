#pragma once

#include <Arduino.h>

#include "_config.h"

//
//	INTERFACE COMMUNICATION PROTOCOL
//
String parameter = "", data = ""; 
#define END_MSG 	'\n'
// *** Computer ---> Arduino *** //
#define YAW_KP 	"YP"
#define YAW_KI 	"YI"
#define YAW_KD 	"YD"
#define PITCH_KP 	"PP"
#define PITCH_KI 	"PI"
#define PITCH_KD 	"PD"
#define ROLL_KP 	"RP"
#define ROLL_KI 	"RI"
#define ROLL_KD 	"RD"
// *** Arduino ---> Computer *** //
#define YAW_INPUT 	"YI"
#define YAW_SETPOINT 	"YS"
#define PITCH_INPUT 	"PI"
#define PITCH_SETPOINT 	"PS"
#define ROLL_INPUT 	"RI"
#define ROLL_SETPOINT 	"RS"
#define MOTOR1 	"A"
#define MOTOR2 	"B"
#define MOTOR3 	"C"
#define MOTOR4 	"D"

#define SEND_DATA_AMOUNT 10
#define RECEIVE_DATA_AMOUNT 9

enum SendID { 
	YAW_IN = 0,
	YAW_SET,
	PITCH_IN,
	PITCH_SET,
	ROLL_IN,
	ROLL_SET,
	M1_VEL,
	M2_VEL,
	M3_VEL,
	M4_VEL
};

enum ReceiveID { 
	KP_YAW = 0,
	KI_YAW,
	KD_YAW,
	KP_PITCH,
	KI_PITCH,
	KD_PITCH,
	KP_ROLL,
	KI_ROLL,
	KD_ROLL
};


float DataToSend[SEND_DATA_AMOUNT] = {0.0f};
float DataReceived[RECEIVE_DATA_AMOUNT]= {0.0f};

void SendDataToInterface(){
	String data = "";

	data = String(DataToSend[SendID::YAW_IN]);
	BT_LOG(YAW_INPUT); 			BT_LOG(data);		BT_LOG(END_MSG);
	data = String(DataToSend[SendID::YAW_SET]);
	BT_LOG(YAW_SETPOINT); 		BT_LOG(data);		BT_LOG(END_MSG);
	data = String(DataToSend[SendID::PITCH_IN]);
	BT_LOG(PITCH_INPUT); 		BT_LOG(data);		BT_LOG(END_MSG);
	data = String(DataToSend[SendID::PITCH_SET]);
	BT_LOG(PITCH_SETPOINT); 	BT_LOG(data);		BT_LOG(END_MSG);
	data = String(DataToSend[SendID::ROLL_IN]);
	BT_LOG(ROLL_INPUT); 		BT_LOG(data);		BT_LOG(END_MSG);
	data = String(DataToSend[SendID::ROLL_SET]);
	BT_LOG(ROLL_SETPOINT); 		BT_LOG(data);		BT_LOG(END_MSG);
	data = String(DataToSend[SendID::M1_VEL]);
	BT_LOG(MOTOR1); 			BT_LOG(data);		BT_LOG(END_MSG);
	data = String(DataToSend[SendID::M2_VEL]);
	BT_LOG(MOTOR2); 			BT_LOG(data);		BT_LOG(END_MSG);
	data = String(DataToSend[SendID::M3_VEL]);
	BT_LOG(MOTOR3); 			BT_LOG(data);		BT_LOG(END_MSG);
	data = String(DataToSend[SendID::M4_VEL]);
	BT_LOG(MOTOR4); 			BT_LOG(data);		BT_LOG(END_MSG);
}


bool ReceiveInterfaceData(){
	char inChar = (char) BT_Read();

	if (inChar == END_MSG){

		float value = data.toFloat();

		if(parameter.equals(YAW_KP))		DataReceived[ReceiveID::KP_YAW] = value;
		else if(parameter.equals(YAW_KI))	DataReceived[ReceiveID::KI_YAW] = value;
		else if(parameter.equals(YAW_KD))	DataReceived[ReceiveID::KD_YAW] = value;
		else if(parameter.equals(PITCH_KP))	DataReceived[ReceiveID::KP_PITCH] = value;
		else if(parameter.equals(PITCH_KI))	DataReceived[ReceiveID::KI_PITCH] = value;
		else if(parameter.equals(PITCH_KD))	DataReceived[ReceiveID::KD_PITCH] = value;
		else if(parameter.equals(ROLL_KP))	DataReceived[ReceiveID::KP_ROLL] = value;
		else if(parameter.equals(ROLL_KI))	DataReceived[ReceiveID::KI_ROLL] = value;
		else if(parameter.equals(ROLL_KD))	DataReceived[ReceiveID::KD_ROLL] = value;

		parameter = "";	data = "";

		return true;

	}else{
		if(isAlpha(inChar))	parameter.concat(inChar);
		else				data.concat(inChar);
	}

	return false;
}