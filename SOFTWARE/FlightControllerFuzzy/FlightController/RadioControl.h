#pragma once

#include <Arduino.h>

#include "_config.h"

/* 
* PWM Capture using Arduino Due Timer Counters
* 
* Channels:
*   TC    Chan   NVIC irq   Handler       PMC ID   Arduino Pin
*   TC0   0      TC0_IRQn   TC0_Handler   ID_TC0   D2     (TIOA0)	CHANNEL 5
*   TC2   1      TC7_IRQn   TC7_Handler   ID_TC7   D3 	  (TIOA7)	CHANNEL 4
*   TC2   0      TC6_IRQn   TC6_Handler   ID_TC6   D5     (TIOA6)	CHANNEL 3
*   TC2   2      TC8_IRQn   TC8_Handler   ID_TC8   D11	  (TIOA8)	CHANNEL 2
*   TC0   1      TC1_IRQn   TC1_Handler   ID_TC1   61/A7  (TIOA1)	CHANNEL 1
* 
*/

// un/comment to enable/disable Calibration of channels
// #define CALIBRATE_RADIO
#define TIME_CALIBRATION	10000	// ms


// 
// GENERAL DEFINITIONS
// 
#define CLOCK_SELECTION TC_CMR_TCCLKS_TIMER_CLOCK1


struct Channel{
	uint32_t MIN = 0xFFFFFFFF;
	uint32_t MAX = 0;
	uint32_t offset = 0;
	volatile uint32_t EDGE_RISING = 0;
	volatile uint32_t EDGE_FALLING = 0;
	volatile uint32_t timer = 0xFFFF;
};


// 
// DEFINITIONS OF CHANNEL 5
// 
Channel SwitchChannel;
#define CH5_TC TC0
#define CH5_CHANNEL 0
#define CH5_IRQn TC0_IRQn
#define CH5_Handler TC0_Handler
#define CH5_ID ID_TC0
#define CH5_PIN 2

// 
// DEFINITIONS OF CHANNEL 4
// 
Channel YawChannel;
#define CH4_TC TC2
#define CH4_CHANNEL 1
#define CH4_IRQn TC7_IRQn
#define CH4_Handler TC7_Handler
#define CH4_ID ID_TC7
#define CH4_PIN 3

// 
// DEFINITIONS OF CHANNEL 3
// 
Channel ThrottleChannel;
#define CH3_TC TC2
#define CH3_CHANNEL 0
#define CH3_IRQn TC6_IRQn
#define CH3_Handler TC6_Handler
#define CH3_ID ID_TC6
#define CH3_PIN 5

// 
// DEFINITIONS OF CHANNEL 2
// 
Channel PitchChannel;
#define CH2_TC TC2
#define CH2_CHANNEL 2
#define CH2_IRQn TC8_IRQn
#define CH2_Handler TC8_Handler
#define CH2_ID ID_TC8
#define CH2_PIN 11

// 
// DEFINITIONS OF CHANNEL 1
// 
Channel RollChannel;
#define CH1_TC TC0
#define CH1_CHANNEL 1
#define CH1_IRQn TC1_IRQn
#define CH1_Handler TC1_Handler
#define CH1_ID ID_TC1
#define CH1_PIN 61


// 
// Setup functions
// 
void setupChannel1(){	
	// configure the PIO pin as peripheral
	PIO_Configure(
		g_APinDescription[CH1_PIN].pPort,
		g_APinDescription[CH1_PIN].ulPinType,
		g_APinDescription[CH1_PIN].ulPin,
		g_APinDescription[CH1_PIN].ulPinConfiguration
	);

	// enable timer peripheral clock
	pmc_enable_periph_clk(CH1_ID);

	// configure the timer
	TC_Configure(CH1_TC, CH1_CHANNEL,
		CLOCK_SELECTION /* Clock Selection */
		| TC_CMR_LDRA_RISING /* RA Loading: rising edge */
		| TC_CMR_LDRB_FALLING /* RB Loading: falling edge */
		| TC_CMR_ABETRG /* External Trigger */
		| TC_CMR_ETRGEDG_FALLING /* External Trigger Edge: Falling edge */
	);

	// configure TC interrupts
	NVIC_DisableIRQ(CH1_IRQn);
	NVIC_ClearPendingIRQ(CH1_IRQn);
	NVIC_SetPriority(CH1_IRQn, 0);
	NVIC_EnableIRQ(CH1_IRQn);

	// enable interrupts
	CH1_TC->TC_CHANNEL[CH1_CHANNEL].TC_IER = TC_IER_LDRBS;

	// start timer counter
	CH1_TC->TC_CHANNEL[CH1_CHANNEL].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;	
}

void setupChannel2(){
	// configure the PIO pin as peripheral
	PIO_Configure(
		g_APinDescription[CH2_PIN].pPort,
		g_APinDescription[CH2_PIN].ulPinType,
		g_APinDescription[CH2_PIN].ulPin,
		g_APinDescription[CH2_PIN].ulPinConfiguration
	);

	// enable timer peripheral clock
	pmc_enable_periph_clk(CH2_ID);

	// configure the timer
	TC_Configure(CH2_TC, CH2_CHANNEL,
		CLOCK_SELECTION 			/* Clock Selection */
		| TC_CMR_LDRA_RISING 		/* RA Loading: rising edge */
		| TC_CMR_LDRB_FALLING 		/* RB Loading: falling edge */
		| TC_CMR_ABETRG 			/* External Trigger*/
		| TC_CMR_ETRGEDG_FALLING 	/* External Trigger Edge: Falling edge */
	);

	// configure TC interrupts
	NVIC_DisableIRQ(CH2_IRQn);
	NVIC_ClearPendingIRQ(CH2_IRQn);
	NVIC_SetPriority(CH2_IRQn, 0);
	NVIC_EnableIRQ(CH2_IRQn);

	// enable interrupts
	CH2_TC->TC_CHANNEL[CH2_CHANNEL].TC_IER = TC_IER_LDRBS;

	// start timer counter
	CH2_TC->TC_CHANNEL[CH2_CHANNEL].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;	
}

void setupChannel3(){
	// configure the PIO pin as peripheral
	PIO_Configure(
		g_APinDescription[CH3_PIN].pPort,
		g_APinDescription[CH3_PIN].ulPinType,
		g_APinDescription[CH3_PIN].ulPin,
		g_APinDescription[CH3_PIN].ulPinConfiguration
	);

	// enable timer peripheral clock
	pmc_enable_periph_clk(CH3_ID);

	// configure the timer
	TC_Configure(CH3_TC, CH3_CHANNEL,
		CLOCK_SELECTION 			/* Clock Selection */
		| TC_CMR_LDRA_RISING 		/* RA Loading: rising edge */
		| TC_CMR_LDRB_FALLING 		/* RB Loading: falling edge */
		| TC_CMR_ABETRG 			/* External Trigger */
		| TC_CMR_ETRGEDG_FALLING 	/* External Trigger Edge: Falling edge */
	);

	// configure TC interrupts
	NVIC_DisableIRQ(CH3_IRQn);
	NVIC_ClearPendingIRQ(CH3_IRQn);
	NVIC_SetPriority(CH3_IRQn, 0);
	NVIC_EnableIRQ(CH3_IRQn);

	// enable interrupts
	CH3_TC->TC_CHANNEL[CH3_CHANNEL].TC_IER = TC_IER_LDRBS;

	// start timer counter
	CH3_TC->TC_CHANNEL[CH3_CHANNEL].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;	
}

void setupChannel4(){
	// configure the PIO pin as peripheral
	PIO_Configure(
		g_APinDescription[CH4_PIN].pPort,
		g_APinDescription[CH4_PIN].ulPinType,
		g_APinDescription[CH4_PIN].ulPin,
		g_APinDescription[CH4_PIN].ulPinConfiguration
	);

	// enable timer peripheral clock
	pmc_enable_periph_clk(CH4_ID);

	// configure the timer
	TC_Configure(CH4_TC, CH4_CHANNEL,
		CLOCK_SELECTION 			/* Clock Selection */
		| TC_CMR_LDRA_RISING 		/* RA Loading: rising edge */
		| TC_CMR_LDRB_FALLING 		/* RB Loading: falling edge */
		| TC_CMR_ABETRG 			/* External Trigger */
		| TC_CMR_ETRGEDG_FALLING 	/* External Trigger Edge: Falling edge */
	);

	// configure TC interrupts
	NVIC_DisableIRQ(CH4_IRQn);
	NVIC_ClearPendingIRQ(CH4_IRQn);
	NVIC_SetPriority(CH4_IRQn, 0);
	NVIC_EnableIRQ(CH4_IRQn);

	// enable interrupts
	CH4_TC->TC_CHANNEL[CH4_CHANNEL].TC_IER = TC_IER_LDRBS;

	// start timer counter
	CH4_TC->TC_CHANNEL[CH4_CHANNEL].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

void setupChannel5(){
	// configure the PIO pin as peripheral
	PIO_Configure(
		g_APinDescription[CH5_PIN].pPort,
		g_APinDescription[CH5_PIN].ulPinType,
		g_APinDescription[CH5_PIN].ulPin,
		g_APinDescription[CH5_PIN].ulPinConfiguration
	);

	// enable timer peripheral clock
	pmc_enable_periph_clk(CH5_ID);

	// configure the timer
	TC_Configure(CH5_TC, CH5_CHANNEL,
		CLOCK_SELECTION 			/* Clock Selection */
		| TC_CMR_LDRA_RISING 		/* RA Loading: rising edge */
		| TC_CMR_LDRB_FALLING 		/* RB Loading: falling edge */
		| TC_CMR_ABETRG 			/* External Trigger */
		| TC_CMR_ETRGEDG_FALLING 	/* External Trigger Edge: Falling edge */
	);

	// configure TC interrupts
	NVIC_DisableIRQ(CH5_IRQn);
	NVIC_ClearPendingIRQ(CH5_IRQn);
	NVIC_SetPriority(CH5_IRQn, 0);
	NVIC_EnableIRQ(CH5_IRQn);

	// enable interrupts
	CH5_TC->TC_CHANNEL[CH5_CHANNEL].TC_IER = TC_IER_LDRBS;

	// start timer counter
	CH5_TC->TC_CHANNEL[CH5_CHANNEL].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;	
}


// 
// Timer interrupt handles
// 
void CH1_Handler() {
	if ((TC_GetStatus(CH1_TC, CH1_CHANNEL) & TC_SR_LDRBS) == TC_SR_LDRBS) {

		RollChannel.EDGE_RISING = CH1_TC->TC_CHANNEL[CH1_CHANNEL].TC_RA;
		RollChannel.EDGE_FALLING = CH1_TC->TC_CHANNEL[CH1_CHANNEL].TC_RB;

		RollChannel.timer = RollChannel.EDGE_FALLING - RollChannel.EDGE_RISING;
	}
}

void CH2_Handler() {
	if ((TC_GetStatus(CH2_TC, CH2_CHANNEL) & TC_SR_LDRBS) == TC_SR_LDRBS) {

		PitchChannel.EDGE_RISING = CH2_TC->TC_CHANNEL[CH2_CHANNEL].TC_RA;
		PitchChannel.EDGE_FALLING = CH2_TC->TC_CHANNEL[CH2_CHANNEL].TC_RB;

		PitchChannel.timer = PitchChannel.EDGE_FALLING - PitchChannel.EDGE_RISING;
	}
}

void CH3_Handler() {
	if ((TC_GetStatus(CH3_TC, CH3_CHANNEL) & TC_SR_LDRBS) == TC_SR_LDRBS) {

		ThrottleChannel.EDGE_RISING = CH3_TC->TC_CHANNEL[CH3_CHANNEL].TC_RA;
		ThrottleChannel.EDGE_FALLING = CH3_TC->TC_CHANNEL[CH3_CHANNEL].TC_RB;

		ThrottleChannel.timer = ThrottleChannel.EDGE_FALLING - ThrottleChannel.EDGE_RISING;
	}
}

void CH4_Handler() {
	if ((TC_GetStatus(CH4_TC, CH4_CHANNEL) & TC_SR_LDRBS) == TC_SR_LDRBS) {

		YawChannel.EDGE_RISING = CH4_TC->TC_CHANNEL[CH4_CHANNEL].TC_RA;
		YawChannel.EDGE_FALLING = CH4_TC->TC_CHANNEL[CH4_CHANNEL].TC_RB;

		YawChannel.timer = YawChannel.EDGE_FALLING - YawChannel.EDGE_RISING;
	}
}

void CH5_Handler() {
	if ((TC_GetStatus(CH5_TC, CH5_CHANNEL) & TC_SR_LDRBS) == TC_SR_LDRBS) {

		SwitchChannel.EDGE_RISING = CH5_TC->TC_CHANNEL[CH5_CHANNEL].TC_RA;
		SwitchChannel.EDGE_FALLING = CH5_TC->TC_CHANNEL[CH5_CHANNEL].TC_RB;

		SwitchChannel.timer = SwitchChannel.EDGE_FALLING - SwitchChannel.EDGE_RISING;
	}
}

// 
// CALIBRATION
// 
void calibrateRadio(){
	LOG(NEW_LINE);
	LOG(F("Calibrating Radio"));

	uint32_t now = millis();
	delay(500);
	while(millis() - now < TIME_CALIBRATION){
		if (RollChannel.timer < RollChannel.MIN) RollChannel.MIN = RollChannel.timer;
		if (RollChannel.timer > RollChannel.MAX) RollChannel.MAX = RollChannel.timer;
		if (PitchChannel.timer < PitchChannel.MIN) PitchChannel.MIN = PitchChannel.timer;
		if (PitchChannel.timer > PitchChannel.MAX) PitchChannel.MAX = PitchChannel.timer;
		if (ThrottleChannel.timer < ThrottleChannel.MIN) ThrottleChannel.MIN = ThrottleChannel.timer;
		if (ThrottleChannel.timer > ThrottleChannel.MAX) ThrottleChannel.MAX = ThrottleChannel.timer;
		if (YawChannel.timer < YawChannel.MIN) YawChannel.MIN = YawChannel.timer;
		if (YawChannel.timer > YawChannel.MAX) YawChannel.MAX = YawChannel.timer;
		if (SwitchChannel.timer < SwitchChannel.MIN) SwitchChannel.MIN = SwitchChannel.timer;
		if (SwitchChannel.timer > SwitchChannel.MAX) SwitchChannel.MAX = SwitchChannel.timer;
	
		LOG((millis() - now) % 1000 == 0 ? '.' : '\0');
		delay(100);
	}
	LOG(NEW_LINE);

	LOGln(F("--- Calibration values ---"));
	LOG(F("CH1 -->   MIN: ")); LOG(RollChannel.MIN); LOG(F("   MAX: ")); LOGln(RollChannel.MAX);
	LOG(F("CH2 -->   MIN: ")); LOG(PitchChannel.MIN); LOG(F("   MAX: ")); LOGln(PitchChannel.MAX);
	LOG(F("CH3 -->   MIN: ")); LOG(ThrottleChannel.MIN); LOG(F("   MAX: ")); LOGln(ThrottleChannel.MAX);
	LOG(F("CH4 -->   MIN: ")); LOG(YawChannel.MIN); LOG(F("   MAX: ")); LOGln(YawChannel.MAX);
	LOG(F("CH5 -->   MIN: ")); LOG(SwitchChannel.MIN); LOG(F("   MAX: ")); LOGln(SwitchChannel.MAX);
	LOG(NEW_LINE);
}


void printRadioChannels(){
	LOG("CH1: "); LOG(RollChannel.timer); LOG(TAB);
	LOG("CH2: "); LOG(PitchChannel.timer); LOG(TAB);
	LOG("CH3: "); LOG(ThrottleChannel.timer); LOG(TAB);
	LOG("CH4: "); LOG(YawChannel.timer); LOG(TAB);
	LOG("CH5: "); LOG(SwitchChannel.timer); LOG(NEW_LINE);
}


// 
// RADIO BEGIN
// 
void RadioControl_begin(){

	LOG("\nInitializing Radio Control...\n");

	setupChannel1();
	setupChannel2();
	setupChannel3();
	setupChannel4();
	setupChannel5();	

#ifdef CALIBRATE_RADIO
	calibrateRadio();
#else
	RollChannel.MIN = 41158;	RollChannel.MAX = 75804;
	PitchChannel.MIN = 41180;	PitchChannel.MAX = 77020;
	ThrottleChannel.MIN = 41220;	ThrottleChannel.MAX = 76029;
	YawChannel.MIN = 41261;	YawChannel.MAX = 76318;
	SwitchChannel.MIN = 41053;	SwitchChannel.MAX = 41058;
#endif
	

	digitalWrite(PIN_LED_DEBUG, 1);
	delay(1000);


	for (int i = 0; i < 100; i++){
			float roll = mapFloat(RollChannel.timer, RollChannel.MIN, RollChannel.MAX, ROLL_MIN, ROLL_MAX);
			float pitch = mapFloat(PitchChannel.timer, PitchChannel.MIN, PitchChannel.MAX, PITCH_MIN, PITCH_MAX);
		
			RollChannel.offset += roll;
			PitchChannel.offset += pitch;
			delay(30);
	}

	RollChannel.offset /= 100;
	PitchChannel.offset /= 100;

	RollChannel.offset -= (ROLL_MAX+ROLL_MIN)/2.0f;
	PitchChannel.offset -= (PITCH_MAX+PITCH_MIN)/2.0f;
}
