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
	volatile uint32_t EDGE_RISING = 0;
	volatile uint32_t EDGE_FALLING = 0;
	volatile uint32_t timer = 0xFFFF;
};


// 
// DEFINITIONS OF CHANNEL 5
// 
Channel RadioChannel5;
#define CH5_TC TC0
#define CH5_CHANNEL 0
#define CH5_IRQn TC0_IRQn
#define CH5_Handler TC0_Handler
#define CH5_ID ID_TC0
#define CH5_PIN 2

// 
// DEFINITIONS OF CHANNEL 4
// 
Channel RadioChannel4;
#define CH4_TC TC2
#define CH4_CHANNEL 1
#define CH4_IRQn TC7_IRQn
#define CH4_Handler TC7_Handler
#define CH4_ID ID_TC7
#define CH4_PIN 3

// 
// DEFINITIONS OF CHANNEL 3
// 
Channel RadioChannel3;
#define CH3_TC TC2
#define CH3_CHANNEL 0
#define CH3_IRQn TC6_IRQn
#define CH3_Handler TC6_Handler
#define CH3_ID ID_TC6
#define CH3_PIN 5

// 
// DEFINITIONS OF CHANNEL 2
// 
Channel RadioChannel2;
#define CH2_TC TC2
#define CH2_CHANNEL 2
#define CH2_IRQn TC8_IRQn
#define CH2_Handler TC8_Handler
#define CH2_ID ID_TC8
#define CH2_PIN 11

// 
// DEFINITIONS OF CHANNEL 1
// 
Channel RadioChannel1;
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

		RadioChannel1.EDGE_RISING = CH1_TC->TC_CHANNEL[CH1_CHANNEL].TC_RA;
		RadioChannel1.EDGE_FALLING = CH1_TC->TC_CHANNEL[CH1_CHANNEL].TC_RB;

		RadioChannel1.timer = RadioChannel1.EDGE_FALLING - RadioChannel1.EDGE_RISING;
	}
}

void CH2_Handler() {
	if ((TC_GetStatus(CH2_TC, CH2_CHANNEL) & TC_SR_LDRBS) == TC_SR_LDRBS) {

		RadioChannel2.EDGE_RISING = CH2_TC->TC_CHANNEL[CH2_CHANNEL].TC_RA;
		RadioChannel2.EDGE_FALLING = CH2_TC->TC_CHANNEL[CH2_CHANNEL].TC_RB;

		RadioChannel2.timer = RadioChannel2.EDGE_FALLING - RadioChannel2.EDGE_RISING;
	}
}

void CH3_Handler() {
	if ((TC_GetStatus(CH3_TC, CH3_CHANNEL) & TC_SR_LDRBS) == TC_SR_LDRBS) {

		RadioChannel3.EDGE_RISING = CH3_TC->TC_CHANNEL[CH3_CHANNEL].TC_RA;
		RadioChannel3.EDGE_FALLING = CH3_TC->TC_CHANNEL[CH3_CHANNEL].TC_RB;

		RadioChannel3.timer = RadioChannel3.EDGE_FALLING - RadioChannel3.EDGE_RISING;
	}
}

void CH4_Handler() {
	if ((TC_GetStatus(CH4_TC, CH4_CHANNEL) & TC_SR_LDRBS) == TC_SR_LDRBS) {

		RadioChannel4.EDGE_RISING = CH4_TC->TC_CHANNEL[CH4_CHANNEL].TC_RA;
		RadioChannel4.EDGE_FALLING = CH4_TC->TC_CHANNEL[CH4_CHANNEL].TC_RB;

		RadioChannel4.timer = RadioChannel4.EDGE_FALLING - RadioChannel4.EDGE_RISING;
	}
}

void CH5_Handler() {
	if ((TC_GetStatus(CH5_TC, CH5_CHANNEL) & TC_SR_LDRBS) == TC_SR_LDRBS) {

		RadioChannel5.EDGE_RISING = CH5_TC->TC_CHANNEL[CH5_CHANNEL].TC_RA;
		RadioChannel5.EDGE_FALLING = CH5_TC->TC_CHANNEL[CH5_CHANNEL].TC_RB;

		RadioChannel5.timer = RadioChannel5.EDGE_FALLING - RadioChannel5.EDGE_RISING;
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
		if (RadioChannel1.timer < RadioChannel1.MIN) RadioChannel1.MIN = RadioChannel1.timer;
		if (RadioChannel1.timer > RadioChannel1.MAX) RadioChannel1.MAX = RadioChannel1.timer;
		if (RadioChannel2.timer < RadioChannel2.MIN) RadioChannel2.MIN = RadioChannel2.timer;
		if (RadioChannel2.timer > RadioChannel2.MAX) RadioChannel2.MAX = RadioChannel2.timer;
		if (RadioChannel3.timer < RadioChannel3.MIN) RadioChannel3.MIN = RadioChannel3.timer;
		if (RadioChannel3.timer > RadioChannel3.MAX) RadioChannel3.MAX = RadioChannel3.timer;
		if (RadioChannel4.timer < RadioChannel4.MIN) RadioChannel4.MIN = RadioChannel4.timer;
		if (RadioChannel4.timer > RadioChannel4.MAX) RadioChannel4.MAX = RadioChannel4.timer;
		if (RadioChannel5.timer < RadioChannel5.MIN) RadioChannel5.MIN = RadioChannel5.timer;
		if (RadioChannel5.timer > RadioChannel5.MAX) RadioChannel5.MAX = RadioChannel5.timer;
	
		LOG((millis() - now) % 1000 == 0 ? '.' : '\0');
		delay(100);
	}
	LOG(NEW_LINE);

	LOGln(F("--- Calibration values ---"));
	LOG(F("CH1 -->   MIN: ")); LOG(RadioChannel1.MIN); LOG(F("   MAX: ")); LOGln(RadioChannel1.MAX);
	LOG(F("CH2 -->   MIN: ")); LOG(RadioChannel2.MIN); LOG(F("   MAX: ")); LOGln(RadioChannel2.MAX);
	LOG(F("CH3 -->   MIN: ")); LOG(RadioChannel3.MIN); LOG(F("   MAX: ")); LOGln(RadioChannel3.MAX);
	LOG(F("CH4 -->   MIN: ")); LOG(RadioChannel4.MIN); LOG(F("   MAX: ")); LOGln(RadioChannel4.MAX);
	LOG(F("CH5 -->   MIN: ")); LOG(RadioChannel5.MIN); LOG(F("   MAX: ")); LOGln(RadioChannel5.MAX);
	LOG(NEW_LINE);
}


void printRadioChannels(){
	LOG("CH1: "); LOG(RadioChannel1.timer); LOG(TAB);
	LOG("CH2: "); LOG(RadioChannel2.timer); LOG(TAB);
	LOG("CH3: "); LOG(RadioChannel3.timer); LOG(TAB);
	LOG("CH4: "); LOG(RadioChannel4.timer); LOG(TAB);
	LOG("CH5: "); LOG(RadioChannel5.timer); LOG(NEW_LINE);
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
	RadioChannel1.MIN = 51596;	RadioChannel1.MAX = 68803;
	RadioChannel2.MIN = 51617;	RadioChannel2.MAX = 69439;
	RadioChannel3.MIN = 41125;	RadioChannel3.MAX = 67336;
	RadioChannel4.MIN = 51700;	RadioChannel4.MAX = 69069;
	RadioChannel5.MIN = 41065;	RadioChannel5.MAX = 82219;
#endif

}
