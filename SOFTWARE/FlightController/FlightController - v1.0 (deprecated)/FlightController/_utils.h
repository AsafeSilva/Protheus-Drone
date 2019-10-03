#pragma once

#include <Arduino.h>

#include "_config.h"



// ***************************************************************************************
enum STATE{
	DISARMED,
	ARMED,
	WAIT_ACTIVATION
};

static STATE DroneState = DISARMED;
// =======================================================================================


// ***************************************************************************************
static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
// =======================================================================================


// ***************************************************************************************
static uint8_t TimerBestClock(double frequency, uint32_t& retRC){
	/*
		Pick the best Clock, thanks to Ogle Basil Hall!
		Timer		Definition
		TIMER_CLOCK1	MCK /  2
		TIMER_CLOCK2	MCK /  8
		TIMER_CLOCK3	MCK / 32
		TIMER_CLOCK4	MCK /128
	*/
	const struct {
		uint8_t flag;
		uint8_t divisor;
	} clockConfig[] = {
		{ TC_CMR_TCCLKS_TIMER_CLOCK1,   2 },
		{ TC_CMR_TCCLKS_TIMER_CLOCK2,   8 },
		{ TC_CMR_TCCLKS_TIMER_CLOCK3,  32 },
		{ TC_CMR_TCCLKS_TIMER_CLOCK4, 128 }
	};
	float ticks;
	float error;
	int clkId = 3;
	int bestClock = 3;
	float bestError = 9.999e99;
	do
	{
		ticks = (float) VARIANT_MCK / frequency / (float) clockConfig[clkId].divisor;
		// error = abs(ticks - round(ticks));
		error = clockConfig[clkId].divisor * abs(ticks - round(ticks));	// Error comparison needs scaling
		if (error < bestError)
		{
			bestClock = clkId;
			bestError = error;
		}
	} while (clkId-- > 0);
	ticks = (float) VARIANT_MCK / frequency / (float) clockConfig[bestClock].divisor;
	retRC = (uint32_t) round(ticks);
	return clockConfig[bestClock].flag;
}
// =======================================================================================


// ***************************************************************************************
static void configTimer(Tc *tc, uint32_t channel, IRQn_Type IRQn, double frequency){
	pmc_set_writeprotect(false);
	pmc_enable_periph_clk(IRQn);

	uint32_t resetCounter = 0;
	uint8_t clock = TimerBestClock(frequency, resetCounter);

	TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | clock);
	TC_SetRC(tc, channel, resetCounter);

	tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
	tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;

	NVIC_ClearPendingIRQ(IRQn);
	NVIC_EnableIRQ(IRQn);

	TC_Start(tc, channel);
}
// =======================================================================================


// ***************************************************************************************
static bool ledState = false;
static unsigned long lastTime;
static unsigned long interval;
static void ledDebug(){

	if(millis() - lastTime > interval){

		ledState = !ledState;
		digitalWrite(PIN_LED_DEBUG, ledState);

		if(DroneState == ARMED)
			interval = ledState ? 100 : 900;
		if(DroneState == DISARMED)
			interval = 500;
		if(DroneState == WAIT_ACTIVATION)
			interval = ledState ? 900 : 100;

		lastTime = millis();
	}
}
// =======================================================================================


// ***************************************************************************************
static void droneChangeState(uint32_t throttle, uint32_t throttleMin, uint32_t yaw, uint32_t yawMin, uint32_t yawMax){
	if(DroneState == DISARMED){
		if((throttle < throttleMin+1000) && (yaw > yawMax-1000)){
			DroneState = ARMED;
			LOGln("ARMED!");
		}
	}else if(DroneState == ARMED){
		if((throttle < throttleMin+1000) && (yaw < yawMin+1000)){
			DroneState = DISARMED;
			LOGln("DISARMED!");
		}
	}
}
// =======================================================================================