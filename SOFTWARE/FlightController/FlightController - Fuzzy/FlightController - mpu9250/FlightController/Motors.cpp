#include "Motors.h"

uint32_t Motors::powers[MOTORS_COUNT];

void Motors::begin(){

	LOG("\nInitializing Motors...\n");

	// PWM Startup code
	pmc_enable_periph_clk(PWM_INTERFACE_ID);
	PWMC_ConfigureClocks(FREQUENCY * MAX_DUTY_CYCLE_PWM, 0, VARIANT_MCK);

	for (uint8_t i = 0; i < MOTORS_COUNT; i++){
		uint8_t pin = MOTOR_PINS[i];
		uint32_t channel = g_APinDescription[pin].ulPWMChannel;
		uint32_t attr = g_APinDescription[pin].ulPinAttribute;

		// If it isn't a PWM pin, "continue"
		if((attr & PIN_ATTR_PWM) != PIN_ATTR_PWM){
			LOG("Pin "); LOG(pin); LOGln(" not is PWM pin.");
			continue;
		}

		pinMode(pin, OUTPUT);

		// Setup PWM for this pin
		PIO_Configure(g_APinDescription[pin].pPort,
				g_APinDescription[pin].ulPinType,
				g_APinDescription[pin].ulPin,
				g_APinDescription[pin].ulPinConfiguration);
		PWMC_ConfigureChannel(PWM_INTERFACE, channel, PWM_CMR_CPRE_CLKA, 0, 0);
		PWMC_SetPeriod(PWM_INTERFACE, channel, MAX_DUTY_CYCLE_PWM);
		PWMC_SetDutyCycle(PWM_INTERFACE, channel, 0);
		PWMC_EnableChannel(PWM_INTERFACE, channel);
	}

	// Stop motors
	Motors::stop();
}

void Motors::setPower(uint32_t powers[MOTORS_COUNT]){

	for (int i = 0; i < MOTORS_COUNT; i++){
		
		uint32_t duty = powers[i];

		// Apply saturation
		duty = (duty < MIN_DUTY_CYCLE) ? MIN_DUTY_CYCLE : duty;
		duty = (duty > MAX_DUTY_CYCLE) ? MAX_DUTY_CYCLE : duty;

		// Update Duty Cycle for this pin
		uint32_t channel = g_APinDescription[MOTOR_PINS[i]].ulPWMChannel;
		PWMC_SetDutyCycle(PWM_INTERFACE, channel, duty);

		Motors::powers[i] = mapFloat(duty, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE, 0, 100);
	}

}

uint32_t* Motors::getPercentPower(){
	return Motors::powers;
}

void Motors::stop(){
	uint32_t powers[] = {MIN_DUTY_CYCLE, MIN_DUTY_CYCLE, MIN_DUTY_CYCLE, MIN_DUTY_CYCLE};
	Motors::setPower(powers);
}
