#pragma once


class RadioChannel{

private:

	int pin;
	int lastTime;
	int interval;

public:

	void begin(void (*_callback)(void)){
		pinMode(pin, INPUT);

		attachInterrupt(digitalPinToInterrupt(pin), _callback, CHANGE);
	}
	
	void setPin(int _pin){
		pin = _pin;
	}

	int getPin(){
		return pin;
	}

	void setTimeHigh(int currentTime){
		lastTime = currentTime;
	}

	void calcInterval(int currentTime){
		interval = currentTime - lastTime;
	}

	int getTime(){
		return interval; 
	}
};