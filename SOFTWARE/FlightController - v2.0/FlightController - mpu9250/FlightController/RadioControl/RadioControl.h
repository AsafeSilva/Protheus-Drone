#pragma once


#include "RadioChannel.h"

#define CH_1	0
#define CH_2	1	
#define CH_3	2
#define CH_4	3
#define CH_5	4
#define CH_6	5
#define CH_7	6
#define CH_8	7
#define CH_9	8
#define CH_10	9

extern void CHANNEL_1();
extern void CHANNEL_2();
extern void CHANNEL_3();
extern void CHANNEL_4();
extern void CHANNEL_5();
extern void CHANNEL_6();
extern void CHANNEL_7();
extern void CHANNEL_8();
extern void CHANNEL_9();
extern void CHANNEL_10();

#define CURRENT_TIME	micros()

typedef void (*Callback)(void);

class RadioControl{

private:

	RadioChannel* channels;
	int numChannels;

	Callback* callbacks;

public:

	RadioControl(int _numChannels, int* _pins, Callback* _callbacks){
		channels = new RadioChannel[_numChannels]();

		numChannels = _numChannels;

		callbacks = _callbacks;

		for (int i = 0; i < _numChannels; i++)
			channels[i].setPin(_pins[i]);
	}

	void begin(){
		for (int i = 0; i < numChannels; i++){
			channels[i].begin(callbacks[i]);
		}
	}

	void readChannel(int _channel, int _value){
		if(_value == HIGH)
			channels[_channel].setTimeHigh(CURRENT_TIME);
		else
			channels[_channel].calcInterval(CURRENT_TIME);
	}

	int getTimeChannel(int _channel){
		return channels[_channel].getTime();
	}

};
