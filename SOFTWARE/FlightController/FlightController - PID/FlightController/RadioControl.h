#pragma once

#include <Arduino.h>

#include "_config.h"

/*
  PWM Capture using Arduino Due Timer Counters

  Channels:
    TC    Chan   NVIC irq   Handler       PMC ID   Arduino Pin
    TC2   2      TC8_IRQn   TC8_Handler   ID_TC8   D11    (TIOA8) CHANNEL 1
    TC2   0      TC6_IRQn   TC6_Handler   ID_TC6   D5     (TIOA6) CHANNEL 2
    TC2   1      TC7_IRQn   TC7_Handler   ID_TC7   D3     (TIOA7) CHANNEL 3
    TC0   0      TC0_IRQn   TC0_Handler   ID_TC0   D2     (TIOA0) CHANNEL 4
    TC0   1      TC1_IRQn   TC1_Handler   ID_TC1   61/A7  (TIOA1) CHANNEL 5

*/

//
// GENERAL DEFINITIONS
//
#define CLOCK_SELECTION TC_CMR_TCCLKS_TIMER_CLOCK1
#define PRESCALER 2

//
// DEFINITIONS OF CHANNEL 1
//
#define CH1_TC TC2
#define CH1_CHANNEL 2
#define CH1_IRQn TC8_IRQn
#define CH1_Handler TC8_Handler
#define CH1_ID ID_TC8
#define CH1_PIN 11

//
// DEFINITIONS OF CHANNEL 2
//
#define CH2_TC TC2
#define CH2_CHANNEL 0
#define CH2_IRQn TC6_IRQn
#define CH2_Handler TC6_Handler
#define CH2_ID ID_TC6
#define CH2_PIN 5

//
// DEFINITIONS OF CHANNEL 3
//
#define CH3_TC TC2
#define CH3_CHANNEL 1
#define CH3_IRQn TC7_IRQn
#define CH3_Handler TC7_Handler
#define CH3_ID ID_TC7
#define CH3_PIN 3

//
// DEFINITIONS OF CHANNEL 4
//
#define CH4_TC TC0
#define CH4_CHANNEL 0
#define CH4_IRQn TC0_IRQn
#define CH4_Handler TC0_Handler
#define CH4_ID ID_TC0
#define CH4_PIN 2

//
// DEFINITIONS OF CHANNEL 5
//
#define CH5_TC TC0
#define CH5_CHANNEL 1
#define CH5_IRQn TC1_IRQn
#define CH5_Handler TC1_Handler
#define CH5_ID ID_TC1
#define CH5_PIN 61


class RadioChannel{
public:
  uint32_t min, max, offset;
  volatile uint32_t risingTime, fallingTime;
  volatile uint32_t interval;

  RadioChannel(){
    min = 0xFFFFFFFF;
    max = 0;
    offset = 0;
    risingTime = 0;
    fallingTime = 0;
    interval = 0;
  }

  void begin(uint32_t pin, uint32_t id, Tc* pTc, uint32_t channel, IRQn_Type IRQn){
    // configure the PIO pin as peripheral
    PIO_Configure(
      g_APinDescription[pin].pPort,
      g_APinDescription[pin].ulPinType,
      g_APinDescription[pin].ulPin,
      g_APinDescription[pin].ulPinConfiguration
    );

    // enable timer peripheral clock
    pmc_enable_periph_clk(id);

    // configure the timer
    TC_Configure(pTc, channel,
                 CLOCK_SELECTION /* Clock Selection */
                 | TC_CMR_LDRA_RISING /* RA Loading: rising edge */
                 | TC_CMR_LDRB_FALLING /* RB Loading: falling edge */
                 | TC_CMR_ABETRG /* External Trigger */
                 | TC_CMR_ETRGEDG_FALLING /* External Trigger Edge: Falling edge */
                );

    // configure TC interrupts
    NVIC_DisableIRQ(IRQn);
    NVIC_ClearPendingIRQ(IRQn);
    NVIC_SetPriority(IRQn, 0);
    NVIC_EnableIRQ(IRQn);

    // enable interrupts
    pTc->TC_CHANNEL[channel].TC_IER = TC_IER_LDRBS;

    // start timer counter
    pTc->TC_CHANNEL[channel].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
  }

  uint32_t read(){
    return interval;
  }

  volatile uint32_t* getInterval(){
    return &interval;
  }

  uint32_t getOffset(){
    return offset;
  }
  
};

class RadioControl{

private:

  static void calibrate(){
    LOG(NEW_LINE);
    LOG(F("Calibrating Radio"));

    uint32_t now = millis();
    delay(500);
    while (millis() - now < TIME_CALIBRATION) {
      if (RollChannel.interval < RollChannel.min) RollChannel.min = RollChannel.interval;
      if (RollChannel.interval > RollChannel.max) RollChannel.max = RollChannel.interval;
      if (PitchChannel.interval < PitchChannel.min) PitchChannel.min = PitchChannel.interval;
      if (PitchChannel.interval > PitchChannel.max) PitchChannel.max = PitchChannel.interval;
      if (ThrottleChannel.interval < ThrottleChannel.min) ThrottleChannel.min = ThrottleChannel.interval;
      if (ThrottleChannel.interval > ThrottleChannel.max) ThrottleChannel.max = ThrottleChannel.interval;
      if (YawChannel.interval < YawChannel.min) YawChannel.min = YawChannel.interval;
      if (YawChannel.interval > YawChannel.max) YawChannel.max = YawChannel.interval;
      if (SwitchChannel.interval < SwitchChannel.min) SwitchChannel.min = SwitchChannel.interval;
      if (SwitchChannel.interval > SwitchChannel.max) SwitchChannel.max = SwitchChannel.interval;

      if ((millis() - now) % 1000 == 0){
        LOG(".");
        digitalWrite(PIN_LED_ERROR, !digitalRead(PIN_LED_ERROR));
        digitalWrite(PIN_LED_DEBUG, !digitalRead(PIN_LED_DEBUG));
      }else{
        LOG("\0");
      }

      delay(100);
    }
    LOG(NEW_LINE);
    digitalWrite(PIN_LED_ERROR, 0);
    digitalWrite(PIN_LED_DEBUG, 0);


    LOGln(F("--- Calibration values ---"));
    LOG(F("#define ROLL_MIN ")); LOGln(RollChannel.min);
    LOG(F("#define ROLL_MAX ")); LOGln(RollChannel.max);
    LOG(F("#define PITCH_MIN "));  LOGln(PitchChannel.min); 
    LOG(F("#define PITCH_MAX "));  LOGln(PitchChannel.max); 
    LOG(F("#define THROTTLE_MIN ")); LOGln(ThrottleChannel.min);
    LOG(F("#define THROTTLE_MAX ")); LOGln(ThrottleChannel.max);
    LOG(F("#define YAW_MIN "));  LOGln(YawChannel.min); 
    LOG(F("#define YAW_MAX "));  LOGln(YawChannel.max); 
    LOG(F("#define SWITCH_MIN ")); LOGln(SwitchChannel.min);
    LOG(F("#define SWITCH_MAX ")); LOGln(SwitchChannel.max);
    LOG(NEW_LINE);
  }

public:

  static RadioChannel RollChannel, PitchChannel, ThrottleChannel, YawChannel, SwitchChannel;

  static bool calibrated;
  
  static void begin(){

    LOG("\nInitializing RadioControl...\n");

    pinMode(PIN_RADIO_CALIB, INPUT_PULLUP);

    RollChannel.begin(CH1_PIN, CH1_ID, CH1_TC, CH1_CHANNEL, CH1_IRQn);
    PitchChannel.begin(CH2_PIN, CH2_ID, CH2_TC, CH2_CHANNEL, CH2_IRQn);
    ThrottleChannel.begin(CH3_PIN, CH3_ID, CH3_TC, CH3_CHANNEL, CH3_IRQn);
    YawChannel.begin(CH4_PIN, CH4_ID, CH4_TC, CH4_CHANNEL, CH4_IRQn);
    SwitchChannel.begin(CH5_PIN, CH5_ID, CH5_TC, CH5_CHANNEL, CH5_IRQn);


    if(digitalRead(PIN_RADIO_CALIB) == false){

      calibrate();

      calibrated = true;
    }else{
      RollChannel.min = ROLL_MIN;
      RollChannel.max = ROLL_MAX;
      PitchChannel.min = PITCH_MIN;
      PitchChannel.max = PITCH_MAX;
      ThrottleChannel.min = THROTTLE_MIN;
      ThrottleChannel.max = THROTTLE_MAX;
      YawChannel.min = YAW_MIN;
      YawChannel.max = YAW_MAX;
      SwitchChannel.min = SWITCH_MIN;
      SwitchChannel.max = SWITCH_MAX;

      calibrated = true;
    }

    // Calculates offset
    // for (int i = 0; i < 100; i++) {
    //   RollChannel.offset += RollChannel.read();
    //   PitchChannel.offset += PitchChannel.read();
    //   delay(30);
    // }

    // RollChannel.offset /= 100;
    // PitchChannel.offset /= 100;

    // RollChannel.offset -= 1500;
    // PitchChannel.offset -= 1500;
  }

  static void debug(){
    LOG("CH1 (us): "); LOG(RollChannel.read());     LOG(TAB);
    LOG("CH2 (us): "); LOG(PitchChannel.read());    LOG(TAB);
    LOG("CH3 (us): "); LOG(ThrottleChannel.read()); LOG(TAB);
    LOG("CH4 (us): "); LOG(YawChannel.read());      LOG(TAB);
    LOG("CH5 (us): "); LOG(SwitchChannel.read());   LOG(NEW_LINE);
  }
  
};

RadioChannel RadioControl::RollChannel;
RadioChannel RadioControl::PitchChannel;
RadioChannel RadioControl::ThrottleChannel;
RadioChannel RadioControl::YawChannel;
RadioChannel RadioControl::SwitchChannel;
bool RadioControl::calibrated = false;


//
// Timer interrupt handles
//
void CH1_Handler() {
  if ((TC_GetStatus(CH1_TC, CH1_CHANNEL) & TC_SR_LDRBS) == TC_SR_LDRBS) {

    RadioControl::RollChannel.risingTime = CH1_TC->TC_CHANNEL[CH1_CHANNEL].TC_RA;
    RadioControl::RollChannel.fallingTime = CH1_TC->TC_CHANNEL[CH1_CHANNEL].TC_RB;

    RadioControl::RollChannel.interval = RadioControl::RollChannel.fallingTime - RadioControl::RollChannel.risingTime;
    RadioControl::RollChannel.interval = clockCyclesToMicroseconds(RadioControl::RollChannel.interval) * PRESCALER;
  
    if(RadioControl::calibrated){
      RadioControl::RollChannel.interval = (RadioControl::RollChannel.interval - RadioControl::RollChannel.min) * 1000.0 / (RadioControl::RollChannel.max - RadioControl::RollChannel.min) + 1000.0;
    
      if(RadioControl::RollChannel.interval > 0xFFFF)
        RadioControl::RollChannel.interval = 1000;
    }

  }
}

void CH2_Handler() {
  if ((TC_GetStatus(CH2_TC, CH2_CHANNEL) & TC_SR_LDRBS) == TC_SR_LDRBS) {

    RadioControl::PitchChannel.risingTime = CH2_TC->TC_CHANNEL[CH2_CHANNEL].TC_RA;
    RadioControl::PitchChannel.fallingTime = CH2_TC->TC_CHANNEL[CH2_CHANNEL].TC_RB;

    RadioControl::PitchChannel.interval = RadioControl::PitchChannel.fallingTime - RadioControl::PitchChannel.risingTime;
    RadioControl::PitchChannel.interval = clockCyclesToMicroseconds(RadioControl::PitchChannel.interval) * PRESCALER;
  
    if(RadioControl::calibrated){
      RadioControl::PitchChannel.interval = (RadioControl::PitchChannel.interval - RadioControl::PitchChannel.min) * 1000.0 / (RadioControl::PitchChannel.max - RadioControl::PitchChannel.min) + 1000.0;
    
      if(RadioControl::PitchChannel.interval > 0xFFFF)
        RadioControl::PitchChannel.interval = 1000;
    }
  }
}

void CH3_Handler() {
  if ((TC_GetStatus(CH3_TC, CH3_CHANNEL) & TC_SR_LDRBS) == TC_SR_LDRBS) {

    RadioControl::ThrottleChannel.risingTime = CH3_TC->TC_CHANNEL[CH3_CHANNEL].TC_RA;
    RadioControl::ThrottleChannel.fallingTime = CH3_TC->TC_CHANNEL[CH3_CHANNEL].TC_RB;

    RadioControl::ThrottleChannel.interval = RadioControl::ThrottleChannel.fallingTime - RadioControl::ThrottleChannel.risingTime;
    RadioControl::ThrottleChannel.interval = clockCyclesToMicroseconds(RadioControl::ThrottleChannel.interval) * PRESCALER;
  
    if(RadioControl::calibrated){
      RadioControl::ThrottleChannel.interval = (RadioControl::ThrottleChannel.interval - RadioControl::ThrottleChannel.min) * 1000.0 / (RadioControl::ThrottleChannel.max - RadioControl::ThrottleChannel.min) + 1000.0;

      if(RadioControl::ThrottleChannel.interval > 0xFFFF)
        RadioControl::ThrottleChannel.interval = 1000;
    }
  }
}

void CH4_Handler() {
  if ((TC_GetStatus(CH4_TC, CH4_CHANNEL) & TC_SR_LDRBS) == TC_SR_LDRBS) {

    RadioControl::YawChannel.risingTime = CH4_TC->TC_CHANNEL[CH4_CHANNEL].TC_RA;
    RadioControl::YawChannel.fallingTime = CH4_TC->TC_CHANNEL[CH4_CHANNEL].TC_RB;

    RadioControl::YawChannel.interval = RadioControl::YawChannel.fallingTime - RadioControl::YawChannel.risingTime;
    RadioControl::YawChannel.interval = clockCyclesToMicroseconds(RadioControl::YawChannel.interval) * PRESCALER;
  
    if(RadioControl::calibrated){
      RadioControl::YawChannel.interval = (RadioControl::YawChannel.interval - RadioControl::YawChannel.min) * 1000.0 / (RadioControl::YawChannel.max - RadioControl::YawChannel.min) + 1000.0;
    
      if(RadioControl::YawChannel.interval > 0xFFFF)
        RadioControl::YawChannel.interval = 1000;
    }
  }
}

void CH5_Handler() {
  if ((TC_GetStatus(CH5_TC, CH5_CHANNEL) & TC_SR_LDRBS) == TC_SR_LDRBS) {

    RadioControl::SwitchChannel.risingTime = CH5_TC->TC_CHANNEL[CH5_CHANNEL].TC_RA;
    RadioControl::SwitchChannel.fallingTime = CH5_TC->TC_CHANNEL[CH5_CHANNEL].TC_RB;

    RadioControl::SwitchChannel.interval = RadioControl::SwitchChannel.fallingTime - RadioControl::SwitchChannel.risingTime;
    RadioControl::SwitchChannel.interval = clockCyclesToMicroseconds(RadioControl::SwitchChannel.interval) * PRESCALER;
  
    if(RadioControl::calibrated){
      RadioControl::SwitchChannel.interval = (RadioControl::SwitchChannel.interval - RadioControl::SwitchChannel.min) * 1000.0 / (RadioControl::SwitchChannel.max - RadioControl::SwitchChannel.min) + 1000.0;
    
      if(RadioControl::SwitchChannel.interval > 0xFFFF)
        RadioControl::SwitchChannel.interval = 1000;
    }
  }
}
