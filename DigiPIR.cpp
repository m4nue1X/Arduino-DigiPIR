#include "DigiPIR.h"
#include <Arduino.h>

volatile bool DigiPIR::trigger = 0; 

void DigiPIR::begin(int serial_pin, int direct_link_pin, bool use_interrupt) {
  if(serial_pin < 0 || direct_link_pin < 0) {
    end();
    return;
  }
  
  ser_pin = serial_pin;
  dl_pin = direct_link_pin;
  interrupt_en = use_interrupt;

  pinMode(ser_pin, OUTPUT);
  pinMode(dl_pin, INPUT);

  sendConfig();

  if(interrupt_en) {
    attachInterrupt(digitalPinToInterrupt(dl_pin), &DigiPIR::handleInterrupt, HIGH);
  }
}

void DigiPIR::end() {
  if(dl_pin >= 0) {
    detachInterrupt(digitalPinToInterrupt(dl_pin));
  }
  
  ser_pin = -1;
  dl_pin = -1;
}

void DigiPIR::configure(uint8_t threshold, uint8_t blind_time, uint8_t pulse_ctr, uint8_t window_time, uint32_t signal_source, 
                        uint32_t hpf_cut_off, uint32_t pulse_detection_mode) {
  uint32_t new_config = kConfigReservedValue;
  new_config |= (uint32_t(threshold) << kConfigDetectionThresholdShift) & kConfigDetectionThresholdMask;
  new_config |= (uint32_t(blind_time) << kConfigBlindTimeShift) & kConfigBlindTimeMask;
  new_config |= (uint32_t(pulse_ctr) << kConfigPulseCounterShift) & kConfigPulseCounterMask;
  new_config |= (uint32_t(window_time) << kConfigWindowTimeShift) & kConfigWindowTimeMask;
  new_config |= kConfigOperationModeValueWakeUp & kConfigOperationModeMask;
  new_config |= signal_source & kConfigSignalSourceMask;
  new_config |= hpf_cut_off & kConfigSignalHPFCutOffMask;
  new_config |= pulse_detection_mode & kConfigPulseDetectionModeMask;
  config = new_config;
  sendConfig();
}

void DigiPIR::sendConfig() {
  if(ser_pin < 0) {
    return;
  }
  
  noInterrupts();
  digitalWrite(ser_pin, LOW);
  for(int i=24; i>=0; i--) {
    digitalWrite(ser_pin, HIGH);
    digitalWrite(ser_pin, (config >> i) & 0x1);
    delayMicroseconds(kTimingBitTime);
    digitalWrite(ser_pin, LOW);
  }
  interrupts();
  delayMicroseconds(kTimingPauseTime);
}

void DigiPIR::handleInterrupt() {
  trigger = true;
}

bool DigiPIR::triggered() {
  if(dl_pin < 0) {
    return false;
  }

  uint8_t state = digitalRead(dl_pin);
  if(interrupt_en) {
    // if interrupt handling is enabled ..
    if(!trigger && state != LOW) {
      // .. cancel reading in case we didn't get an interrupt but there is data
      cancelRead();
    } else if(trigger && state == LOW) {
      // .. return false if we got an interrupt but there is no data
      return false;
    } else {
      // in all other (i. e. trigger is true and state is HIGH or trigger is false 
      // and state is LOW) return trigger
      return trigger;
    }
  } else {
    // if interrupt handling is disabled, rely on the availability of data
    return (state == HIGH);
  }
}

int DigiPIR::readData(int* adc_value) {
  if(dl_pin < 0) {
    return -1;
  }
  
  // Check if start condition is met
  if(digitalRead(dl_pin) != HIGH) {
    trigger = false;
    return -1;
  }
  trigger = false;

  // Make sure we waited at least kTimingReadoutDelay after the interrupt
  delayMicroseconds(kTimingReadoutDelay);

  noInterrupts();
  uint64_t res = 0;
  for(int i=39; i>=0; i--) {
    // Pull line low
    // NOTE: setting the pin to an output pin will write the previously set value
    // therefore we set it to low first and then switch to output
    digitalWrite(dl_pin, LOW);
    pinMode(dl_pin, OUTPUT);
    // Create low to high transition
    digitalWrite(dl_pin, HIGH);
    // Release pin
    pinMode(dl_pin, INPUT);
    // Read pin
    delayMicroseconds(kTimingReadoutBitTime);
    res |= uint64_t((digitalRead(dl_pin) == HIGH) ? 0x1 : 0x0) << i;
    delayMicroseconds(1);
  }
  // We're done -> force to low for >= 500ns
  pinMode(dl_pin, OUTPUT);
  digitalWrite(dl_pin, LOW);
  delayMicroseconds(1);
  // Release the pin again
  pinMode(dl_pin, INPUT);
  interrupts();

  if((res & kDataOutOfRangeMask) != kDataOutOfRangeValueOk) {
    return -2;
  }
  
  if(adc_value != NULL) {
    if((res & kConfigSignalSourceMask) == kConfigSignalSourceValuePIRBPF) {
      // signed integer as two's complement between -8192 .. 8191
      *adc_value = (res & kDataADCCountsMask) >> kDataADCCountsShift;
      if((*adc_value & 0x2000) != 0) {
        *adc_value = int(-1) & *adc_value;
      }
    } else {
      // unsigned value between 0 .. 16383
      *adc_value = (res & kDataADCCountsMask) >> kDataADCCountsShift;
    }
    
  }

  return 0;
}

void DigiPIR::cancelRead() {
  if(dl_pin < 0) {
    return;
  }
  
  digitalWrite(dl_pin, LOW);
  pinMode(dl_pin, OUTPUT);
  delayMicroseconds(kTimingReadoutCancel);
  pinMode(dl_pin, INPUT);
}
