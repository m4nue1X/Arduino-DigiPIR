
/**********************************************************
 * DigiPIR.h                                              *
 *                                                        *
 * Author: Manuel Schoch <info@manuel-schoch.net>         *                                                       *
 *                                                        *
 **********************************************************/

#ifndef DIGIPIR_H
#define DIGIPIR_H

#include <stdint.h>

#ifdef ESP8266
#include <ESP8266WiFi.h>
#define INTERRUPT_ROUTINE_ATTTR ICACHE_RAM_ATTR
#else
#define INTERRUPT_ROUTINE_ATTTR
#endif

class DigiPIR {
public:
  // Config values
  // Reserved config fileds
  static constexpr uint32_t kConfigReservedValue = (2 << 3);
  static constexpr uint32_t kConfigReservedMask = (0x3 << 3 | 0x1 << 1);
  // Detection threshold
  static constexpr uint32_t kConfigDetectionThresholdShift = 17;
  static constexpr uint32_t kConfigDetectionThresholdMask = (0xFF << kConfigDetectionThresholdShift);
  // Blind time
  static constexpr uint32_t kConfigBlindTimeShift = 13; // 0.5s + <val> * 0.5s
  static constexpr uint32_t kConfigBlindTimeMask = (0xF << kConfigBlindTimeShift);
  // Pulse counter
  static constexpr uint32_t kConfigPulseCounterShift = 11; // 1 + <val>
  static constexpr uint32_t kConfigPulseCounterMask = (0x3 << kConfigPulseCounterShift);
  // Window time
  static constexpr uint32_t kConfigWindowTimeShift = 9; // 2s + <val> * 2s
  static constexpr uint32_t kConfigWindowTimeMask = (0x3 << kConfigWindowTimeShift);
  // Operation mode
  // - kConfigOperationModeValueForcedReadout: forced read out initiated by host (not supported currently)
  // - kConfigOperationModeValueInterruptReadout: periodic interrupt read out every ~16ms initiated by sensor (not supported currently)
  // - kConfigOperationModeValueWakeUp: interrupt triggered by sensor (/read out) when motion detected (used by this library)
  static constexpr uint32_t kConfigOperationModeShift = 7;
  static constexpr uint32_t kConfigOperationModeMask = (0x3 << kConfigOperationModeShift);
  static constexpr uint32_t kConfigOperationModeValueForcedReadout = (0x0 << kConfigOperationModeShift);
  static constexpr uint32_t kConfigOperationModeValueInterruptReadout = (0x1 << kConfigOperationModeShift);
  static constexpr uint32_t kConfigOperationModeValueWakeUp = (0x2 << kConfigOperationModeShift);
  // Signal source
  // - kConfigSignalSourceValuePIRBPF: Use band pass filter output as source for ADC value
  // - kConfigSignalSourceValuePIRLPF: Use band low filter output as source for ADC value
  // - kConfigSignalSourceValueTemperatureSensor: Use temperature sensor as source for ADC value
  static constexpr uint32_t kConfigSignalSourceShift = 5;
  static constexpr uint32_t kConfigSignalSourceMask = (0x3 << kConfigSignalSourceShift);
  static constexpr uint32_t kConfigSignalSourceValuePIRBPF = (0x0 << kConfigSignalSourceShift);
  static constexpr uint32_t kConfigSignalSourceValuePIRLPF = (0x1 << kConfigSignalSourceShift);
  static constexpr uint32_t kConfigSignalSourceValueTemperatureSensor = (0x3 << kConfigSignalSourceShift);
  // High pass filter cut-off frequency
  // - kConfigSignalHPFCutOffValueDot4Hz: 0.4Hz
  // - kConfigSignalHPFCutOffValueDot2Hz: 0.2Hz
  static constexpr uint32_t kConfigSignalHPFCutOffShift = 2;
  static constexpr uint32_t kConfigSignalHPFCutOffMask = (0x1 << kConfigSignalHPFCutOffShift);
  static constexpr uint32_t kConfigSignalHPFCutOffValueDot4Hz = (0x0 << kConfigSignalHPFCutOffShift);
  static constexpr uint32_t kConfigSignalHPFCutOffValueDot2Hz = (0x1 << kConfigSignalHPFCutOffShift);
  // Pulse detection mode
  // - kConfigPulseDetectionModeValueWithBFPSignChange: count only pulses above threshold after pulse sign changed
  // - kConfigPulseDetectionModeValueWithoutBFPSignChange: count all pulses above threshold
  static constexpr uint32_t kConfigPulseDetectionModeShift = 0;
  static constexpr uint32_t kConfigPulseDetectionModeMask = (0x1 << kConfigPulseDetectionModeShift);
  static constexpr uint32_t kConfigPulseDetectionModeValueWithBFPSignChange = (0x0 << kConfigPulseDetectionModeShift);
  static constexpr uint32_t kConfigPulseDetectionModeValueWithoutBFPSignChange = (0x1 << kConfigPulseDetectionModeShift);
  
  // Data values
  // Out of range
  // - kDataOutOfRangeValueReset: signal of PIR was out of range, reset performed
  // - kDataOutOfRangeValueOk: normal operation
  static constexpr uint32_t kDataOutOfRangeShift = 39;
  static constexpr uint64_t kDataOutOfRangeMask = (0x1ull << kDataOutOfRangeShift);
  static constexpr uint64_t kDataOutOfRangeValueReset = (0x0ull << kDataOutOfRangeShift);
  static constexpr uint64_t kDataOutOfRangeValueOk = (0x1ull << kDataOutOfRangeShift);
  // ADC counts
  static constexpr uint32_t kDataADCCountsShift = 25;
  static constexpr uint64_t kDataADCCountsMask = (0x3FFFull << kDataADCCountsShift);
  // Config used
  static constexpr uint32_t kDataConfigShift = 0;
  static constexpr uint64_t kDataConfigMask = 0x1FFFFFFull;

  // Timing values
  static constexpr uint32_t kTimingBitTime = 72;
  static constexpr uint32_t kTimingPauseTime = 580 + 5;
  static constexpr uint32_t kTimingReadoutDelay = 75;
  static constexpr uint32_t kTimingReadoutBitTime = 2 + 5; // oscilloscope shows that bit settled ~2us after pulled high by host, add 5 for safety
  static constexpr uint32_t kTimingReadoutCancel = 145;

  /**
   * Start monitoring PIR sensor with specified serial pin and direct link pin.
   * Configures the PIR with default config/last configuration set. Registerns an interrup handler for the direct link pin.
   * @param serial_pin The pin number the PIRs serial pin is conncetect to
   * @param direct_link_pin The pin number the PIRs direct link pin is connected to
   * @param use_interrupt Set true if an interrupt handler should be registered and interrup handling enabled (recommended).
   * Set to false e. g. if your borad doesn't support an interrupt and you want to poll (or, e. g. want to wake up ESP8266 from
   * light sleep when the pin goes high)
   */
  void begin(int serial_pin, int direct_link_pin, bool use_interrupt=true);
  void end();

  /**
   * Configures the PIR with the specified parameters.
   * @param threshold Detection threshold after band pass filtering (0-255; default: 128)
   * @param blind_time Re-triggering surpression duration (0.5s + <value>*0.5s; 0-15; default: 1)
   * @param pulse_ctr Number of pulses above threshold within time window required to trigger (1 + <value>; 0-3; default: 0)
   * @param window_time Length of the time frame within n pulses above thresold must be registered to trigger (2s + <value>*2s; 0-3; default: 0)
   * @param signal_source Source of the signal returned as <code>adc_value</code> of readData(). (default: kConfigSignalSourceValuePIRBPF)
   * @param hpf_cut_off High pass filter cut-off frequence in band pass filter. (default: kConfigSignalHPFCutOffValueDot4Hz)
   * @param pulse_detection_mode Specifies whether a zero-crossing is required before a new pulse is counted (default: kConfigPulseDetectionModeValueWithBFPSignChange)
   */
  void configure(uint8_t threshold=128, uint8_t blind_time=1, uint8_t pulse_ctr=0, uint8_t window_time=0, uint32_t signal_source=kConfigSignalSourceValuePIRBPF, 
                 uint32_t hpf_cut_off=kConfigSignalHPFCutOffValueDot4Hz, uint32_t pulse_detection_mode=kConfigPulseDetectionModeValueWithBFPSignChange);

  /**
   * Returns true if this PIR has been triggered and not yet read.
   * If the direct link pin signalls data but the trigger has not been set, a read-out cancellation is performed.
   * @return true if PIR has been triggered, false otherwise.
   */
  bool triggered();

  /**
   * Reads the data from the last trigger from the sensor.
   * @param Optional output parameter providing the ADC value configured to be the signal source (-8192 .. 8191 if signal source is BFP; 0 .. 16383 otherwise)
   * @return 0 if successful, < 0 if an error occurred
   */
  int readData(int* adc_value=NULL);

private:
  int ser_pin = -1;
  int dl_pin = -1;
  bool interrupt_en = true;
  uint32_t config = (kConfigReservedMask & kConfigReservedValue) | 
                    (~kConfigReservedMask & (
                      (128 << kConfigDetectionThresholdShift) | (1 << kConfigBlindTimeShift) | (0 << kConfigPulseCounterShift) | (0 << kConfigWindowTimeShift) |
                      kConfigOperationModeValueWakeUp | kConfigSignalSourceValuePIRBPF | kConfigSignalHPFCutOffValueDot4Hz | kConfigPulseDetectionModeValueWithBFPSignChange)
                    );
  static volatile bool trigger;

  static INTERRUPT_ROUTINE_ATTTR void handleInterrupt();
  void sendConfig();
  void cancelRead();
  
};

#endif /* DIGIPIR_H */
