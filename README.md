## DigiPIR

The DigiPIR library for Arduino provides an easy to use interface to PYD 1598 and 7655 pyro-electric motion sensors. The current version
only supports wake-up operation mode (i. e. the sensor triggers an interrupt on movement). The library is written for general use with
any Arduino borad with 2 GPIOs, one of them with interrupt support. It has been tested only with ESP8266 boards so far.

Simple example:
~~~~C++
#include <DigiPIR.h>

DigiPIR pir;

void setup() {
  pir.begin(13, 12);
}

void loop() {
  if(pir.triggered()) {
    if(pir.readData() == 0) {
      Serial.printf("Movement detected\n");
    }
  }
  delay(100);
}
~~~~

Advanced example with ESP8266 in light sleep and modified threshold:
~~~~C++
#include <DigiPIR.h>

DigiPIR pir;

void setup() {
  [...]
  wifi_set_sleep_type(LIGHT_SLEEP_T);
  wifi_enable_gpio_wakeup(12, GPIO_PIN_INTR_HILEVEL);
  [...]

  // Lower the PIR threshold to 32 (from default 128)
  pir.configure(32);
  // Start PIR sensor
  // NOTE: handling must be disabled because we need the above interrupt handling
  pir.begin(13, 12, false);
}

void loop() {
  if(pir.triggered()) {
    int adc_value = 0; 
    if(pir.readData(&adc_value) == 0) {
      Serial.printf("Movement detected: %d\n", adc_value);
    }
  }
  delay(100);
}
~~~~
