#ifndef H_KWBATTERYMONITOR
#define H_KWBATTERYMONITOR

#include <Arduino.h>
#include <esp_adc_cal.h>

#define MAXBATT \
  4200  // The default Lipo is 4200mv when the battery is fully charged.
#define LIGHT_SLEEP_VOLTAGE 3750  // Point where start light sleep
#define MINBATT \
  3200  // The default Lipo is 3200mv when the battery is empty...this WILL be
        // low on the 3.3v rail specs!!!

#define VOLTAGE_DIVIDER \
  3.20  // Lora has 220k/100k voltage divider so need to reverse that reduction
        // via (220k+100k)/100k on vbat GPIO37 or ADC1_1 (early revs were GPIO13
        // or ADC2_4 but do NOT use with WiFi.begin())
#define DEFAULT_VREF 1100  // Default VREF use if no e-fuse calibration
#define VBATT_SAMPLE 500   // Battery sample rate in ms
#define VBATT_SMOOTH 50    // Number of averages in sample
#define ADC_READ_STABILIZE \
  5  // in ms (delay from GPIO control and ADC connections times)
#define LO_BATT_SLEEP_TIME \
  10 * 60 * 1000 * 1000  // How long when low batt to stay in sleep (us)
#define HELTEC_V2_1 \
  1  // Set this to switch between GPIO13(V2.0) and GPIO37(V2.1) for VBatt ADC.
#define VBATT_GPIO \
  21  // Heltec GPIO to toggle VBatt read connection ... WARNING!!! This also
      // connects VEXT to VCC=3.3v so be careful what is on header.  Also, take
      // care NOT to have ADC read connection in OPEN DRAIN when GPIO goes HIGH
#define __DEBUG 0  // DEBUG Serial output

void     initialiseBatteryMonitor();
uint16_t Sample();
uint16_t ReadVBatt();

#endif