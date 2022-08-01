#ifndef _THERMISTOR_H_
#define _THERMISTOR_H_

#include "main.h"

#define RESISTOR2_VALUE     10000  // Value of the 2nd resistance of the voltage divider

// Constants specific to each thermistor (see the Steinhart–Hart equation)
#define BETA                3950
#define ROOM_TEMP           298.15 // 25°C In K
#define RT_AT_ROOM_TEMP     10000  // Resistance at 25°C
#define VOLTAGE_PULLUP      3.3   // V pull-up
#define VCC                 3.3  // VCC supply
#define MAX_NTC_MEM  100

static double voltage;
static double temperatureK;
static double temperatureC;
static double thermistorResistor;

void ADC_Enable1 (void);
void ADC_Disable1 (void);
uint16_t ADC_GetVal (void);
void ADC_ChannelSelect (uint8_t Channel);
void ADC_Start(uint8_t Channel);
void ADC_WaitForConv (void);

#endif

