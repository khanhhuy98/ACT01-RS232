#include "thermistor.h"

void ADC_Enable1 (void)
{
	/************** STEPS TO FOLLOW *****************
	1. Enable the ADC by setting ADON bit in CR2
	2. Wait for ADC to stabilize (approx 10us) 
	************************************************/
	ADC1->CR |= 1<<0;   // ADON =1 enable ADC1
	
	uint32_t delay = 10000;
	while (delay--);
}
void ADC_Disable1 (void)
{
	/************** STEPS TO FOLLOW *****************
	1. Disable the ADC by Clearing ADON bit in CR2
	************************************************/	
	ADC1->CR &= ~(1<<0);  // Disable ADC
}
uint16_t ADC_GetVal (void)
{
	return ADC1->DR;  // Read the Data Register
}
void ADC_WaitForConv (void)
{
	/*************************************************
	EOC Flag will be set, once the conversion is finished
	*************************************************/
	while (!(ADC1->ISR & (1<<2)));  // wait for EOC flag to set
}
void ADC_ChannelSelect (uint8_t Channel)
{
	ADC1->CHSELR |= 1 << Channel;
}
void ADC_Start(uint8_t Channel)
{
	ADC1->CR |= ADC_CR_ADSTART;
//	ADC1->CHSELR = 0;
//	ADC1->CHSELR = (Channel <<0);
	ADC1->CHSELR |= 1 << Channel;
}

