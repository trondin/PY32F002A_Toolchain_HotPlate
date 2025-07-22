#ifndef NTC_LUT_H
#define NTC_LUT_H
#include "py32f030x6.h"

void ADC_Init(void); 
int16_t analog2temp(uint16_t adc);

#endif