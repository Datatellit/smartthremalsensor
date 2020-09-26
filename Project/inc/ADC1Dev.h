#ifndef _ADC1_DEV_H_
#define _ADC1_DEV_H_

#include "_global.h"

void ADC_Config();

uint16_t adc_read();
void enable_eq1();
void enable_eq2();
uint16_t adc_readMAValue(const uint8_t _count);
void eq_checkData(uint16_t* eq1, uint16_t* eq2);

#endif