#include "ADC1Dev.h"
#include "_global.h"

#define COL_MA_NUM                  10

// ToDO: 
// 1. Change to Interupt mode
// 2. Average values and discard extreme samples

void ADC_Config()
{
  CLK_PeripheralClockConfig(CLK_Peripheral_ADC1 , ENABLE);    // ʹ��ADCʱ��  
  ADC_Init(ADC1 ,
            ADC_ConversionMode_Single ,     // ����ADC1�ǵ��β���
            ADC_Resolution_12Bit ,          // ����ADC1Ϊ12BIT��ת������ 
            ADC_Prescaler_2                 // ����ADC1��ʱ��Ϊ2��Ƶ
           );
  ADC_Cmd(ADC1 , ENABLE);                   // ʹ��ADC1
}

void enable_eq1()
{
  ADC_ChannelCmd(ADC1, ADC_Channel_22, ENABLE);     // PD0, eq1
  ADC_ChannelCmd(ADC1, ADC_Channel_21, DISABLE);    // PD1, eq2
}

void enable_eq2()
{
  ADC_ChannelCmd(ADC1, ADC_Channel_22, DISABLE);    // PD0, eq1
  ADC_ChannelCmd(ADC1, ADC_Channel_21, ENABLE);     // PD1, eq2
}

uint16_t adc_value = 0;
uint16_t adc_read()
{ 
  ADC_SoftwareStartConv(ADC1);                  // ����ADC1��ʼת������
  // �ȴ�ADC��������
  while(ADC_GetFlagStatus(ADC1 , ADC_FLAG_EOC) == RESET);
  ADC_ClearFlag(ADC1 , ADC_FLAG_EOC);           // ���ת��������־  
  adc_value = ADC_GetConversionValue(ADC1);     // ����ADCת�����  
  return adc_value;
}

uint16_t adc_readMAValue(const uint8_t _count)
{
  uint32_t lv_adcSum = 0;
  uint16_t lv_newData;
  for( uint8_t i = 0; i < _count + 1; i++ ) {
    lv_newData = adc_read();
    if(i != 0) { // Skip the first 1
      lv_adcSum += lv_newData;
    }
  }
  return((uint16_t)(lv_adcSum / _count + 0.5));
}

void eq_checkData(uint16_t* eq1, uint16_t* eq2)
{
  enable_eq1();
  *eq1 = adc_readMAValue(COL_MA_NUM);
    
  enable_eq2();
  *eq2 = adc_readMAValue(COL_MA_NUM);
}