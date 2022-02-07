#ifndef ADC_H_
#define ADC_H_

#ifndef ADC_Cx_//如果没有定义
#define ADC_Cx_ extern
#else
#define ADC_Cx_
#endif

#include "stm32f10x.h"

void AdcInit(void);
u16 Get_Adc(u8 ch);
u16 Get_Adc_Average(u8 ch,u8 times);

#endif

