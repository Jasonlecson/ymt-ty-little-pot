#define BUZZER_C_

#include "buzzer.h"


/**
* @brief  buzzer init
* @param  
* @param  None
* @param  None
* @retval None
* @example 
**/
void BuzzerInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(BUZZER_CLK, ENABLE);	 //Clock
	
	GPIO_InitStructure.GPIO_Pin = BUZZER_PIN;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BUZZER_PORT, &GPIO_InitStructure);
}


/**
* @brief  control  buzzer  ON or OFF
* @param  value 1:ON; 0:OFF
* @param  None
* @param  None
* @retval None
* @example 
**/
void BuzzerSet(int value)
{
	switch(value)
	{
		case 1:
			GPIO_SetBits(BUZZER_PORT,BUZZER_PIN);
		break;
		case 0:
			GPIO_ResetBits(BUZZER_PORT,BUZZER_PIN);
		break;
	}
}




