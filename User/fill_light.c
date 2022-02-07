#define FILL_LIGHT_C_

#include "fill_light.h"


/**
* @brief  fill_light init
* @param  
* @param  None
* @param  None
* @retval None
* @example 
**/
void fill_light_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(FILL_LIGHT_CLK, ENABLE);	 //Clock
	
	GPIO_InitStructure.GPIO_Pin = FILL_LIGHT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(FILL_LIGHT_PORT, &GPIO_InitStructure);
}


/**
* @brief  fill_light_set
* @param  value 1:ON; 0:OFF
* @param  None
* @param  None
* @retval None
* @example 
**/
void fill_light_set(int value)
{
	switch(value)
	{
		case 1:
			GPIO_SetBits(FILL_LIGHT_PORT,FILL_LIGHT_PIN);
		break;
		case 0:
			GPIO_ResetBits(FILL_LIGHT_PORT,FILL_LIGHT_PIN);
		break;
		case -1:
			if(GPIO_ReadInputDataBit(FILL_LIGHT_PORT,FILL_LIGHT_PIN))
				GPIO_ResetBits(FILL_LIGHT_PORT,FILL_LIGHT_PIN);
			else
				GPIO_SetBits(FILL_LIGHT_PORT,FILL_LIGHT_PIN);
		break;
	}
}


/**
* @brief  fill_light_get
* @param  None
* @param  None
* @param  None
* @retval value 1:ON; 0:OFF
* @example 
**/
char fill_light_get(void)
{
	return GPIO_ReadInputDataBit(FILL_LIGHT_PORT,FILL_LIGHT_PIN);
}


