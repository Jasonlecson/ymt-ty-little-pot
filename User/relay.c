#define RELAY_C_

#include "relay.h"


/**
* @brief  relay init
* @param  
* @param  None
* @param  None
* @retval None
* @example 
**/
void RelayInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RELAY_CLK, ENABLE);	 //Clock
	
	GPIO_InitStructure.GPIO_Pin = RELAY_PIN;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(RELAY_PORT, &GPIO_InitStructure);
	RelaySet(0);
}


/**
* @brief  control  relay  ON or OFF
* @param  value 1:ON; 0:OFF
* @param  None
* @param  None
* @retval None
* @example 
**/
void RelaySet(int value)
{
	switch(value)
	{
		case 1:
			GPIO_SetBits(RELAY_PORT,RELAY_PIN);
		break;
		case 0:
			GPIO_ResetBits(RELAY_PORT,RELAY_PIN);
		break;
		case -1:
			if(GPIO_ReadInputDataBit(RELAY_PORT,RELAY_PIN))
				GPIO_ResetBits(RELAY_PORT,RELAY_PIN);
			else
				GPIO_SetBits(RELAY_PORT,RELAY_PIN);
		break;
	}
}

/**
* @brief  RelayGet
* @param  None
* @param  None
* @param  None
* @retval value 1:ON; 0:OFF
* @example 
**/
char RelayGet(void)
{
	return GPIO_ReadInputDataBit(RELAY_PORT,RELAY_PIN);
}




