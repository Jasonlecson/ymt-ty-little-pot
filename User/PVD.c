#define _PVD_C_
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "PVD.h"
#include "flash_helper.h"

/**
* @brief  ≥ı ºªØPVD
* @param  PWR_PVDLevel ºÏ≤‚µÁ—π
* @param  None
* @param  None
* @retval None
* @example 
**/
void PVDInit(uint32_t PWR_PVDLevel)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	EXTI_DeInit();
	EXTI_StructInit(&EXTI_InitStructure);
	EXTI_InitStructure.EXTI_Line = EXTI_Line16;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	PWR_PVDLevelConfig(PWR_PVDLevel);
	PWR_PVDCmd(ENABLE);
}



void PVD_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line16);//clear bit;

	if(PWR_GetFlagStatus(PWR_FLAG_PVDO))
	{
		printf("1111111111111111111");
	}      
}


