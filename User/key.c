/**
  ******************************************************************************
  * @file    key.c
  * @author  fengwu yang
  * @version V1.0.0
  * @date    2019/10/12
  * @brief   按键检测模板
  ******************************************************************************
  一,使用说明:
		1,把以下程序放在1ms定时器中断中
			KeyLoop();
		
		2,主循环调用使用
			while(1)
			{
				KeyWhile();
				
				//例1:检测到按键按下执行某段代码
				if(KeyValueTemp[3] == 1)
				{
					KeyValueTemp[3] = 0;//清零以后,只有按键松开再按下的时候才会进入
					//执行的代码
				}

				//例2:检测到按键按下,然后根据按下时间执行某段代码
				if(KeyValueTemp[3] == 1)
				{
					if(KeyValueTemp[5]>=3000)//按下时间大于3S
					{
						KeyValueTemp[3] = 0;//清零以后,只有按键松开再按下的时候才会进入
						//执行的代码
					}
				}
				
				//例3:检测到按键松开执行某段代码
				if(KeyValueTemp[4] == 1)
				{
					KeyValueTemp[4] = 0;//清零以后,只有按键按下然后松开的时候才会进入
					//执行的代码
				}
				
				//例4:检测到按键松开,然后根据松开时间执行某段代码
				if(KeyValueTemp[4] == 1)
				{
					if(KeyValueTemp[6]>=3000)//松开时间大于3S
					{
						KeyValueTemp[4] = 0;//清零以后,只有按键按下再松开的时候才会进入
						//执行的代码
					}
				}
			}
  ******************************************************************************
  */

#define __KEY_C_
#include "key.h"

int KeyValueTemp[7]={0};//消抖变量 按下标记 松开标记 按下标记拷贝  松开标记拷贝 按下时间(Ms) 松开时间(Ms)
	


/**
* @brief  key init
* @param  
* @param  None
* @param  None
* @retval None
* @example 
**/
void KeyInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(KEY_CLK, ENABLE);	 //Clock
	
	GPIO_InitStructure.GPIO_Pin = KEY_PIN;//pin		

	#if KeyDown == 0 //0 is key down
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	#else
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	#endif
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(KEY_PORT, &GPIO_InitStructure);
}


/**
* @brief  按键处理
* @param  
* @param  None
* @param  None
* @retval None
* @example 
**/
void KeyWhile(void)
{
  if(KeyPinIn == KeyDown)
	{
		if(KeyValueTemp[1] == 0)
		{
			KeyValueTemp[0]++;
			if(KeyValueTemp[0]>=100)//按下以后进入
			{
				KeyValueTemp[0] = 0;//Cnt
				KeyValueTemp[2] = 0;//UP
				KeyValueTemp[1] = 1;//Down
				KeyValueTemp[3] = 1;//DownCopy
				KeyValueTemp[4] = 0;//UpCopy
			}
		}
	}
	else
	{
//		if(KeyValueTemp[1]==1)
		{
			KeyValueTemp[0]++;
			if(KeyValueTemp[0]>200)
			{
				KeyValueTemp[0]=0;
				if(KeyValueTemp[1]==1)//按下后松开后进入
				{
					KeyValueTemp[2] = 1;//UP
					KeyValueTemp[1] = 0;//Down
					KeyValueTemp[3] = 0;//DownCopy
					KeyValueTemp[4] = 1;//UpCopy
				}
			}
		}
	}
}



/**
* @brief  place in  1ms timer
* @param  
* @param  None
* @param  None
* @retval None
* @example 
**/
void KeyLoop(void)
{
	if(KeyValueTemp[1])
		KeyValueTemp[5]++;
	else
		KeyValueTemp[5]=0;

	if(KeyValueTemp[2])
		KeyValueTemp[6]++;
	else
		KeyValueTemp[6]=0;
}

