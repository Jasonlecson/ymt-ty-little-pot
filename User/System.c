#include "System.h"

void NVIC_Configuration(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
}
/*复位芯片*/
void Reset_MCU(void)
{
  __disable_fault_irq();   
	NVIC_SystemReset();
}

