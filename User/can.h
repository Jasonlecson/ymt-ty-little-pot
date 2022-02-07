#ifndef __CAN_H
#define __CAN_H

#ifndef _CAN_C_//如果没有定义
#define _CAN_Cx_ extern
#else
#define _CAN_Cx_
#endif

#include "stm32f10x.h"

/* Private define ------------------------------------------------------------*/
#define __CAN1_USED__
/* #define __CAN2_USED__*/   /* Please check that you device is 
                                Connectivity line when using CAN2 */
#ifdef  __CAN1_USED__
  #define CANx CAN1
#else /*__CAN2_USED__*/
  #define CANx CAN2
#endif  /* __CAN1_USED__ */

#define RCC_APBxPeriph_CAN_IO RCC_APB2Periph_GPIOA
#define CAN_RXD GPIO_Pin_11
#define CAN_TXD GPIO_Pin_12
#define CAN_IO  GPIOA


char CAN_Polling(void);

#endif
