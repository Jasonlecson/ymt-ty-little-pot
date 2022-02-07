#ifndef FILL_LIGHT_Cc_
#define FILL_LIGHT_Cc_

#ifndef FILL_LIGHT_C_//如果没有定义
#define FILL_LIGHT_Cx_ extern
#else
#define FILL_LIGHT_Cx_
#endif

#include "stm32f10x.h"

/*config pin*/
#define FILL_LIGHT_CLK    RCC_APB2Periph_GPIOB
#define FILL_LIGHT_PORT   GPIOB
#define FILL_LIGHT_PIN    GPIO_Pin_5

void fill_light_init(void);
void fill_light_set(int value);
char fill_light_get(void);

#endif
