#ifndef WIFI_STATE_LED_Cc_
#define WIFI_STATE_LED_Cc_

#ifndef WIFI_STATE_LED_C_//如果没有定义
#define WIFI_STATE_LED_Cx_ extern
#else
#define WIFI_STATE_LED_Cx_
#endif

#include "stm32f10x.h"

/*config pin*/
#define WIFI_STATE_LEDA_PIN    GPIO_Pin_6
#define WIFI_STATE_LEDA_PORT   GPIOA
#define WIFI_STATE_LEDA_CLK    RCC_APB2Periph_GPIOA

#define WIFI_STATE_LEDB_PIN    GPIO_Pin_5
#define WIFI_STATE_LEDB_PORT   GPIOA
#define WIFI_STATE_LEDB_CLK    RCC_APB2Periph_GPIOA

#define WIFI_STATE_LEDC_PIN    GPIO_Pin_12
#define WIFI_STATE_LEDC_PORT   GPIOB
#define WIFI_STATE_LEDC_CLK    RCC_APB2Periph_GPIOB

void wifi_state_led_init(void);
void wifi_state_leda_set(int value);
char wifi_state_leda_get(void);

void wifi_state_ledb_set(int value);
char wifi_state_ledb_get(void);

void wifi_state_ledc_set(int value);
char wifi_state_ledc_get(void);

#endif
