#ifndef __TIMER_H
#define __TIMER_H

#ifndef _TIME_C_//如果没有定义
#define _TIME_Ex_ extern
#else
#define _TIME_Ex_
#endif

#include "stm32f10x.h"

#define TIMER_COMPARE_SEC  1000 //1S
#define TIMER_COMPARE_3SEC  3000 //3S
#define TIMER_COMPARE_60SEC  60000 //60S
#define TIMER_COMPARE_3MIN  180000 //3 min

#define TIMER_COMPARE_20MIN  1200000 //20 min
//#define TIMER_COMPARE_20MIN  60000 //20 min

#define TIMER_COMPARE_HALF_HOUR  1800000 //30 min
#define TIMER_COMPARE_1HOUR  3600000 //1 hour
#define TIMER_COMPARE_3HOUR  10800000 //3 hour

_TIME_Ex_ int timer2Value;
_TIME_Ex_ int timer2Value1;
_TIME_Ex_ int timer2Value2;//每隔1S把时间戳转换为时间
_TIME_Ex_ int timer2Value3;//每隔1S获取一次网络状态
_TIME_Ex_ int timer2_watering;//控制浇水
_TIME_Ex_ u32 timer2_auto_watering;//控制自动浇水
_TIME_Ex_ int timer2_temp_humi_value;

_TIME_Ex_ int timer2_water_stage;//控制采集水位传感器时间

_TIME_Ex_ u32 timer2_event_appear;//事件上报
_TIME_Ex_ int timer2_appear1;//控制上报频率
_TIME_Ex_ int timer2_appear2;//控制上报频率
_TIME_Ex_ int timer2_appear3;//控制上报频率


_TIME_Ex_ int TimerDataValue1;//定时获取时间戳


_TIME_Ex_ void Timer2Init(void);
#endif

