#ifndef __MAIN_H_
#define __MAIN_H_
#include <stm32f10x.h>

#ifndef __MAIN_C_
#define __MAIN_EXT_ extern
#else
#define __MAIN_EXT_
#endif


//缓存数据使用
#define MainBufferLen 1500
__MAIN_EXT_ unsigned char MainBuffer[MainBufferLen];//缓存数据,全局通用
__MAIN_EXT_ u32  MainLen;      //全局通用变量
__MAIN_EXT_ unsigned char *MainString;    //全局通用变量
__MAIN_EXT_ u16 HardwareVersion;

__MAIN_EXT_ void TimerLoopFun(int value);

#endif
