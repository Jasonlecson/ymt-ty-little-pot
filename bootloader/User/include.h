#ifndef __INCLUDE_H__
#define __INCLUDE_H__

//==========================================================================
#include	<stdio.h>
#include	<stdlib.h>
#include	<string.h>
//#include 	<intrins.h>
#include	<ctype.h>
#include	"stm32f10x.h"

#include	"user_flash.h"


//用户根据自己的需要设置
#define STM32_FLASH_SIZE 128 	 		//所选STM32的FLASH容量大小(单位为K)
#define STM32_FLASH_WREN 1        //使能FLASH写入(0，不是能;1，使能)
#define FLASH_IAP_SIZE 20         //BootLoader所用程序大小(KB)
#define FLASH_DATA_SIZE 6         //存储用户数据所用程序大小(KB)
//用户程序大小 = (总FLASH容量 - 存储用户数据所用程序大小 - IAP所用程序大小)/2
#define FLASH_USER_SIZE  ((STM32_FLASH_SIZE - FLASH_DATA_SIZE - FLASH_IAP_SIZE)/2)  //用户程序大小(KB)

#define STM32_FLASH_BASE 0x8000000 	//STM32 FLASH的起始地址

//用户程序运行地址
#define FIREWARE_ADDR_L  (STM32_FLASH_BASE+1024*FLASH_IAP_SIZE)
//用户程序备份地址
#define FIREWARE_ADDR_H  (STM32_FLASH_BASE+1024*(FLASH_USER_SIZE+FLASH_IAP_SIZE))
//存储用户数据的地址 :Flash最后的 FLASH_DATA_SIZE KB  作为数据存储的地址 
#define FLASH_DATA_ADDR  (STM32_FLASH_BASE + 1024*(STM32_FLASH_SIZE - FLASH_DATA_SIZE) + 10) 


//#define FLASH_RUNUSER_ADDR (FLASH_DATA_ADDR+0)       //存储运行的哪一份程序信息 0:没有可以运行的程序;1:运行的第一份程序;2:运行的第二份程序
#define FLASH_UPDATE_FLAGE_ADDR (FLASH_DATA_ADDR+2)  //存储是不是有更新程序
//#define FLASH_UPDATE_STATUS_ADDR (FLASH_DATA_ADDR+4) //存储更新的状态
//#define FLASH_UPDATE_VERSION1_ADDR (FLASH_DATA_ADDR+6) //存储云端发来的版本号,版本号最长20个字符
#define FLASH_UPDATE_VERSION2_ADDR (FLASH_DATA_ADDR+26) //存储设备当前的版本号,版本号最长20个字符
//后期的存储请从  FLASH_DATA_ADDR+46  开始


//(7+1K)+28K+28K
//#define         BASIC_FLASH_ADDR                        0x08000000

//#define         OFFSET_PARA                             0x1c00                                
//#define         OFFSET_FIRMWARE_L                       0x2000
//#define         OFFSET_FIRMWARE_H                       0x9000

//#define         PARA_ADDR                               (BASIC_FLASH_ADDR + OFFSET_PARA)
//#define         FIREWARE_ADDR_L                         (BASIC_FLASH_ADDR + OFFSET_FIRMWARE_L)   //
//#define         FIREWARE_ADDR_H                         (BASIC_FLASH_ADDR + OFFSET_FIRMWARE_H)   //

#define         FIREWARE_UPDATE_FLAG                    0x55555555


#endif

