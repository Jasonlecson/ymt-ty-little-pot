#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include <stm32f10x.h>


//用户根据自己的需要设置
#define STM32_FLASH_SIZE 128 	 		//所选STM32的FLASH容量大小(单位为K)
#define STM32_FLASH_WREN 1        //使能FLASH写入(0，不是能;1，使能)
#define FLASH_IAP_SIZE 20         //BootLoader所用程序大小(KB)
#define FLASH_DATA_SIZE 6         //存储用户数据所用程序大小(KB)
//用户程序大小 = (总FLASH容量 - 存储用户数据所用程序大小 - IAP所用程序大小)/2
#define FLASH_USER_SIZE  ((STM32_FLASH_SIZE - FLASH_DATA_SIZE - FLASH_IAP_SIZE)/2)  //用户程序大小(KB)

#define STM32_FLASH_BASE 0x8000000 	//STM32 FLASH的起始地址

//程序运行地址
#define FIREWARE_ADDR_L  (STM32_FLASH_BASE+1024*FLASH_IAP_SIZE)
//程序备份地址
#define FIREWARE_ADDR_H  (STM32_FLASH_BASE+1024*(FLASH_USER_SIZE+FLASH_IAP_SIZE))
//存储用户数据的地址 :Flash最后的 FLASH_DATA_SIZE KB  作为数据存储的地址 
#define FLASH_DATA_ADDR  (STM32_FLASH_BASE + 1024*(STM32_FLASH_SIZE - (FLASH_DATA_SIZE)) + 10) 


//数据存储地址(默认存储到最后4页)
#define FLASH_DATA_ADDR_1  (STM32_FLASH_BASE+1024*124)
#define FLASH_DATA_ADDR_2  (STM32_FLASH_BASE+1024*125)
#define FLASH_DATA_ADDR_3  (STM32_FLASH_BASE+1024*126)
#define FLASH_DATA_ADDR_4  (STM32_FLASH_BASE+1024*127)


#define FLASH_DATA_ADDR_OFFSET 0

//#define FLASH_RUNUSER_ADDR (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+0)       //存储运行的哪一份程序信息 0:没有可以运行的程序;1:运行的第一份程序;2:运行的第二份程序
#define FLASH_UPDATE_FLAGE_ADDR (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+2)  //存储是不是有更新程序
//#define FLASH_UPDATE_STATUS_ADDR (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+4) //存储更新的状态
//#define FLASH_UPDATE_VERSION1_ADDR (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+6) //存储云端发来的版本号,版本号最长20个字符
#define FLASH_UPDATE_VERSION2_ADDR (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+26) //存储设备当前的版本号,版本号最长20个字符
//后期的存储请从  FLASH_DATA_ADDR+46  开始

#define FLASH_USER_WATER_MODE_ADDR  (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+46)  //浇水模式 46,47
#define FLASH_USER_WATER_COUNT_ADDR (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+48)  //浇水次数 48,49
#define FLASH_USER_BOTANY_ADDR (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+50) //植物类型 50 51 52 53 54 55 56 57

#define FLASH_USER_TEMP_ADDR (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+54+4) //阈值:温度  54-55(L) 56-57(H)
#define FLASH_USER_HUMI_ADDR (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+58+4) //阈值:湿度  58-59(L) 60-61(H)
#define FLASH_USER_ILL_ADDR  (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+62+4) //阈值:光照  62-63(L) 64-65(H)
#define FLASH_USER_FERT_ADDR (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+66+4) //阈值:肥力  66-67(L) 68-69(H)

#define FLASH_USER_DATA_ADDR (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+70+4) //种植时间:70-77
#define FLASH_USER_LOVE_ADDR (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+78+4) //爱心值:78-79
#define FLASH_USER_HARVEST_ADDR (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+80+4) //累计收获:80-81
//#define FLASH_USER_MATRIX_ADDR  (FLASH_DATA_ADDR+82) //基质:82-83 废弃

#define FLASH_USER_ONEDATA_ADDR (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+84+4) //首次种植时间:84-91
#define FLASH_USER_MATRIX_ADDR (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+92+4) //基质时间:92-99
#define FLASH_USER_NURTURE_ADDR (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+102+4) //培育的天数 102-103

/*使用 flash_helper 存储数据*/
#define FLASH_USE_FLASH_HELPER (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+104+4) //104 105
/*是否上报光照统计事件*/
#define FLASH_USE_FLASH_ILLEVENT (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+106+4) //106 107
/*光照统计报警值*/
#define FLASH_USE_FLASH_LIGHTALARM (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+108+4) //108 109 110 111
/*控制在休息时间段是否报警*/
#define FLASH_USE_FLASH_REPORTEVENT (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+112+4) //112 113
/*控制是否自动打开补光灯*/
#define FLASH_USE_FLASH_FILLLIGHT (FLASH_DATA_ADDR+FLASH_DATA_ADDR_OFFSET+114+4) //114 115






u16 STMFLASH_ReadHalfWord(u32 faddr);		  //读出半字  
void STMFLASH_WriteLenByte(u32 WriteAddr,u32 DataToWrite,u16 Len);	//指定地址开始写入指定长度的数据
u32 STMFLASH_ReadLenByte(u32 ReadAddr,u16 Len);						//指定地址开始读取指定长度数据
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);		//从指定地址开始写入指定长度的数据
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);   		//从指定地址开始读出指定长度的数据

char WriteFlashHalfWord(uint32_t WriteAddress,u16 data);
char FlashErasePage(uint32_t EraseAddress,u16 PageCnt);	
char FlashErasePages(uint32_t EraseAddress,u16 PageCnt);
void WriteFlashHalfWords(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);


/*****************************************************************************
函数名称 : Write_Flash
功能描述 : 对Flash进行字节编程
输入参数 : Addr_Sour:写入的源数据
           Addr_Obj:FLASH的目标起始地址
           Length:写入的字节数
返回参数 : SUCCESS:写入成功
            ERROR:写入失败
使用说明 : 无
*****************************************************************************/
uint8_t Write_Flash(uint32_t Addr_Obj,uint8_t *Addr_Sour,uint16_t Length);

/*****************************************************************************
函数名称 : Earse_Flash
功能描述 : 对Flash擦除
输入参数 : Addr_Obj:FLASH的目标起始地址
返回参数 : SUCCESS:擦除成功
            ERROR:擦除失败
使用说明 : 无
*****************************************************************************/
uint8_t Earse_Flash(uint32_t Addr_Obj);

/*****************************************************************************
函数名称 : Read_Flash
功能描述 : 读取FLASH的数据
输入参数 : pulData:源数据
           ulAddress:FLASH的起始地址
           Length:读取的字节数
返回参数 : true:读取成功
           false:读取失败
*****************************************************************************/
uint8_t Read_Flash(uint8_t *pulData,uint32_t ulAddress,uint32_t Length);

#endif

















