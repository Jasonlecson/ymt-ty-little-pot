/****************************************Copyright (c)*************************
**                               版权所有 (C), 2015-2020, 涂鸦科技
**
**                                 http://www.tuya.com
**
**--------------文件信息-------------------------------------------------------
**文   件   名: main.c
**描        述: 主函数
**使 用 说 明 : 无
**
**
**--------------当前版本修订---------------------------------------------------

** 版  本: v2.0
** 日　期: 2016年3月29日
** 描　述: 1:优化代码结构
**
**-----------------------------------------------------------------------------
******************************************************************************/
#define MAIN_GLOBAL
#include "include.h"

uint8_t value[1024];

/**
* @brief   串口初始化
* @param   bound  波特率
* @param   None
* @param   None
* @retval  None
* @warning None
* @example 
**/
void UartInit(u32 bound1){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_USART1|RCC_APB2Periph_AFIO , ENABLE);
	
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //USART 初始化设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	
	USART_InitStructure.USART_BaudRate = bound1;//串口波特率
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	/*串口--1*/
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

  USART_Cmd(USART1, ENABLE);                    //使能串口
}

/**
* @brief  串口发送字符串数据
* @param  *c:发送的数据指针  cnt:数据个数
* @param  None
* @param  None
* @retval None
* @example 
**/
void UsartOutStr(unsigned char *c,uint32_t cnt)
{
	while(cnt--)
	{
		USART_SendData(USART1, *c++);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET );
	}
}


/*****************************************************************************
函数名称 : NVIC_Configuration
功能描述 : NVIC配置
输入参数 : 无
返回参数 : 无
使用说明 : 无
*****************************************************************************/
void NVIC_Configuration(void)
{
  NVIC_SetVectorTable(NVIC_VectTab_FLASH,0);
  
  //Configure the NVIC Preemption Priority Bits
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}
/*****************************************************************************
函数名称 : Exist_MainProgram
功能描述 : 判断程序是否存在
输入参数 : 无
返回参数 : 无
使用说明 : 无
*****************************************************************************/
uint8_t Exist_MainProgram(uint32_t *pulApp)
{
  // See if the first location is 0xfffffffff or something that does not
  // look like a stack pointer, or if the second location is 0xffffffff or
  // something that does not look like a reset vector.
  //
  if((pulApp[0] == 0xffffffff) || ((pulApp[0] & 0xfff00000) != 0x20000000) ||
     (pulApp[1] == 0xffffffff) || ((pulApp[1] & 0xfff00000) != 0x08000000))
  {
    return 0;
  }
  else{
    return 1; 
	}
}
/*****************************************************************************
函数名称 : RCC_Configuration
功能描述 : 时钟配置
输入参数 : 无
返回参数 : 无
使用说明 : 无
*****************************************************************************/
void RCC_Configuration(void)
{
  //启动DMA时钟 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  
  
  //Enable AFIO clocks
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  
  //使能JTAG口作为GPIO  
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}
/*****************************************************************************
函数名称 : JumpToApp
功能描述 : 程序跳转
输入参数 : 无
返回参数 : 无
使用说明 : 无
*****************************************************************************/
void JumpToApp(uint32_t app_address)            
{
  typedef  void (*pFunction)(void);
  pFunction Jump_To_Application;
  uint32_t JumpAddress;
  
  JumpAddress = *(__IO uint32_t*) (app_address + 4);
  Jump_To_Application = (pFunction) JumpAddress;
  
  __set_MSP(*(__IO uint32_t*) app_address);
  Jump_To_Application();
}

/*****************************************************************************
函数名称 : update_firmware
功能描述 : 固件升级
输入参数 : 无
返回参数 : 无
使用说明 : 无
*****************************************************************************/
uint8_t update_firmware(void)
{
  uint32_t src,obj;
  uint8_t t;
  uint32_t offset;
  
  src = FIREWARE_ADDR_H;
  obj = FIREWARE_ADDR_L;
  
  for(t = 0; t < FLASH_USER_SIZE; t ++)
  {
    offset = t * 1024;
    if(Read_Flash(value,src + offset,1024) == ERROR)
		{
      return ERROR;
    }
		
    if(Earse_Flash(obj + offset) == ERROR)
    {
      return ERROR;
    }
    
    if(Write_Flash(obj + offset,value,1024) == ERROR)
    {
      return ERROR;
    }
  }
	return SUCCESS;
}
/*****************************************************************************
函数名称 : Reset
功能描述 : 系统复位
输入参数 : 无
返回参数 : 无
使用说明 : 无
*****************************************************************************/
void Reset(void)
{
  NVIC_SystemReset();
}


//获取设备的版本
u16 IAPGetHardwareVersion(void)
{
	u16 ReadDat = 0;
  STMFLASH_Read(FLASH_UPDATE_VERSION2_ADDR,&ReadDat,1);
	return ReadDat == 65535 ? 0 : ReadDat;
}

void IAPSetHardwareVersion(u16 data)
{
	u16 ReadDat = data;
  STMFLASH_Write(FLASH_UPDATE_VERSION2_ADDR,&ReadDat,1);
}


/*****************************************************************************
函数名称 : main
功能描述 : 主函数
输入参数 : 无
返回参数 : 无
使用说明 : 无
*****************************************************************************/
void main (void)
{
	u16  HardwareVersion = 1;
  /*
  //解锁(必须运行在RAM状态下)
  FLASH_Unlock();
  FLASH_ReadOutProtection(DISABLE); 
  */
  uint32_t magic_code;
  uint32_t app_address;
  
  RCC_Configuration();
  
  NVIC_Configuration();
  UartInit(115200);
	
	if(IAPGetHardwareVersion() != HardwareVersion)
	{
		IAPSetHardwareVersion(HardwareVersion);
	}
	
	
  //获取升级状态
  Read_Flash((unsigned char *)&magic_code,FLASH_UPDATE_FLAGE_ADDR,4);
   
	UsartOutStr("update_firmware...\r\n",20);
	//有升级的新程序
  if(magic_code == FIREWARE_UPDATE_FLAG)
  {
		//拷贝升级程序
    if(update_firmware() != SUCCESS)
    {
			UsartOutStr("update_firmware_err\r\n",21);
      Reset();
    }
  }  
  
  //判断程序是否存在
  app_address = FIREWARE_ADDR_L;
	
	UsartOutStr("check_firmware....\r\n",20);
  if(Exist_MainProgram((uint32_t *)app_address) == 1)
  {
		UsartOutStr("load_user_program\r\n",19);
    JumpToApp(app_address); 
  }

	UsartOutStr("update_firmware_err\r\n",21);
	
  while(1)
  {
		
  }
}

