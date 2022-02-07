/**
  ******************************************************************************
  * @file    usart.c
  * @author  fengwu yang
  * @version V1.0.0
  * @date    2019/10/12
  * @brief   串口配置函数
				
  ******************************************************************************
  */
#define USART_C_
#include <stdio.h>
#include "usart.h"
#include "BufferManage.h"
#include "main.h"
#include "cString.h"

#include "wifi.h"
/****************************串口1****************************/
unsigned char Usart1SendBuff[Usart1SendBufflen];  //串口发送数据缓存
u8 Usart1SendBits=0;//串口提取环形队列1个字节

unsigned char *Usart1SendData;//串口中断发送数据的地址
u32   Usart1SendDataCnt=0;//串口中断发送数据的个数


/****************************串口2****************************/
unsigned char Usart2ReadBuff[Usart2ReadBuffLen]={0};  //接收数据缓存
u32  Usart2ReadCnt = 0;//串口接收到的数据个数


unsigned char Usart2SendBuff[Usart2SendBufflen];  //串口发送数据缓存
u8 Usart2SendBits=0;//串口提取环形队列1个字节

unsigned char *Usart2SendData;//串口中断发送数据的地址
u32   Usart2SendDataCnt=0;//串口中断发送数据的个数


/*用于空闲时间检测*/
u32 Usart2ReadCntCopy=0;//串口接收到的数据个数
u8  Usart2IdleCnt=0;//空闲计数



/**
* @brief   串口初始化
* @param   bound  波特率
* @param   None
* @param   None
* @retval  None
* @warning None
* @example 
**/
void UartInit(u32 bound1,u32 bound2,u32 bound3){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	rbCreate(&rb_t_usart1_send,Usart1SendBuff,Usart1SendBufflen);//创建环形队列(用于发送数据)
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
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
	
	//USART1_TX   GPIOA.2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //USART1_RX	  GPIOA.3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
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
	
	USART_InitStructure.USART_BaudRate = bound2;//串口波特率
	USART_Init(USART2, &USART_InitStructure); //初始化串口2
	
	
	/*串口--1*/
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
  /*串口--2*/
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
	
  USART_Cmd(USART1, ENABLE);                    //使能串口
	USART_Cmd(USART2, ENABLE);                    //使能串口
}


/**
* @brief  串口中断发送数据
* @param  c:数据的首地址  cnt:发送的数据个数
* @param  None
* @param  None
* @retval None
* @example 
**/
void UsartOutStrIT(USART_TypeDef* USARTx, unsigned char *c, uint32_t cnt)
{
	if(USARTx == USART1){
		PutData(&rb_t_usart1_send,c,cnt);
		USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);
	}
	else if(USARTx == USART2){
		Usart2SendData=c;
		Usart2SendDataCnt=cnt;
		USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);
	}
}


/**
* @brief  串口发送字符串数据
* @param  *c:发送的数据指针  cnt:数据个数
* @param  None
* @param  None
* @retval None
* @example 
**/
void UsartOutStr(USART_TypeDef* USARTx, unsigned char *c,uint32_t cnt)
{
	while(cnt--)
	{
		USART_SendData(USARTx, *c++);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET );
	}
}


/**
* @brief  串口发送一个字节
* @param  *c:发送的数据指针  cnt:数据个数
* @param  None
* @param  None
* @retval None
* @example 
**/
void UsartOutChar(USART_TypeDef* USARTx, unsigned char c)
{
	USART_SendData(USARTx,c);
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET );
}


/**
* @brief  串口2自定义空闲中断检测(放到1ms定时器)
* @param  IdleCnt: 空闲时间
* @param  None
* @param  None
* @retval None
* @example 
**/
void Usart2IdleLoop(int IdleCnt)
{
	if(Usart2ReadCnt!=0)//串口接收到数据
	{
		if(Usart2ReadCntCopy != Usart2ReadCnt)
		{
			Usart2ReadCntCopy = Usart2ReadCnt;
			Usart2IdleCnt = 0;
		}
		else
		{
			Usart2IdleCnt++;
			if(Usart2IdleCnt > IdleCnt)//到达空闲时间
			{
				Usart2IdleCnt = 0;
				Usart2ReadCnt=0;
				
//				PutData(&rb_t_usart1_send,"\r\n\r\nUsart2Read:\r\n",17);
//				HexToStr((char*)Usart2ReadBuff,(char*)MainBuffer,Usart2ReadCntCopy);
//				PutData(&rb_t_usart1_send,MainBuffer,Usart2ReadCntCopy*2);
//				USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
				
				uart_receive_buff_input(Usart2ReadBuff,Usart2ReadCntCopy);
			}
		}
	}
}


void USART1_IRQHandler(void)//串口中断服务程序
{
	u8 Res;
	int value;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		Res =USART_ReceiveData(USART1);	//读取接收到的数据
	} 
	
	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
  {
    if(rbCanRead(&rb_t_usart1_send)>0)//如果里面的数据个数大于0
    {
			rbRead(&rb_t_usart1_send,&Usart1SendBits,1);
			USART_SendData(USART1, Usart1SendBits);
    }
    else
    {
      //发送字节结束			
      USART_ClearITPendingBit(USART1,USART_IT_TXE);
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
      USART_ITConfig(USART1, USART_IT_TC, ENABLE);
    }
  }
	//发送完成
  if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
  {
    USART_ClearITPendingBit(USART1,USART_IT_TC);
    USART_ITConfig(USART1, USART_IT_TC, DISABLE);
  }
} 


void USART2_IRQHandler(void)//串口中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		Res =USART_ReceiveData(USART2);	//读取接收到的数据
		
//		uart_receive_input(Res);
		
		Usart2ReadBuff[Usart2ReadCnt] = Res;	//接收的数据存入数组
		Usart2ReadCnt++;
		if(Usart2ReadCnt > Usart2ReadBuffLen -10)//防止数组溢出
		{
			Usart2ReadCnt = 0;
		}
	}
}


/**
* @brief  使用microLib的方法,使用printf
* @warningg 勾选 microLib
* @param  None
* @param  None
* @param  None
* @retval None
* @example 
**/

int fputc(int ch, FILE *f)
{
	static char flage=0;
	uint8_t data = ch;
	if(!flage)
	{
	  flage=1;
//		USART_ClearITPendingBit(USART1,USART_IT_TC);//解决printf丢失第一个字节的问题
	}
//	PutData(&rb_t_usart1_send,&data,1);
//	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  return ch;
}

