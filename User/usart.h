#ifndef __USART_H
#define __USART_H
#include <stm32f10x.h>
#ifndef USART_C_//如果没有定义
#define USART_Ex_ extern
#else
#define USART_Ex_
#endif


#include "LoopList.h"

/****************************串口1****************************/
USART_Ex_ rb_t rb_t_usart1_send;//串口1发送缓存环形队列
#define Usart1SendBufflen 2048  //串口1发送数据缓存大小
USART_Ex_ unsigned char Usart1SendBuff[Usart1SendBufflen];  //串口1发送数据缓存


/****************************串口2****************************/
#define Usart2ReadBuffLen 1460  //串口接收数据数组大小
USART_Ex_ unsigned char Usart2ReadBuff[Usart2ReadBuffLen];  //接收数据缓存
USART_Ex_ u32  Usart2ReadCnt;//串口接收到的数据个数


USART_Ex_ rb_t rb_t_usart2_send;//串口发送缓存环形队列
#define Usart2SendBufflen 1024  //串口发送数据缓存大小
USART_Ex_ unsigned char Usart2SendBuff[Usart2SendBufflen];  //串口发送数据缓存




void UartInit(u32 bound1,u32 bound2,u32 bound3);//串口初始化
void UsartOutStrIT(USART_TypeDef* USARTx, unsigned char *c,uint32_t cnt);
void UsartOutStr(USART_TypeDef* USARTx, unsigned char *c,uint32_t cnt);//串口发送字符串数据
void UsartOutChar(USART_TypeDef* USARTx, unsigned char c);//串口发送一个字节
void Usart2IdleLoop(int IdleCnt);
#endif


