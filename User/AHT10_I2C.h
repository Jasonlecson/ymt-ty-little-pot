#ifndef __AHT10_I2C_H
#define __AHT10_I2C_H
#include <stm32f10x.h>
#include "delay.h"
#include "System.h"

//IO操作函数	
#define AHT10_PORT  GPIOB    //
#define AHT10_SCL_PIN     GPIO_Pin_6
#define AHT10_SDA_PIN     GPIO_Pin_7
#define AHT10_IIC_SCL    PBout(6) //SCL
#define AHT10_IIC_SDA    PBout(7) //SDA	 
#define AHT10_READ_SDA   PBin(7)  //输入SDA

#define AHT10_SDA_IN()  {AHT10_PORT->CRL&=0X0FFFFFFF;AHT10_PORT->CRL|=(u32)8<<28;}
#define AHT10_SDA_OUT() {AHT10_PORT->CRL&=0X0FFFFFFF;AHT10_PORT->CRL|=(u32)3<<28;}


void AHT10_Init(void);                //初始化IIC的IO口	
void  read_AHT10_once(void);          //获取温湿度（放在循环中）
extern volatile	int H1;   //湿度，552为55.2%
extern volatile int T1;   //温度，255为25.5度

/*	printf("\r\n");
		printf("温度:%d%d.%d",T1/100,(T1/10)%10,T1%10);
		printf("湿度:%d%d.%d",H1/100,(H1/10)%10,H1%10);
		printf("\r\n");
*/
#endif
