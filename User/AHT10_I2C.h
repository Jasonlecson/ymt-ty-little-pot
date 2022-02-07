#ifndef __AHT10_I2C_H
#define __AHT10_I2C_H
#include <stm32f10x.h>
#include "delay.h"
#include "System.h"

//IO��������	
#define AHT10_PORT  GPIOB    //
#define AHT10_SCL_PIN     GPIO_Pin_6
#define AHT10_SDA_PIN     GPIO_Pin_7
#define AHT10_IIC_SCL    PBout(6) //SCL
#define AHT10_IIC_SDA    PBout(7) //SDA	 
#define AHT10_READ_SDA   PBin(7)  //����SDA

#define AHT10_SDA_IN()  {AHT10_PORT->CRL&=0X0FFFFFFF;AHT10_PORT->CRL|=(u32)8<<28;}
#define AHT10_SDA_OUT() {AHT10_PORT->CRL&=0X0FFFFFFF;AHT10_PORT->CRL|=(u32)3<<28;}


void AHT10_Init(void);                //��ʼ��IIC��IO��	
void  read_AHT10_once(void);          //��ȡ��ʪ�ȣ�����ѭ���У�
extern volatile	int H1;   //ʪ�ȣ�552Ϊ55.2%
extern volatile int T1;   //�¶ȣ�255Ϊ25.5��

/*	printf("\r\n");
		printf("�¶�:%d%d.%d",T1/100,(T1/10)%10,T1%10);
		printf("ʪ��:%d%d.%d",H1/100,(H1/10)%10,H1%10);
		printf("\r\n");
*/
#endif
