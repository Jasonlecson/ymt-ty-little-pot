#include "iic.h"
#include "delay.h"
#include <math.h>
#include "main.h"

//初始化IIC
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO, ENABLE);	 //使能PA端口时钟
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	IIC_SCL=1;
	IIC_SDA=1;
}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
//	IIC_SDA=1;
//	delay_us(1);
//	IIC_SDA=0;
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(5);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(5);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
	delay_us(5);
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
//	SDA_OUT();//sda线输出
//	IIC_SCL=0;
//	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
// 	delay_us(4);
//	IIC_SCL=1; 
//	IIC_SDA=1;//发送I2C总线结束信号
//	delay_us(4);					
	SDA_OUT();
	IIC_SDA=0;
	delay_us(1);
	IIC_SCL=1;
	delay_us(5);
	IIC_SDA=1;
	delay_us(5);
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
u8 IIC_Send_Byte(u8 txd)
{                        
	u8 t;   
	u8 ucErrTime=0;
	SDA_OUT(); 	    
	IIC_SCL=0;//拉低时钟开始数据传输
	
	for(t=0;t<8;t++)
	{              
		IIC_SDA=(txd&0x80)>>7;
		txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
	}	 
	SDA_IN();//SDA设置为输入
	delay_us(10);
	IIC_SCL=1;
	
	while(READ_SDA==1 && ucErrTime<200){
		ucErrTime++;
	}
	if(READ_SDA==0 && ucErrTime<200){
		IIC_SCL=0;
		delay_us(2);
		return 1;
	}else{
		IIC_SCL=0;
		delay_us(2);
		return 0;
	}
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
	for(i=0;i<8;i++ )
	{
		IIC_SCL=0; 
		delay_us(2);
		IIC_SCL=1;
		receive<<=1;
		if(READ_SDA)receive++;   
		delay_us(1); 
	}					 
	if (!ack)
			IIC_NAck();//发送nACK
	else
			IIC_Ack(); //发送ACK   
	return receive;
}



int iic_read_byte(char addr){
	u8 ack=0;
	u8 data;
	IIC_Start();
	ack = IIC_Send_Byte(0x94);//我要写给你东西
	delay_us(2); 
	//需要器件应答
	if(!ack) return -1;
	
	ack = IIC_Send_Byte(addr);//给你个地址
	delay_us(2); 
	//需要器件应答
	if(!ack) return -1;
	
	IIC_Start();
	ack = IIC_Send_Byte(0x95);//我要读取上次给你地址里面的值
	delay_us(2); 
	//需要器件应答
	if(!ack) return -1;
	
	data = IIC_Read_Byte(0);//读取一个字节,不需要应答
	
	IIC_Stop();
	return(data);
}

//获取光照强度
int illumination_get(void)
{
	int value=0;
	char temp[2];
	float sensor_lignt_exponent,sensor_lignt_mantissa;
	u8 ack=0;
	int data=0;
	static int delay=0;
	
//	if(HardwareVersion == 1)
	{//BH1750
		if(delay==0){
			IIC_Start();
			ack = IIC_Send_Byte(0x46);//地址+写
			delay_us(2); 
			//需要器件应答
			if(!ack) return -1;
			IIC_Send_Byte(0x01);//启动
			IIC_Stop();

			IIC_Start();
			ack = IIC_Send_Byte(0x46);//地址+写
			delay_us(2); 
			//需要器件应答
			if(!ack) return -2;
			IIC_Send_Byte(0x10);
			IIC_Stop();
		}
		
		delay = delay+1;
		
		if(delay>=150){
			delay=0;
			IIC_Start();
			ack = IIC_Send_Byte(0x47);
			delay_us(2); 
			//需要器件应答
			if(!ack) return -3;

			data = IIC_Read_Byte(1);
			data = data<<8;
			data +=0x00ff&IIC_Read_Byte(0);

			IIC_Stop();                        //?????? 
		}else{
			data=-4;
		}
		return data; 
	}
//	else
//	{
//		value = iic_read_byte(0x03);
//		if(value == -1){
//			
//			return -1;
//		}

//		temp[0] = value;

//		value = iic_read_byte(0x04);
//		if(value == -1){
//			return -1;
//		}

//		temp[1] = value;

//		sensor_lignt_exponent = (temp[0] & 0xF0) >> 4;
//		sensor_lignt_mantissa = ((temp[0] & 0x0F) << 4) | (temp[1] &0x0F); 
//		value = sensor_lignt_mantissa * pow(2, sensor_lignt_exponent)  * 0.045;

//		return value;
//	}
	
	
//	IIC_Start();
//	ack = IIC_Send_Byte(0x94);//我要写给你东西
//	delay_us(2); 
//	//需要器件应答
//	if(!ack) return -1;
//	
//	ack = IIC_Send_Byte(addr);//给你个地址
//	delay_us(2); 
//	//需要器件应答
//	if(!ack) return -1;
//	
//	IIC_Start();
//	ack = IIC_Send_Byte(0x95);//我要读取上次给你地址里面的值
//	delay_us(2); 
//	//需要器件应答
//	if(!ack) return -1;
//	
//	data = IIC_Read_Byte(0);//读取一个字节,不需要应答
//	
//	IIC_Stop();
//	return(data);
}

