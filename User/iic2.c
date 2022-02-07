#include "iic2.h"
#include "delay.h"


//初始化IIC
void IIC2_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO, ENABLE);	 //使能PA端口时钟
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	IIC2_SCL=1;
	IIC2_SDA=1;
}
//产生IIC起始信号
void IIC2_Start(void)
{
	SDA2_OUT();     //sda线输出
	IIC2_SDA=1;
	IIC2_SDA=0;
	IIC2_SDA=1;
	IIC2_SCL=1;
	delay_us(4);
 	IIC2_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC2_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC2_Stop(void)
{
	SDA2_OUT();//sda线输出
	IIC2_SCL=0;
	IIC2_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC2_SCL=1; 
	IIC2_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC2_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA2_IN();      //SDA设置为输入  
	IIC2_SDA=1;delay_us(1);	   
	IIC2_SCL=1;delay_us(1);	 
	while(READ2_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC2_Stop();
			return 1;
		}
	}
	IIC2_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC2_Ack(void)
{
	IIC2_SCL=0;
	SDA2_OUT();
	IIC2_SDA=0;
	delay_us(2);
	IIC2_SCL=1;
	delay_us(2);
	IIC2_SCL=0;
}
//不产生ACK应答		    
void IIC2_NAck(void)
{
	IIC2_SCL=0;
	SDA2_OUT();
	IIC2_SDA=1;
	delay_us(2);
	IIC2_SCL=1;
	delay_us(2);
	IIC2_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
u8 IIC2_Send_Byte(u8 txd)
{                        
	u8 t=0,ucErrTime=0;   
	SDA2_OUT(); 	    
	IIC2_SCL=0;//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{              
		IIC2_SDA=(txd&0x80)>>7;
		txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC2_SCL=1;
		delay_us(2); 
		IIC2_SCL=0;	
		delay_us(2);
	}	 
	SDA2_IN();//SDA设置为输入
	delay_us(2);
	IIC2_SCL=1;
	
	while(READ2_SDA==1 && ucErrTime<200){
		ucErrTime++;
	}
	if(READ2_SDA==0 && ucErrTime<200){
		IIC2_SCL=0;
		delay_us(2);
		return 1;
	}else{
		IIC2_SCL=0;
		delay_us(2);
		return 0;
	}
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC2_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA2_IN();//SDA设置为输入
	for(i=0;i<8;i++ )
	{
		IIC2_SCL=0; 
		delay_us(2);
		IIC2_SCL=1;
		receive<<=1;
		if(READ2_SDA)receive++;   
		delay_us(1); 
	}					 
	if (!ack)
			IIC2_NAck();//发送nACK
	else
			IIC2_Ack(); //发送ACK   
	return receive;
}


int iic2_read_byte(void){
	u8 ack=0;
	u8 data[4];
	IIC2_Start();
	ack = IIC2_Send_Byte(0x81);
	delay_us(2); 
	//需要器件应答
	if(!ack) return -1;
	
//	IIC2_Send_Byte(0);//给你个地址
//	delay_us(2); 
//	//需要器件应答
//	
//	IIC2_Start();
//	IIC2_Send_Byte(0x81);//我要读取上次给你地址里面的值
//	delay_us(2); 
//	//需要器件应答
	
	
	data[0] = IIC2_Read_Byte(1);//读取一个字节
	data[1] = IIC2_Read_Byte(1);//读取一个字节
	data[2] = IIC2_Read_Byte(1);//读取一个字节
	data[3] = IIC2_Read_Byte(1);//读取一个字节
	
	IIC2_Stop();
	return(data[0]);
}























