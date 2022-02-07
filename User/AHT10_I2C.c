#include "AHT10_I2C.h"
#include "delay.h"

uint8_t   ack_status=0;
uint8_t   readByte[6];
uint8_t   aht10_status=0;

volatile int H1=0;  //Humility
volatile int T1=0;  //Temperature

uint8_t  AHT10_OutData[4];
uint8_t  AHT10sendOutData[10] = {0xFA, 0x06, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF};

//IIC所有操作函数  声明/////////////////////////
void AHT10_IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 AHT10_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
//u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void AHT10_IIC_Ack(void);					//IIC发送ACK信号
void AHT10_IIC_NAck(void);				//IIC不发送ACK信号
void IIC_WriteByte(uint16_t addr,uint8_t data,uint8_t device_addr);
uint16_t IIC_ReadByte(uint16_t addr,uint8_t device_addr,uint8_t ByteNumToRead);//寄存器地址，器件地址，要读的字节数  
void  reset_AHT10(void);
void  init_AHT10(void);	
void  startMeasure_AHT10(void);
void  read_AHT10(void);
uint8_t  Receive_ACK(void);
void  AHT10_Send_ACK(void);
void  AHT10_SendNot_Ack(void);
void I2C_Write1Byte(uint8_t  input);
uint8_t I2C_ReadByte(void);	
void  set_AHT10sendOutData(void);
//void  I2C_Start(void);
//void  I2C_Stop(void);
//////////////////////////////////////////////


void AHT10_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = AHT10_SCL_PIN|AHT10_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(AHT10_PORT, &GPIO_InitStructure);
 
	AHT10_IIC_SCL=1;
	AHT10_IIC_SDA=1;
}
//产生IIC起始信号
void AHT10_IIC_Start(void)
{
	AHT10_SDA_OUT();     //sda线输出
	AHT10_IIC_SDA=1;	  	  
	AHT10_IIC_SCL=1;
	delay_us(4);
 	AHT10_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	AHT10_IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void AHT10_IIC_Stop(void)
{
	AHT10_SDA_OUT();//sda线输出
	AHT10_IIC_SCL=0;
	AHT10_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	AHT10_IIC_SCL=1; 
	AHT10_IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}

void  I2C_Start(void)
{
	AHT10_SDA_OUT();
	AHT10_IIC_SCL = 1;
	delay_us(10);

	AHT10_IIC_SDA = 1;
	delay_us(10);
	AHT10_IIC_SDA = 0;
	delay_us(10);

	AHT10_IIC_SCL = 0;
	delay_us(10);
}

void  I2C_Stop(void)
{
	AHT10_SDA_OUT();
	AHT10_IIC_SDA = 0;
	delay_us(10);

	AHT10_IIC_SCL = 1;
	delay_us(10);

	AHT10_IIC_SDA = 1;
	delay_us(10);
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 AHT10_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	AHT10_SDA_IN();      //SDA设置为输入  
	AHT10_IIC_SDA=1;delay_us(1);	   
	AHT10_IIC_SCL=1;delay_us(1);	 
	while(AHT10_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			AHT10_IIC_Stop();
			return 1;
		}
	}
	AHT10_IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void AHT10_IIC_Ack(void)
{
	AHT10_IIC_SCL=0;
	AHT10_SDA_OUT();
	AHT10_IIC_SDA=0;
	delay_us(2);
	AHT10_IIC_SCL=1;
	delay_us(2);
	AHT10_IIC_SCL=0;
}
//不产生ACK应答		    
void AHT10_IIC_NAck(void)
{
	AHT10_IIC_SCL=0;
	AHT10_SDA_OUT();
	AHT10_IIC_SDA=1;
	delay_us(2);
	AHT10_IIC_SCL=1;
	delay_us(2);
	AHT10_IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void AHT10_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
		AHT10_SDA_OUT(); 	    
    AHT10_IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        AHT10_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		AHT10_IIC_SCL=1;
		delay_us(2); 
		AHT10_IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 AHT10_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	AHT10_SDA_IN();//SDA设置为输入
  for(i=0;i<8;i++ )
	{
    AHT10_IIC_SCL=0; 
    delay_us(2);
		AHT10_IIC_SCL=1;
    receive<<=1;
    if(AHT10_READ_SDA)receive++;   
		delay_us(1); 
  }					 
	if (!ack)
			AHT10_IIC_NAck();//发送nACK
	else
			AHT10_IIC_Ack(); //发送ACK   
	return receive;
}
 
void IIC_WriteByte(uint16_t addr,uint8_t data,uint8_t device_addr)
{
	AHT10_IIC_Start();  
	
	if(device_addr==0xA0) //eeprom地址大于1字节
		AHT10_IIC_Send_Byte(0xA0 + ((addr/256)<<1));//发送高地址
	else
		AHT10_IIC_Send_Byte(device_addr);	    //发器件地址
	AHT10_IIC_Wait_Ack(); 
	AHT10_IIC_Send_Byte(addr&0xFF);   //发送低地址
	AHT10_IIC_Wait_Ack(); 
	AHT10_IIC_Send_Byte(data);     //发送字节							   
	AHT10_IIC_Wait_Ack();  		    	   
  AHT10_IIC_Stop();//产生一个停止条件 
	if(device_addr==0xA0) //
		delay_ms(10);
	else
		delay_us(2);
}
 
uint16_t IIC_ReadByte(uint16_t addr,uint8_t device_addr,uint8_t ByteNumToRead)  //读寄存器或读数据
{	
		uint16_t data;
		AHT10_IIC_Start();  
		if(device_addr==0xA0)
			AHT10_IIC_Send_Byte(0xA0 + ((addr/256)<<1));
		else
			AHT10_IIC_Send_Byte(device_addr);	
		AHT10_IIC_Wait_Ack();
		AHT10_IIC_Send_Byte(addr&0xFF);   //发送低地址
		AHT10_IIC_Wait_Ack(); 
 
		AHT10_IIC_Start();  	
		AHT10_IIC_Send_Byte(device_addr+1);	    //发器件地址
		AHT10_IIC_Wait_Ack();
		if(ByteNumToRead == 1)//LM75温度数据为11bit
		{
			data=AHT10_IIC_Read_Byte(0);
		}
		else
			{
				data=AHT10_IIC_Read_Byte(1);
				data=(data<<8)+AHT10_IIC_Read_Byte(0);
			}
		AHT10_IIC_Stop();//产生一个停止条件	    
		return data;
}


/**********
*上面部分为IO口模块I2C配置
*
*从这以下开始为AHT10的配置I2C
*函数名有IIC和I2C的区别，请注意！！！！！
*
*2020/2/23最后修改日期
*
***********/
void  read_AHT10_once(void)
{
	static int flag=0;
	
	static int delay=0;
	
	if(flag==0)
	{
		flag=1;
		AHT10_Init();
	}
	
	if(delay == 0)
	{
		delay_us(2);
		reset_AHT10();
		delay_ms(1);

		init_AHT10();
		delay_ms(1);

		startMeasure_AHT10();
	}
	delay = delay+1;
	if(delay >12)
	{
		delay = 0;
		read_AHT10();
	}
}


void  reset_AHT10(void)
{

	I2C_Start();

	I2C_Write1Byte(0x70);
	ack_status = Receive_ACK();
	//if(ack_status) printf("1");
	//else printf("1-n-");
	I2C_Write1Byte(0xBA);
	ack_status = Receive_ACK();
	//	if(ack_status) printf("2");
	//else printf("2-n-");
	I2C_Stop();

	/*
	AHT10_OutData[0] = 0;
	AHT10_OutData[1] = 0;
	AHT10_OutData[2] = 0;
	AHT10_OutData[3] = 0;
	*/
}



void  init_AHT10(void)
{
	I2C_Start();

	I2C_Write1Byte(0x70);
	ack_status = Receive_ACK();
	//if(ack_status) printf("3");
	//else printf("3-n-");	
	I2C_Write1Byte(0xE1);
	ack_status = Receive_ACK();
	//if(ack_status) printf("4");
	//else printf("4-n-");
	I2C_Write1Byte(0x08);
	ack_status = Receive_ACK();
	//if(ack_status) printf("5");
	//else printf("5-n-");
	I2C_Write1Byte(0x00);
	ack_status = Receive_ACK();
	//if(ack_status) printf("6");
	//else printf("6-n-");
	I2C_Stop();
}



void  startMeasure_AHT10(void)
{
	//------------
	I2C_Start();

	I2C_Write1Byte(0x70);
	ack_status = Receive_ACK();
	//if(ack_status) printf("7");
	//else printf("7-n-");
	I2C_Write1Byte(0xAC);
	ack_status = Receive_ACK();
	//if(ack_status) printf("8");
	//else printf("8-n-");
	I2C_Write1Byte(0x33);
	ack_status = Receive_ACK();
	//if(ack_status) printf("9");
	//else printf("9-n-");
	I2C_Write1Byte(0x00);
	ack_status = Receive_ACK();
	//if(ack_status) printf("10");
	//else printf("10-n-");
	I2C_Stop();
}



void read_AHT10(void)
{
	uint8_t   i;
	long T11,H11;

	for(i=0; i<6; i++)
	{
		readByte[i]=0;
	}

	//-------------
	I2C_Start();

	I2C_Write1Byte(0x71);
	ack_status = Receive_ACK();
	//if(ack_status) printf("11");
	//else printf("11-n-");
	readByte[0]= I2C_ReadByte();
	//printf("test0:%d",readByte[0]);
	AHT10_Send_ACK();

	readByte[1]= I2C_ReadByte();
	//printf("test1:%d",readByte[1]);
	AHT10_Send_ACK();

	readByte[2]= I2C_ReadByte();
//	printf("test2:%d",readByte[2]);
	AHT10_Send_ACK();

	readByte[3]= I2C_ReadByte();
	//printf("test3:%d",readByte[3]);
	AHT10_Send_ACK();

	readByte[4]= I2C_ReadByte();
	//printf("test4:%d",readByte[4]);
	AHT10_Send_ACK();

	readByte[5]= I2C_ReadByte();
	//printf("test5:%d",readByte[5]);
	AHT10_SendNot_Ack();
	//Send_ACK();

	I2C_Stop();

	//--------------
	if( (readByte[0] & 0x68) == 0x08 )
	{
		H11 = readByte[1];
		H11 = (H11<<8) | readByte[2];
		H11 = (H11<<8) | readByte[3];
		H11 = H11>>4;

		H11 = (H11*1000)/1024/1024;

		T11 = readByte[3];
		T11 = T11 & 0x0000000F;
		T11 = (T11<<8) | readByte[4];
		T11 = (T11<<8) | readByte[5];

		T11 = (T11*2000)/1024/1024 - 500;
    T1 = T11;
		H1 = H11;
		AHT10_OutData[0] = (H11>>8) & 0x000000FF;
		AHT10_OutData[1] = H11 & 0x000000FF;

		AHT10_OutData[2] = (T11>>8) & 0x000000FF;
		AHT10_OutData[3] = T11 & 0x000000FF;
		//printf("成功了");
	}
	/*else
	{
		AHT10_OutData[0] = 0xFF;
		AHT10_OutData[1] = 0xFF;

		AHT10_OutData[2] = 0xFF;
		AHT10_OutData[3] = 0xFF;
		printf("失败了");

	}*/
	//printf("\r\n");
	//printf("温度:%d%d.%d",T1/100,(T1/10)%10,T1%10);
	//printf("湿度:%d%d.%d",H1/100,(H1/10)%10,H1%10);
	//printf("\r\n");
}




uint8_t  Receive_ACK(void)
{
	uint8_t result=0;
	uint8_t cnt=0;

	AHT10_IIC_SCL = 0;
	AHT10_SDA_IN(); 
	delay_us(4);

	AHT10_IIC_SCL = 1;
	delay_us(4);

	while(AHT10_READ_SDA && (cnt<100))
	{
		cnt++;
	}

	AHT10_IIC_SCL = 0;
	delay_us(4);

	if(cnt<100)
	{
		result=1;
	}
	return result;
}



void  AHT10_Send_ACK(void)
{
	AHT10_SDA_OUT();
	AHT10_IIC_SCL = 0;
	delay_us(4);

	AHT10_IIC_SDA = 0;
	delay_us(4);

	AHT10_IIC_SCL = 1;
	delay_us(4);
	AHT10_IIC_SCL = 0;
	delay_us(4);

	AHT10_SDA_IN();
}



void  AHT10_SendNot_Ack(void)
{
	AHT10_SDA_OUT();
	AHT10_IIC_SCL = 0;
	delay_us(4);

	AHT10_IIC_SDA = 1;
	delay_us(4);

	AHT10_IIC_SCL = 1;
	delay_us(4);

	AHT10_IIC_SCL = 0;
	delay_us(4);

	AHT10_IIC_SDA = 0;
	delay_us(4);
}


void I2C_Write1Byte(uint8_t  input)
{
	uint8_t  i;
	AHT10_SDA_OUT();
	for(i=0; i<8; i++)
	{
		AHT10_IIC_SCL = 0;
		delay_us(4);

		if(input & 0x80)
		{
			AHT10_IIC_SDA = 1;
			//delaymm(10);
		}
		else
		{
			AHT10_IIC_SDA = 0;
			//delaymm(10);
		}

		AHT10_IIC_SCL = 1;
		delay_us(4);

		input = (input<<1);
	}

	AHT10_IIC_SCL = 0;
	delay_us(4);

	AHT10_SDA_IN();
	delay_us(4);
}	


uint8_t I2C_ReadByte(void)
{
	uint8_t  resultByte=0;
	uint8_t  i=0, a=0;

	AHT10_IIC_SCL = 0;
	AHT10_SDA_IN();
	delay_us(4);

	for(i=0; i<8; i++)
	{
		AHT10_IIC_SCL = 1;
		delay_us(4);

		a=0;
		if(AHT10_READ_SDA)
		{
			a=1;
		}
		else
		{
			a=0;
		}

		//resultByte = resultByte | a;
		resultByte = (resultByte << 1) | a;

		AHT10_IIC_SCL = 0;
		delay_us(4);
	}

	AHT10_SDA_IN();
	delay_us(4);

	return   resultByte;
}


void  set_AHT10sendOutData(void)
{
	/* --------------------------
	 * 0xFA 0x06 0x0A temperature(2 Bytes) humility(2Bytes) short Address(2 Bytes)
	 * And Check (1 byte)
	 * -------------------------*/
	AHT10sendOutData[3] = AHT10_OutData[0];
	AHT10sendOutData[4] = AHT10_OutData[1];
	AHT10sendOutData[5] = AHT10_OutData[2];
	AHT10sendOutData[6] = AHT10_OutData[3];

//	AHT10sendOutData[7] = (drf1609.shortAddress >> 8) & 0x00FF;
//	AHT10sendOutData[8] = drf1609.shortAddress  & 0x00FF;

//	AHT10sendOutData[9] = getXY(AHT10sendOutData,10);
}

