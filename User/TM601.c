

#include "TM601.h"
//***************************************************************************  
void i2c_delay()//简单延时函数  
{ 
	char i;
	for(i=0;i<2;i++); }  
//***************************************************************************  
void i2c_start()  //开始信号 SCL在高电平期间，SDA一个下降沿则表示启动信号  
{     
	TM601_SDA_OUT=1; //释放SDA总线  
	i2c_delay();  
	SCL=1;  
	i2c_delay();  
	TM601_SDA_OUT=0;  
	i2c_delay();  
	SCL=0;
	i2c_delay(); 
}  
//***************************************************************************  
void i2c_stop()   //停止 SCL在高电平期间，SDA一个上升沿则表示停止信号  
{  
    SCL=1;  
}
//***************************************************************************  
char i2c_checkack()  //应答 SCL在高电平期间，SDA被从设备拉为低电平表示应答  
{  
	char	bit_temp;  
	SCL=1;  
	bit_temp=TM601_SDA_IN;
	i2c_delay();  
	if(bit_temp)
		//无ACK回应
		return 0;
	SCL=0;  
	return 1;  
}  
//***************************************************************************  
void i2c_sendack()  //应答 SCL在高电平期间，SDA被从设备拉为低电平表示应答  
{  
	TM601_SDA_OUT=0;
	i2c_delay();
	SCL=1;
	i2c_delay();
	SCL=0;
	TM601_SDA_OUT=1;

}  
//***************************************************************************  
void i2c_sendnack()  //应答 SCL在高电平期间，SDA被从设备拉为低电平表示应答  
{  
	TM601_SDA_OUT=1;
	i2c_delay();
	SCL=1;
	i2c_delay();
	SCL=0;
}
//***************************************************************************  
void i2c_write_byte(unsigned char date) //写一个字节  
{  
	unsigned char i,temp,m;  
	temp=date;  

	for(i=0;i<8;i++)  
	{  
		m=temp;
		m=m&0x80;
		if(m==0x80){
			TM601_SDA_OUT=1;
		}
		else 
			TM601_SDA_OUT=0;
		temp=temp<<1;
		SCL=1;
		SCL=0;
	}  
	TM601_SDA_OUT=1;//释放SDA总线，接下来由从设备控制，比如从设备接收完数据后，在SCL为高时，拉低SDA作为应答信号  
	i2c_delay();  
}  
//***************************************************************************  
unsigned char i2c_read_byte()//读一个字节  
{  
	unsigned char i,k;  

	for(i=0;i<8;i++)  
	{
		i2c_delay();  
		SCL=1;//上升沿时，IIC设备将数据放在sda线上，并在高电平期间数据已经稳定，可以接收啦     
		k=(k<<1)|TM601_SDA_IN;  
		SCL=0;
	}  
	return k;  
}  
//***************************************************************************  
char	i2c_write(unsigned dev_addr, unsigned reg_addr, unsigned char * src_buf, unsigned char len)
{
	char i;
	i2c_start();
	i2c_write_byte((dev_addr<<1));//发送发送从设备地址 写操作 
	if(~i2c_checkack())
	{
		return 0;
	}
	i2c_delay();  		 
	i2c_write_byte(reg_addr);//发送发送寄存器地址 写操作 
	if(~i2c_checkack())
	{
		return 0; 
	}
	i2c_delay();  
	for(i=0;i<len;i++)
	{
		i2c_write_byte(src_buf[i]);//发送缓冲区数据 写操作 
		i2c_checkack();//等待从设备的响应  
		i2c_delay();  				
	}
	i2c_stop();//停止 
	return 1;
}
//***************************************************************************  
char	i2c_read_direct(unsigned dev_addr,unsigned char * dest_buf,unsigned char len)
{
	char i;
	i2c_start();//启动  
	i2c_write_byte((dev_addr<<1)+1);//发送发送从设备地址 读操作  
	
	if(~i2c_checkack())
		return 0;//等待从设备的响应  
	i2c_delay();
	dest_buf[0]=i2c_read_byte();//获取数据 
	for(i=1;i<len;i++)
	{
		i2c_sendack();
		i2c_delay();
		dest_buf[i]=i2c_read_byte();
	}
	i2c_sendnack();
	i2c_delay();
	i2c_stop();//停止  
	return 1;
}
//***************************************************************************  
/*
功能：33MHz频率下简单延时500us
输入：无
输出：无
*/
void Delay500us() 
{ 
	unsigned int i;
	for(i=0;i<800;i++);
	for(i=0;i<800;i++);
} 
//***************************************************************************  
/*
功能：读取出当前601的采样值
输入：chNum为需要读取采样值的通道号，
			sample为读取出采样值后需要存至的变量，
输出：1表示读取成功0表示读取失败
*/
char ReadThresholdFrom601(unsigned char chNum,unsigned int *sample)
{
	unsigned char i,j,chnum;
	unsigned char buffer[4];
	unsigned char CON_RESA2EEP=0xCA;
	char flag=0;
	if(i2c_write(CON_ADDR, 0x01,&CON_RESA2EEP, 1)==0)//进行单片机与水位芯片握手操作
	{
		return 0;
	}
	for(i=0;i<10;i++)
	{
		if(i2c_read_direct(CON_ADDR,buffer,0x03)==0)//轮询水位芯片寄存器中的数据，判断单片机和水位芯片是否握手成功
		{	
			return 0;
		}
		if(buffer[0]==0xCD)
		{
			flag=1;
			break;
		}
		else if(j>=9)
		{
			return 0;
		}
		else
			Delay500us();
	}

	if(flag==1)
	{
		chnum=chNum+0x30;
		if(i2c_write(CON_ADDR, 0x01,&chnum, 1)==0)//握手成功后，发送水位芯片采样的通道编号加上0x30
			return 0;
		for(j=0;j<10;j++)
		{
			if(i2c_read_direct(CON_ADDR,buffer,0x03)==0)//轮询水位芯片寄存器，判断是否操作成功
				return 0;
			if(buffer[0]==chNum)//判断水位芯片操作成功后，将得到的采样值赋值给指针
			{
				*sample=buffer[1]*256+buffer[2];
				break;
			}
			else if(j>=9)
			{
				return 0;
			}
			else
				Delay500us();
		}
		return 1;
	}
	else
		return 0;
}
//***************************************************************************  
/*
功能：将当前需要设置的采样值发送至601中
输入：chnelnum为需要设置采样值的通道号，
			sample为设置至601的采样值变量，
输出：1表示设置成功0表示设置失败
*/
char SetThresholdTo601(unsigned char chNum,unsigned int *sample)
{
	unsigned char i,j,chnum;
	unsigned char setsampl[2];
	unsigned char buffer[4];
	unsigned char CON_SET601CR=0x8C;
	char flag=0;
	if(i2c_write(CON_ADDR, 0x01,&CON_SET601CR, 1)==0)//进行单片机与水位芯片握手操作
		return 0;
	for(i=0;i<10;i++)
	{
		if(i2c_read_direct(CON_ADDR,buffer,0x03)==0)//轮询水位芯片寄存器中的数据，判断单片机和水位芯片是否握手成功
			return 0;	
		if(buffer[0]==0xCD)
		{
			flag=1;
			break;
		}
		else if(j>=9)
			return 0;
		else
			Delay500us();
	}
	if(flag==1)//握手成功后，发送需要设置的水位芯片参数和通道编号加上0x30
	{
		chnum=chNum+0x30;
		setsampl[0]=(*sample)/256;
		setsampl[1]=(*sample)%256;
		if(i2c_write(CON_ADDR, 0x02, setsampl, 2)==0)	
			return 0;	
		if(i2c_write(CON_ADDR, 0x01, &chnum, 1)==0)			
			return 0;	
		for(j=0;j<10;j++)
		{
			if(i2c_read_direct(CON_ADDR,buffer,0x03)==0)//轮询水位芯片寄存器，判断是否操作成功
				return 0;	
			if(buffer[0]==chNum)
				break;
			else if(j>=9)				
				return 0;	
			else
				Delay500us();
		}
		return 1;
	}
	else
		return 0;
}
