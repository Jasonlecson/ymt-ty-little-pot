/**
  ******************************************************************************
  * @author  yang feng wu 
  * @version V1.0.0
  * @date    2020/10/10
  * @brief   main
  ******************************************************************************
  ******************************************************************************
*/

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cString.h"
#include "System.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
#include "adc.h"
#include "relay.h"
#include "key.h"
#include "buzzer.h"
#include "ConfigModuleNoBlock.h"
#include "PVD.h"
#include "flash_helper.h"
#include "iwdg.h"
#include "wifi.h"
#include "wifi_state_led.h"
#include "ds18b20.h"
#include "TM601.h"
#include "iic.h"
#include "iic2.h"
#include "sensor_control.h"
#include "fill_light.h"
#include "FlashData.h"


/*0:最初版本*/
/*1:更改光照传感器为BH1750; 更改PID*/
u16 HardwareVersion = 0;

//缓存数据使用
unsigned char MainBuffer[MainBufferLen];//缓存数据,全局通用
u32  MainLen=0;      //全局通用变量
unsigned char *MainString;    //全局通用变量
u16 ReadDat[2];

char RelayState=0;

//
char led_state_change_flag=0;

void wifi_work_state_led(void);
void KeyFun(void);//处理按键
void TimerData(void);//需要定时运行的程序

int main(void)
{
	SCB->VTOR = FLASH_BASE | 0x5000;
  NVIC_Configuration();
	/*获取硬件版本*/
	HardwareVersion = IAPGetHardwareVersion();
	if(HardwareVersion == 1)
	{
		UartInit(115200,9600,115200);	 //串口初始化为9600
	}
	else
	{
		UartInit(115200,115200,115200);	 //串口初始化为115200
	}
	printf("\r\nHardwareVersion:%d\r\n",HardwareVersion);
	Timer2Init();
	DelayInit();
	/*初始化存储*/
	FlashHelperInit(FLASH_DATA_ADDR_1,FLASH_DATA_ADDR_2,FLASH_DATA_ADDR_3,FLASH_DATA_ADDR_4);
	
	
	RelayInit();
	AdcInit();
	KeyInit();
	BuzzerInit();
//	DS18B20Init();//温度
	IIC_Init();//光照
	fill_light_init();//补光灯
	wifi_state_led_init();
	PVDInit(PWR_PVDLevel_2V9);
	
	STMFLASH_Read(FLASH_UPDATE_FLAGE_ADDR,ReadDat,2);
	if(ReadDat[0] != 0xFFFF || ReadDat[1] != 0xFFFF){
		ReadDat[0] = 0xFFFF;
		ReadDat[1] = 0xFFFF;
		STMFLASH_Write(FLASH_UPDATE_FLAGE_ADDR,ReadDat,2);
	}
	
	wifi_protocol_init();
	sensor_data_value_init();//初始化传感器数据

	printf("***********start***********\r\n");
	IWDG_Init(IWDG_Prescaler_256,156*5);
	
	wifi_state_leda_set(1);
	
	while(1)
	{
		IWDG_Feed();//喂狗
		
		wifi_work_state_led();
		
		if(wifi_state == WIFI_CONN_CLOUD && mcu_updata_flag ==0){
			//采集传感器数据
			sensor_data_collection();
			//上报传感器数据
			sensor_data_appear();

			//每培育1天,基质减一 , 爱心值累加10
			NurtureDayFunction();
			//循环浇水
			WateringCirculationFunction();
			//事件上报函数
			event_report_fun();
			//统计光照强度
			illumination_intensity_statistics();
		}
		
		wifi_uart_service();
			/*******************************************???*******************************************/ 
			sensor_control_struct_value.pump_value = RelayGet(); 
			//??????h????????????? 
			sensor_watering(); 
		if(mcu_updata_flag==0){
			//需要定时运行的程序
			TimerData();
			KeyFun();//处理按键
		}
	}
}

/*****************************************************************************
函数名称 : wifi_work_state_led
功能描述 : wifi状态led控制
输入参数 : 无
返回参数 : 无
使用说明 : 无
*****************************************************************************/
void wifi_work_state_led(void)
{
	wifi_state = mcu_get_wifi_work_state();
	if(wifi_state != wifi_state_copy){
		wifi_state_copy = wifi_state;
		printf("*****************wifi_state*****************: %d \r\n",wifi_state_copy);
	}
	
	if(SmartConfigStartFlag){wifi_state=0;}
	switch(wifi_state)
	{
		case SMART_CONFIG_STATE://配网模式,快闪
			if(timer2Value1>250){
				timer2Value1=0;
				wifi_state_leda_set(-1);
			}
			led_state_change_flag=1;
		break;
			
		case AP_STATE://AP模式,慢闪
			if(timer2Value1>1500){
				timer2Value1=0;
				wifi_state_leda_set(-1);
			}
			led_state_change_flag=1;
		break;

		case WIFI_NOT_CONNECTED://没有连接路由器
			if(timer2Value1>1500){
				timer2Value1=0;
				wifi_state_leda_set(-1);
			}
			led_state_change_flag=1;
		break;

		case WIFI_CONNECTED://连接路由器
//			if(SmartConfigFlag==1){
//				SmartConfigFlag=0;
//				factory_reset();//恢复出厂设置
//				wifi_state_led_set(1);
//				FlashSetLedState(1);
//			}
//			led_state_change_flag=1;
		break;

		case WIFI_CONN_CLOUD://连接上云端
			if(SmartConfigFlag==1){
				SmartConfigFlag=0;
				factory_reset();//恢复出厂设置
				wifi_state_leda_set(1);
				FlashSetLedState(1);
				all_data_update();
			}
			if(led_state_change_flag==1){
				led_state_change_flag=0;
				sensor_light_set(FlashGetLedState());
			}
		break;
		
		default:
			if(timer2Value1>1500){
				timer2Value1=0;
				wifi_state_leda_set(-1);
			}
			led_state_change_flag=1;
		break;
	}
}



/**
* @brief   处理按键
* @param   None
* @retval  None
* @warning None
* @example
**/
void KeyFun(void)
{
	static int Key_down_value=0;
	if(KeyValueTemp[3] == 1)
	{
		Key_down_value = KeyValueTemp[5];
		if(KeyValueTemp[5]>=2500)//按下时间大于3S
		{
			printf("\r\nKeyFun\r\n");
			SmartConfigStartFlag = 1;
			SmartConfigFlag = 1;
			KeyValueTemp[3] = 0;//清零以后,只有按键松开再按下的时候才会进入
			mcu_set_wifi_mode(SMART_CONFIG);
		}
	}
	
	//松开
	if(KeyValueTemp[4] == 1)
	{
		KeyValueTemp[4] = 0;//清零以后,只有按键按下然后松开的时候才会进入
		//执行的代码
		printf("\r\nKeyUP\r\n");
		if(Key_down_value>100 && Key_down_value<2000){
			if(wifi_state == WIFI_CONN_CLOUD){
			printf("\r\nKeyUP _ 浇水\r\n");
			sensor_pump_set(1);
			}
		}
	}
}


/**
* @brief   定时循环函数(在1ms定时器里面执行)
* @param   value
* @retval  None
* @warning None
* @example
**/
void TimerLoopFun(int value)
{
	
}



/**
* @brief   需要定时运行的程序
* @param   None
* @retval  None
* @warning None
* @example
**/
void TimerData(void)
{
	if(wifi_state == WIFI_CONN_CLOUD){
		//定时获取一次时间戳
		if(timestamp == 0){//没有获取到时间戳
			if(TimerDataValue1 > TIMER_COMPARE_3SEC){
				TimerDataValue1 = 0;
				mcu_get_system_time();// mcu_get_greentime
			}
		}else{
			if(TimerDataValue1>=TIMER_COMPARE_HALF_HOUR){
//			if(TimerDataValue1>=TIMER_COMPARE_3SEC){
				TimerDataValue1 = 0;
				mcu_get_system_time();// mcu_get_greentime
			}
		}
	}

//	if(timer2Value3>TIMER_COMPARE_SEC){
//		timer2Value3=0;
//		mcu_get_wifi_connect_status();
//	}
	
	
	//每隔1S把时间戳转换为时间
	if(timestamp!=0)//已经获取到时间戳
	{
		if(timer2Value2 > TIMER_COMPARE_SEC){
			timer2Value2 = 0;
			
			//拷贝时间
			timestamp_copy = timestamp;
			//时区变换
			timestamp_copy = timestamp_copy + timezone*3600;
			//时间戳转时间
			lcTime = localtime(&timestamp_copy);
			lcTime->tm_year = lcTime->tm_year + 1900;
			lcTime->tm_mon = lcTime->tm_mon + 1;
//			printf("\r\n时间=%d,%d,%d,%d,%d,%d\r\n",lcTime->tm_year,lcTime->tm_mon,lcTime->tm_mday,\
//			lcTime->tm_hour,lcTime->tm_min,lcTime->tm_sec);
		}
	}
}
