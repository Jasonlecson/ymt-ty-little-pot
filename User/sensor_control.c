#define SENSOR_CONTROL_C_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "sensor_control.h"
#include "relay.h"
#include "wifi_state_led.h"
#include "adc.h"
#include "timer.h"
#include "DataParse.h"
#include "wifi.h"
#include "ds18B20.h"
#include "iic.h"//光照强度
#include "iic2.h"//水位
#include "FlashData.h"
#include "fill_light.h"
#include "buzzer.h"
#include "delay.h"
#include "AHT10_I2C.h"

//用户按下按钮启动SmartConfig配网绑定
char SmartConfigStartFlag = 0;
//配网设置标志,连接上路由器以后清除此标志
//并恢复出厂设置
char SmartConfigFlag=0;

int wifi_state = -1;//模组状态
int wifi_state_copy = -1;//模组状态
int mcu_updata_flag =0 ;//0-正常运行; 1-更新中
int mcu_updata_flag_clear_cnt=0;//清除更新状态标记

struct tm *lcTime;//时间戳转时间
time_t timestamp; //时间戳
time_t timestamp_copy; //时间戳(会转换为北京时间)
char timezone = 0; //时区偏差(默认8,北京时间)
extern int fill_light_auto_off_cnt; 

sensor_control_struct sensor_control_struct_value;
event_struct event_struct_value;


//恢复出厂设置
void factory_reset(void){
	//种植的植物
	sensor_control_struct_value.botany_type = -1;
	sensor_control_struct_value.botany_type_copy = -1;
	FlashSetBotanyType(sensor_control_struct_value.botany_type);
	
	//爱心值
	sensor_control_struct_value.love_value = 0;
	sensor_control_struct_value.love_value_copy =0;
	FlashSetLoveValue(sensor_control_struct_value.love_value);
	
	//浇水模式
	sensor_control_struct_value.mode_value = 0;
	sensor_control_struct_value.mode_value_copy =0;
	FlashSetModeAuto();
	
	//补光灯模式
	sensor_control_struct_value.fill_light_mode_value = 0;
	sensor_control_struct_value.fill_light_mode_value_copy = 0;
	FlashSetFillLightMode(sensor_control_struct_value.fill_light_mode_value);
	
	
	//浇水次数
	sensor_control_struct_value.watering_count =0;
	sensor_control_struct_value.watering_count_copy =0;
	FlashSetWaterCount(sensor_control_struct_value.watering_count);
	
	//种植时间
	sensor_control_struct_value.plant_value=0;
//	sensor_control_struct_value.plant_value_copy=0;
	FlashSetDataPlant(sensor_control_struct_value.plant_value);
	
	//首次种植时间
	sensor_control_struct_value.plant_value=0;
//	sensor_control_struct_value.plant_value_copy=0;
	FlashSetOneDataPlant(sensor_control_struct_value.plant_value);
	
	//累计收获
	sensor_control_struct_value.cumulative_harvest_value=0;
	sensor_control_struct_value.cumulative_harvest_value_copy=0;
	FlashSetHarvestCount(sensor_control_struct_value.cumulative_harvest_value);
	
	//更改基质剩余量
	sensor_control_struct_value.matrix_value=100;
	sensor_control_struct_value.matrix_value_copy=100;
	FlashSetMatrixData(sensor_control_struct_value.matrix_value);
	
	//培育天数
	sensor_control_struct_value.nurture_day=0;
	sensor_control_struct_value.nurture_day_copy=0;
	FlashSetNurtureDay(sensor_control_struct_value.nurture_day);

	//夜间不报警浇水
	sensor_control_struct_value.report_night_value = 1;
	sensor_control_struct_value.report_night_value_copy = 1;
	FlashSetReportEvent(sensor_control_struct_value.report_night_value);
	
	//设置光照强度报警值
	sensor_control_struct_value.light_low_limit_value = 0;
	FlashSetLightAlarm(sensor_control_struct_value.light_low_limit_value);
}


//初始化传感器数据 + 事件变量
void sensor_data_value_init(void){
	
	wifi_state = -1;//模组状态
	wifi_state_copy = -1;//模组状态
	
	/*事件*/
	/*温度*/
	event_struct_value.low_temperature_timer_cnt=0;
	event_struct_value.high_temperature_timer_cnt=0;
	
	/*湿度*/
	event_struct_value.low_humidity_timer_cnt=0;
	event_struct_value.high_humidity_timer_cnt=0;
	/*光照*/
	event_struct_value.low_illumination_timer_cnt=0;
	event_struct_value.high_illumination_timer_cnt=0;
	/*肥力*/
	event_struct_value.low_fertility_timer_cnt=0;
	event_struct_value.high_fertility_timer_cnt=0;
	//缺水
	event_struct_value.hydropenia_timer_cnt = 0;
	//水位低
	event_struct_value.low_water_timer_cnt = 0;
	//水位高
	event_struct_value.high_water_timer_cnt=0;
	
	sensor_control_struct_value.water_full_flag=0;//水满标志
	sensor_control_struct_value.water_lack_flag=0;//缺水标志
	
	/*传感器*/
	//LED灯
	sensor_control_struct_value.light_err_flag=0;
	//水泵
	sensor_control_struct_value.pump_err_flag=0;
	//湿度
	sensor_control_struct_value.humidity_err_count=0;//传感器错误采集计数
	sensor_control_struct_value.humidity_err_flag=0;//湿度传感器异常
	sensor_control_struct_value.humidity_value=0;
	sensor_control_struct_value.humidity_value_copy=0;
	//空气湿度
	sensor_control_struct_value.humidity1_err_count=0;//传感器错误采集计数
	sensor_control_struct_value.humidity1_err_flag=0;//湿度传感器异常
	sensor_control_struct_value.humidity1_value=0;
	sensor_control_struct_value.humidity1_value_copy=0;
	//温度
	sensor_control_struct_value.temperature_err_count=0;//传感器错误采集计数
	sensor_control_struct_value.temperature_err_flag=0;//温度传感器异常
	sensor_control_struct_value.temperature_value=0;
	sensor_control_struct_value.temperature_value_copy=0;
	//光照
	sensor_control_struct_value.illumination_err_count=0;
	sensor_control_struct_value.illumination_err_flag=0;
	sensor_control_struct_value.illumination_value=0;
	sensor_control_struct_value.illumination_value_copy=0;
	//水位
	sensor_control_struct_value.water_stage_err_count=0;//传感器错误采集计数
	sensor_control_struct_value.water_stage_err_flag=0;//传感器异常
	sensor_control_struct_value.water_stage_value=0;
	sensor_control_struct_value.water_stage_value_copy=0;
	//肥力
	sensor_control_struct_value.fertility_err_count=0;//传感器错误采集计数
	sensor_control_struct_value.fertility_err_flag=0;//传感器异常
	sensor_control_struct_value.fertility_value=0;
	sensor_control_struct_value.fertility_value_copy=0;
	//电池电量
	sensor_control_struct_value.soc_value=0;
	sensor_control_struct_value.soc_value_copy=0;
	
	//浇水模式
	sensor_control_struct_value.mode_value = FlashGetModeWatering();
	sensor_control_struct_value.mode_value_copy = 0;
	printf("\r\n浇水模式=%d\r\n",sensor_control_struct_value.mode_value);
	
	//补光灯模式
	sensor_control_struct_value.fill_light_mode_value = FlashGetFillLightMode();
	sensor_control_struct_value.fill_light_mode_value_copy = 1- sensor_control_struct_value.fill_light_mode_value;
	printf("\r\n补光灯模式=%d\r\n",sensor_control_struct_value.fill_light_mode_value);
	
	//浇水次数
	sensor_control_struct_value.watering_count = FlashGetWaterCount();
	sensor_control_struct_value.watering_count_copy = 0;
	printf("\r\n浇水次数=%d\r\n",sensor_control_struct_value.watering_count);
	
	//植物类型
	sensor_control_struct_value.botany_type = FlashGetBotanyType();
	sensor_control_struct_value.botany_type_copy = 0;
	printf("\r\n植物类型=%ld\r\n",sensor_control_struct_value.botany_type);
	
	//爱心值
	sensor_control_struct_value.love_value = FlashGetLoveValue();
	sensor_control_struct_value.love_value_copy = 0;
	printf("\r\n爱心值=%d\r\n",sensor_control_struct_value.love_value);
	
	//夜间不报警浇水
	sensor_control_struct_value.report_night_value = FlashGetReportEvent();
	sensor_control_struct_value.report_night_value_copy = 0;
	printf("\r\n夜间不报警浇水=%d\r\n",sensor_control_struct_value.report_night_value);
	
	//安全值-温度
	FlashGetSafe(&sensor_control_struct_value.safe_temperature[0],&sensor_control_struct_value.safe_temperature[1],safe_type_temperature);
	printf("\r\n安全值-温度=%d,%d\r\n",sensor_control_struct_value.safe_temperature[0],sensor_control_struct_value.safe_temperature[1]);
	
	//安全值-湿度
	FlashGetSafe(&sensor_control_struct_value.safe_humidity[0],&sensor_control_struct_value.safe_humidity[1],safe_type_humidity);
	printf("\r\n安全值-温度=%d,%d\r\n",sensor_control_struct_value.safe_humidity[0],sensor_control_struct_value.safe_humidity[1]);
	
	//安全值-光照
	FlashGetSafe(&sensor_control_struct_value.safe_illumination[0],&sensor_control_struct_value.safe_illumination[1],safe_type_illumination);
	printf("\r\n安全值-光照=%d,%d\r\n",sensor_control_struct_value.safe_illumination[0],sensor_control_struct_value.safe_illumination[1]);
	
	//安全值-肥力
	FlashGetSafe(&sensor_control_struct_value.safe_fertility[0],&sensor_control_struct_value.safe_fertility[1],safe_type_fertility);
	printf("\r\n安全值-肥力=%d,%d\r\n",sensor_control_struct_value.safe_fertility[0],sensor_control_struct_value.safe_fertility[1]);
	
	//安全值-光照总量
	sensor_control_struct_value.light_low_limit_value = FlashGetLightAlarm();
	printf("\r\n安全值-光照总量=%d\r\n",sensor_control_struct_value.light_low_limit_value);
	//光照总量统计时间
	sensor_control_struct_value.illumination_statistics_timer_start = ILLUMINATION_STATISTICS_TIMER_START;
	sensor_control_struct_value.illumination_statistics_timer_stop = ILLUMINATION_STATISTICS_TIMER_STOP;
	printf("\r\n光照总量统计时间=%d,%d\r\n",sensor_control_struct_value.illumination_statistics_timer_start,sensor_control_struct_value.illumination_statistics_timer_stop);
	sensor_control_struct_value.illumination_statistics_timer_delay=0;//在定时器里面累加,设置统计时间间隔
	sensor_control_struct_value.illumination_statistics_cnt=0;
	sensor_control_struct_value.illumination_statistics_sum=0;//光照统计总量
	sensor_control_struct_value.illumination_statistics_count=0;//光照统计次数
	
	
	sensor_control_struct_value.illumination_statistics_timer_delay1=0;//在定时器里面累加,设置统计时间间隔
	sensor_control_struct_value.illumination_statistics_cnt1=0;
	
	
	//基质剩余量
	sensor_control_struct_value.matrix_value = FlashGetMatrixData();
	sensor_control_struct_value.matrix_value_copy = 0;
	printf("\r\n基质剩余量=%d\r\n",sensor_control_struct_value.matrix_value);
	
	sensor_control_struct_value.on = 1; 
	sensor_control_struct_value.on_copy = 1; 
	//更换植物种植的时间
	sensor_control_struct_value.plant_value = FlashGetDataPlant();
	printf("\r\n更换植物种植的时间=%d\r\n",sensor_control_struct_value.plant_value);
	
	//累计收获
	sensor_control_struct_value.cumulative_harvest_value = FlashGetHarvestCount();
	sensor_control_struct_value.cumulative_harvest_value_copy = 0;
	printf("\r\n累计收获=%ld\r\n",sensor_control_struct_value.cumulative_harvest_value);
	
	//首次种植植物时间(如果没有时间,则赋值植物种植时间)
	sensor_control_struct_value.begin_date_value = FlashGetOneDataPlant();
	if(sensor_control_struct_value.begin_date_value ==0 && sensor_control_struct_value.plant_value!=0){
		FlashSetOneDataPlant(sensor_control_struct_value.plant_value);
		sensor_control_struct_value.begin_date_value = FlashGetOneDataPlant();
	}
	printf("\r\n首次种植植物时间=%d\r\n",sensor_control_struct_value.begin_date_value);

	//安全值-浇水报警时间
	sensor_control_struct_value.safe_limit_time[0] = SAFE_LIMIT_TIME_START;
	sensor_control_struct_value.safe_limit_time[1] = SAFE_LIMIT_TIME_STOP;
	
}

/**
* @brief  轮训采集传感器数据
* @param  None
* @param  None
* @param  None
* @retval None
* @example 
**/
void sensor_data_collection(void)
{
	float float_value=0;
	float Value;
	unsigned short int usiValue[12];
	int int_value;
	/*******************************************获取湿度*******************************************/
	usiValue[0] = Get_Adc(1);usiValue[3] = Get_Adc(1);usiValue[6] = Get_Adc(1);usiValue[9] = Get_Adc(1);
	usiValue[1] = Get_Adc(1);usiValue[4] = Get_Adc(1);usiValue[7] = Get_Adc(1);usiValue[10] = Get_Adc(1);
	usiValue[2] = Get_Adc(1);usiValue[5] = Get_Adc(1);usiValue[8] = Get_Adc(1);usiValue[11] = Get_Adc(1);
	
	Value = SortMethod(usiValue,12,2);
	Value = Value/8;
	Value = Value/4096*3.3;//PA1(湿度)
	sensor_control_struct_value.humidity_v = Value;
	
	if(Value<1){//传感器有问题
		sensor_control_struct_value.humidity_err_count++;//错误计数累加
		if(sensor_control_struct_value.humidity_err_count > 3){//超过3次
			sensor_control_struct_value.humidity_err_count=0;
			sensor_control_struct_value.humidity_err_flag=1;//置位传感器错误标志
		}
	}
	else{
		sensor_control_struct_value.humidity_err_flag=0;
		sensor_control_struct_value.humidity_err_count=0;
		sensor_control_struct_value.humidity_value = humidity_conversion(Value);
	}
	
	
	/*******************************************获取空气温湿度*******************************************/
	if(timer2_temp_humi_value>2000)
	{
		timer2_temp_humi_value=0;
		read_AHT10_once();
		int_value = T1/10;
		if(int_value == 255)//采集有错误
		{
			sensor_control_struct_value.temperature_err_count++;//错误计数累加
			if(sensor_control_struct_value.temperature_err_count>3){//超过3次
				sensor_control_struct_value.temperature_err_count=0;
				sensor_control_struct_value.temperature_err_flag=1;//置位传感器错误标志
			}
		}
		else
		{
			sensor_control_struct_value.temperature_err_count=0;
			sensor_control_struct_value.temperature_err_flag=0;
			sensor_control_struct_value.temperature_value = int_value;
		}
		
		int_value = H1/10;
		if(int_value==255){//传感器有问题
			sensor_control_struct_value.humidity1_err_count++;//错误计数累加
			if(sensor_control_struct_value.humidity1_err_count > 3){//超过3次
				sensor_control_struct_value.humidity1_err_count=0;
				sensor_control_struct_value.humidity1_err_flag=1;//置位传感器错误标志
			}
		}
		else{
			sensor_control_struct_value.humidity1_err_flag=0;
			sensor_control_struct_value.humidity1_err_count=0;
			sensor_control_struct_value.humidity1_value = int_value;
		}
	}
	
//	/*******************************************获取温度*******************************************/
//	Value = DS18B20_Get_Temp();
//	if(Value==255){//采集有错误
//		sensor_control_struct_value.temperature_err_count++;//错误计数累加
//		if(sensor_control_struct_value.temperature_err_count>3){//超过3次
//			sensor_control_struct_value.temperature_err_count=0;
//			sensor_control_struct_value.temperature_err_flag=1;//置位传感器错误标志
//		}
//	}
//	else{
//		sensor_control_struct_value.temperature_err_count=0;
//		sensor_control_struct_value.temperature_err_flag=0;
//		sensor_control_struct_value.temperature_value = Value;
//	}
	
	
	/*******************************************获取光照强度*******************************************/
	Value = illumination_get();
	
	if(Value<=0){
		sensor_control_struct_value.illumination_err_count++;//错误计数累加
		if(sensor_control_struct_value.illumination_err_count>3){//超过3次
			sensor_control_struct_value.illumination_err_count=0;
			sensor_control_struct_value.illumination_err_flag=1;//置位传感器错误标志
		}
	}
	else{
		sensor_control_struct_value.illumination_err_count=0;
		sensor_control_struct_value.illumination_err_flag=0;
		sensor_control_struct_value.illumination_value = Value;
	}
	
	
	/*******************************************水位*******************************************/
	usiValue[0] = Get_Adc(9);usiValue[3] = Get_Adc(9);usiValue[6] = Get_Adc(9);usiValue[9] = Get_Adc(9);
	usiValue[1] = Get_Adc(9);usiValue[4] = Get_Adc(9);usiValue[7] = Get_Adc(9);usiValue[10] = Get_Adc(9);
	usiValue[2] = Get_Adc(9);usiValue[5] = Get_Adc(9);usiValue[8] = Get_Adc(9);usiValue[11] = Get_Adc(9);
	
	Value = SortMethod(usiValue,12,2);
	Value = Value/8;
	Value = Value/4096*3.3;//PA1(湿度)
	
	if(Value > 0.5){
		sensor_control_struct_value.water_stage_value=100;
	}
	else{
		sensor_control_struct_value.water_stage_value=0;
	}
	
	/*******************************************电池电量*******************************************/
	
	usiValue[0] = Get_Adc(7);usiValue[3] = Get_Adc(7);usiValue[6] = Get_Adc(7);usiValue[9] = Get_Adc(7);
	usiValue[1] = Get_Adc(7);usiValue[4] = Get_Adc(7);usiValue[7] = Get_Adc(7);usiValue[10] = Get_Adc(7);
	usiValue[2] = Get_Adc(7);usiValue[5] = Get_Adc(7);usiValue[8] = Get_Adc(7);usiValue[11] = Get_Adc(7);
	Value = SortMethod(usiValue,12,2);
	Value = Value/8;
	Value = Value/4096*3.3;
	if(Value>=2.1)
	{
		sensor_control_struct_value.soc_value=100;
	}
	else if(Value<=1.9)
	{
		sensor_control_struct_value.soc_value=0;
	}
	else
	{
		sensor_control_struct_value.soc_value = 500*Value-950;
	}
	
//	if(timer2_water_stage>100){
//		timer2_water_stage=0;
////		__disable_irq();
//		Value = iic2_read_byte();
////		__enable_irq();
//		if(Value==-1){
//			sensor_control_struct_value.water_stage_err_count++;//错误计数累加
//			if(sensor_control_struct_value.water_stage_err_count>3){//超过3次
//				sensor_control_struct_value.water_stage_err_count=0;
//				sensor_control_struct_value.water_stage_err_flag=1;//置位传感器错误标志
////				printf("水位传感器读取错误!\r\n");
//			}
//		}
//		else{
//			sensor_control_struct_value.water_stage_err_count=0;
//			sensor_control_struct_value.water_stage_err_flag=0;
//			Value = Value/2.5;
//			if(Value>100){
//				Value = 100;
//			}
//			if(Value<30){
//				Value=0;
//			}
//			else{
//				Value = Value-30;
//				Value = Value/70;
//				Value = Value*100;
//			}
//			sensor_control_struct_value.water_stage_value = Value;
//		}
//	}
	
	
	/*******************************************肥力*******************************************/
	if(sensor_control_struct_value.humidity_value>95){
		sensor_control_struct_value.fertility_value = 2000;
	}
	else{
		sensor_control_struct_value.fertility_value = \
		(sensor_control_struct_value.humidity_value*sensor_control_struct_value.humidity_value + 29*sensor_control_struct_value.humidity_value)/6 + \
		sensor_control_struct_value.temperature_value/2 ;
		if(sensor_control_struct_value.fertility_value<0){ sensor_control_struct_value.fertility_value=0; }
		if(sensor_control_struct_value.fertility_value>2000){ sensor_control_struct_value.fertility_value=2000; }
	}
	if(sensor_control_struct_value.humidity_value<=0){
		sensor_control_struct_value.fertility_value=0;
	}
	
	

	
	/*******************************************指示灯*******************************************/
	sensor_control_struct_value.light_value = wifi_state_leda_get();
	
	/*******************************************补光灯*******************************************/
	sensor_control_struct_value.fill_light_value = fill_light_get();
	
	
	
	if(timer2Value>1500){
		timer2Value=0;
		
		
//		mcu_dp_fault_update(DPID_FAULT,test);//上报事件
//		test = test*2;
//		if(test==2048){
//			test=1;
//		}
//			mcu_dp_value_update(DPID_WATER,test1);//上报
//		
//			
//			
//			if(test1==0){ test1 = 5; }
//			else if(test1==5){ test1 = 10; }
//			else if(test1==10){ test1 = 20; }
//			else if(test1==20){ test1 = 0; }
//			printf("\r\nfloat_value=%d\r\n",sensor_control_struct_value.illumination_value);
		
//		printf("\r\n植物类型=%ld\r\n",sensor_control_struct_value.botany_type);
		
//		mcu_get_system_time();
//		mcu_get_green_time();
		
//		sensor_control_struct_value.watering_count++;
//		mcu_dp_value_update(DPID_WATER_COUNT,1);
		
//		printf("humidity=%f\r\n",float_value);
//		printf("temp=%d\r\n",DS18B20_Get_Temp());
//		printf("illumination_get=%d\r\n",illumination_get());
//		printf("water_stage_value=%d\r\n",sensor_control_struct_value.water_stage_value);
//		printf("fertility_value=%d\r\n",sensor_control_struct_value.fertility_value);
//		
//		printf("pump_value=%d\r\n",sensor_control_struct_value.pump_value);
		
		
	}
}


//上报传感器数据
void sensor_data_appear(void){
	
	/*空气温湿度传感器*/
	if(timer2_appear1>100){
		timer2_appear1=0;
		
		//温度传感器
		if(sensor_control_struct_value.temperature_value - sensor_control_struct_value.temperature_value_copy >=1 ||
			sensor_control_struct_value.temperature_value_copy - sensor_control_struct_value.temperature_value >=1)
		{
			sensor_control_struct_value.temperature_value_copy = sensor_control_struct_value.temperature_value;
			
			mcu_dp_value_update(DPID_TEMP_CURRENT,sensor_control_struct_value.temperature_value);//上报
		}
		
		//湿度传感器(达到一定的变化)
		if(sensor_control_struct_value.humidity1_value - sensor_control_struct_value.humidity1_value_copy >2 ||
			sensor_control_struct_value.humidity1_value_copy - sensor_control_struct_value.humidity1_value >2)
		{
			sensor_control_struct_value.humidity1_value_copy = sensor_control_struct_value.humidity1_value;
			mcu_dp_value_update(DPID_HUMIDITY_ENVIRONMENT,sensor_control_struct_value.humidity1_value);//上报
			printf("\r\n空气湿度=%d\r\n",sensor_control_struct_value.humidity1_value);
		}
	}
	
	if(timer2_appear2>500){
		timer2_appear2=0;
		//湿度传感器(达到一定的变化)
		if(sensor_control_struct_value.humidity_value - sensor_control_struct_value.humidity_value_copy >2 ||
			sensor_control_struct_value.humidity_value_copy - sensor_control_struct_value.humidity_value >2)
		{
			sensor_control_struct_value.humidity_value_copy = sensor_control_struct_value.humidity_value;
			mcu_dp_value_update(DPID_HUMIDITY_CURRENT,sensor_control_struct_value.humidity_value);//上报
			
			printf("\r\n土壤湿度=%d\r\n",sensor_control_struct_value.humidity_value);
		}
		
		//光照传感器(达到一定的变化)
		if(sensor_control_struct_value.illumination_value - sensor_control_struct_value.illumination_value_copy >5 ||
			sensor_control_struct_value.illumination_value_copy - sensor_control_struct_value.illumination_value >5)
		{
			sensor_control_struct_value.illumination_value_copy = sensor_control_struct_value.illumination_value;
			mcu_dp_value_update(DPID_ILLUMINATION,sensor_control_struct_value.illumination_value);//上报
			
			printf("\r\n光照=%d\r\n",sensor_control_struct_value.illumination_value);
		}
		
//		mcu_dp_value_update(DPID_WATER,50);//上报
		
		//水位
		if(sensor_control_struct_value.water_stage_value - sensor_control_struct_value.water_stage_value_copy >2 ||
			sensor_control_struct_value.water_stage_value_copy - sensor_control_struct_value.water_stage_value >2)
		{
			sensor_control_struct_value.water_stage_value_copy = sensor_control_struct_value.water_stage_value;
			mcu_dp_value_update(DPID_WATER,sensor_control_struct_value.water_stage_value);//上报
			printf("\r\n水位=%d\r\n",sensor_control_struct_value.water_stage_value);
		}
	}
	
	
	if(timer2_appear3>3000){
		timer2_appear3=0;
		//肥力
		if(sensor_control_struct_value.fertility_value - sensor_control_struct_value.fertility_value_copy >20 ||
			sensor_control_struct_value.fertility_value_copy - sensor_control_struct_value.fertility_value >20)
		{
			sensor_control_struct_value.fertility_value_copy = sensor_control_struct_value.fertility_value;
			mcu_dp_value_update(DPID_FERTILITY,sensor_control_struct_value.fertility_value);//上报
			
			printf("\r\n肥力=%d\r\n",sensor_control_struct_value.fertility_value);
		}
		
		
		//电池电量
		if(sensor_control_struct_value.soc_value-sensor_control_struct_value.soc_value_copy>2 || 
			sensor_control_struct_value.soc_value_copy - sensor_control_struct_value.soc_value>2)
		{
			sensor_control_struct_value.soc_value_copy = sensor_control_struct_value.soc_value;
			mcu_dp_value_update(DPID_SOC,sensor_control_struct_value.soc_value);//上报
			
			printf("\r\n电池电量=%d\r\n",sensor_control_struct_value.soc_value);
		}
		
	}
	
	//水泵开关状态
	if(sensor_control_struct_value.pump_value != sensor_control_struct_value.pump_value_copy){
		sensor_control_struct_value.pump_value_copy = sensor_control_struct_value.pump_value;
		mcu_dp_bool_update(DPID_PUMP,sensor_control_struct_value.pump_value);//上报
	}
	
	//LED
	if(sensor_control_struct_value.light_value != sensor_control_struct_value.light_value_copy){
		sensor_control_struct_value.light_value_copy = sensor_control_struct_value.light_value;
		mcu_dp_bool_update(DPID_LIGHT,sensor_control_struct_value.light_value);//上报
	}
	
	
	/*******************************************补光灯*******************************************/
	if(sensor_control_struct_value.fill_light_value != sensor_control_struct_value.fill_light_value_copy){
		sensor_control_struct_value.fill_light_value_copy = sensor_control_struct_value.fill_light_value;
		mcu_dp_bool_update(DPID_FILL_LIGHT,sensor_control_struct_value.fill_light_value);//上报
	}
	
	//浇水模式
	if(sensor_control_struct_value.mode_value != sensor_control_struct_value.mode_value_copy){
		sensor_control_struct_value.mode_value_copy = sensor_control_struct_value.mode_value;
		mcu_dp_enum_update(DPID_MODE,sensor_control_struct_value.mode_value); //上报
		printf("\r\n浇水模式=%d\r\n",sensor_control_struct_value.mode_value);
	}
	
	//补光灯模式
	if(sensor_control_struct_value.fill_light_mode_value != sensor_control_struct_value.fill_light_mode_value_copy){
		sensor_control_struct_value.fill_light_mode_value_copy = sensor_control_struct_value.fill_light_mode_value;
		mcu_dp_enum_update(DPID_FILL_LIGHT_MODE,sensor_control_struct_value.fill_light_mode_value); //上报
		printf("\r\n补光灯模式=%d\r\n",sensor_control_struct_value.fill_light_mode_value);
	}
	
	//浇水次数
	if(sensor_control_struct_value.watering_count != sensor_control_struct_value.watering_count_copy){
		sensor_control_struct_value.watering_count_copy = sensor_control_struct_value.watering_count;
		
		mcu_dp_value_update(DPID_WATER_COUNT_DEVICE,sensor_control_struct_value.watering_count);
		printf("\r\n浇水次数=%d\r\n",sensor_control_struct_value.watering_count);
		mcu_dp_value_update(DPID_WATER_COUNT,1);
//		mcu_dp_value_update(DPID_WATER_COUNT,0);
	}
	
	//爱心值
	if(sensor_control_struct_value.love_value != sensor_control_struct_value.love_value_copy){
		sensor_control_struct_value.love_value_copy = sensor_control_struct_value.love_value;
		mcu_dp_value_update(DPID_LOVE_VALUE,sensor_control_struct_value.love_value);
		printf("\r\n爱心值=%d\r\n",sensor_control_struct_value.love_value);
	}
	
	//植物类型
	if(sensor_control_struct_value.botany_type != sensor_control_struct_value.botany_type_copy){
		sensor_control_struct_value.botany_type_copy = sensor_control_struct_value.botany_type;
		mcu_dp_value_update(DPID_TYPE,sensor_control_struct_value.botany_type);
		printf("\r\n植物类型=%ld\r\n",sensor_control_struct_value.botany_type);
	}

	//夜间不报警浇水
	if(sensor_control_struct_value.report_night_value != sensor_control_struct_value.report_night_value_copy){
		sensor_control_struct_value.report_night_value_copy = sensor_control_struct_value.report_night_value;
		mcu_dp_bool_update(DPID_REPORT_NIGHT,sensor_control_struct_value.report_night_value);
		printf("\r\n夜间不报警浇水=%d\r\n",sensor_control_struct_value.report_night_value);
	}
	
	
	//累计收获
	if(sensor_control_struct_value.cumulative_harvest_value != sensor_control_struct_value.cumulative_harvest_value_copy){
		sensor_control_struct_value.cumulative_harvest_value_copy = sensor_control_struct_value.cumulative_harvest_value;
		mcu_dp_value_update(DPID_CUMULATIVE_HARVEST,sensor_control_struct_value.cumulative_harvest_value);
		printf("\r\n累计收获=%ld\r\n",sensor_control_struct_value.cumulative_harvest_value);
	}
	
	//基质剩余量
	if(sensor_control_struct_value.matrix_value != sensor_control_struct_value.matrix_value_copy){
		sensor_control_struct_value.matrix_value_copy = sensor_control_struct_value.matrix_value;
		mcu_dp_value_update(DPID_MATRIX,sensor_control_struct_value.matrix_value);
		printf("\r\n基质剩余量=%d\r\n",sensor_control_struct_value.matrix_value);
	}
	
		if(sensor_control_struct_value.on != sensor_control_struct_value.on_copy){ 
		sensor_control_struct_value.on_copy = sensor_control_struct_value.on; 
		mcu_dp_bool_update(DPID_ON,sensor_control_struct_value.on);//??? 
	} 
}

/**
* @brief   事件采集函数(放在1ms定时器)
* @param   None
* @param   None
* @param   None
* @retval  None
* @warning None
* @example 
**/
void event_loop_fun(void){
	//温度低
	if(sensor_control_struct_value.temperature_value <= sensor_control_struct_value.safe_temperature[0]){
		event_struct_value.low_temperature_timer_cnt++;
		if(event_struct_value.low_temperature_timer_cnt > TIMER_COMPARE_20MIN){
			event_struct_value.low_temperature_timer_cnt = TIMER_COMPARE_20MIN;
		}
	}else{
		event_struct_value.low_temperature_timer_cnt = 0;
	}
	//温度高
	if(sensor_control_struct_value.temperature_value >= sensor_control_struct_value.safe_temperature[1]){
		event_struct_value.high_temperature_timer_cnt++;
		if(event_struct_value.high_temperature_timer_cnt > TIMER_COMPARE_20MIN){
			event_struct_value.high_temperature_timer_cnt = TIMER_COMPARE_20MIN;
		}
	}else{
		event_struct_value.high_temperature_timer_cnt = 0;
	}
	
	//湿度低
	if(sensor_control_struct_value.humidity_value <= sensor_control_struct_value.safe_humidity[0]){
		event_struct_value.low_humidity_timer_cnt++;
		if(event_struct_value.low_humidity_timer_cnt > TIMER_COMPARE_20MIN){
			event_struct_value.low_humidity_timer_cnt = TIMER_COMPARE_20MIN;
		}
	}else{
		event_struct_value.low_humidity_timer_cnt = 0;
	}
	//湿度高
	if(sensor_control_struct_value.humidity_value >= sensor_control_struct_value.safe_humidity[1]){
		event_struct_value.high_humidity_timer_cnt++;
		if(event_struct_value.high_humidity_timer_cnt > TIMER_COMPARE_20MIN){
			event_struct_value.high_humidity_timer_cnt = TIMER_COMPARE_20MIN;
		}
	}else{
		event_struct_value.high_humidity_timer_cnt = 0;
	}
	
	//肥力低
	if(sensor_control_struct_value.fertility_value <= sensor_control_struct_value.safe_fertility[0]){
		event_struct_value.low_fertility_timer_cnt++;
		if(event_struct_value.low_fertility_timer_cnt > TIMER_COMPARE_20MIN){
			event_struct_value.low_fertility_timer_cnt = TIMER_COMPARE_20MIN;
		}
	}else{
		event_struct_value.low_fertility_timer_cnt = 0;
	}
	//肥力高
	if(sensor_control_struct_value.fertility_value >= sensor_control_struct_value.safe_fertility[1]){
		event_struct_value.high_fertility_timer_cnt++;
		if(event_struct_value.high_fertility_timer_cnt > TIMER_COMPARE_20MIN){
			event_struct_value.high_fertility_timer_cnt = TIMER_COMPARE_20MIN;
		}
	}else{
		event_struct_value.high_fertility_timer_cnt = 0;
	}
	
	//光照低
	if(sensor_control_struct_value.illumination_value <= sensor_control_struct_value.safe_illumination[0]){
		event_struct_value.low_illumination_timer_cnt++;
		if(event_struct_value.low_illumination_timer_cnt > TIMER_COMPARE_20MIN){
			event_struct_value.low_illumination_timer_cnt = TIMER_COMPARE_20MIN;
		}
	}else{
		event_struct_value.low_illumination_timer_cnt = 0;
	}
	//光照高
	if(sensor_control_struct_value.illumination_value >= sensor_control_struct_value.safe_illumination[1]){
		event_struct_value.high_illumination_timer_cnt++;
		if(event_struct_value.high_illumination_timer_cnt > TIMER_COMPARE_20MIN){
			event_struct_value.high_illumination_timer_cnt = TIMER_COMPARE_20MIN;
		}
	}else{
		event_struct_value.high_illumination_timer_cnt = 0;
	}
	
	//缺水
	if(sensor_control_struct_value.water_stage_value <= SAFE_HYDROPENIA){
		event_struct_value.hydropenia_timer_cnt++;
		if(event_struct_value.hydropenia_timer_cnt >= TIMER_COMPARE_20MIN){
			event_struct_value.hydropenia_timer_cnt = TIMER_COMPARE_20MIN;
		}
	}
	else{
		event_struct_value.hydropenia_timer_cnt = 0;
	}
	//水位低
	if(sensor_control_struct_value.water_stage_value <= SAFE_WATER_LOW){
		event_struct_value.low_water_timer_cnt++;
		if(event_struct_value.low_water_timer_cnt >= TIMER_COMPARE_20MIN){
			event_struct_value.low_water_timer_cnt = TIMER_COMPARE_20MIN;
		}
	}
	else{
		event_struct_value.low_water_timer_cnt = 0;
	}
	
	//水位高
	if(sensor_control_struct_value.water_stage_value >= SAFE_WATER_HIGH){
		event_struct_value.high_water_timer_cnt++;
		if(event_struct_value.high_water_timer_cnt >= TIMER_COMPARE_20MIN){
			event_struct_value.high_water_timer_cnt = TIMER_COMPARE_20MIN;
		}
	}
	else{
		event_struct_value.high_water_timer_cnt = 0;
	}
}


/**
* @brief   统计光照强度
* @param   None
* @param   None
* @param   None
* @retval  None
* @warning None
* @example 
**/
void illumination_intensity_statistics(void){
	u32 value=0;
	long long endtime;
	time_t time_t_value;
	//从模块获取到了时间戳
  if(timestamp !=0){
		//种植了植物
		if(sensor_control_struct_value.botany_type != -1){
			
//			else
			{	//统计光照强度 (早晚8点)
				if(lcTime->tm_hour >= sensor_control_struct_value.illumination_statistics_timer_start && lcTime->tm_hour < sensor_control_struct_value.illumination_statistics_timer_stop)
				{
					if(sensor_control_struct_value.illumination_statistics_timer_delay>1000*0)//1分钟
					{
						sensor_control_struct_value.illumination_statistics_timer_delay = 0;
						
						if(sensor_control_struct_value.illumination_statistics_cnt<10)//采集 10 个数据
						{
							sensor_control_struct_value.illumination_statistics_temp[sensor_control_struct_value.illumination_statistics_cnt] = sensor_control_struct_value.illumination_value;
							sensor_control_struct_value.illumination_statistics_cnt++;
						}
						else//采集了数据
						{
							sensor_control_struct_value.illumination_statistics_cnt=0;
							
							if(FlashGetIlluminationState() == 1)//清除上报
							{
								FlashClearIlluminationState();
							}
							
							SortMethod(sensor_control_struct_value.illumination_statistics_temp,10,0);//排序
							
							value = sensor_control_struct_value.illumination_statistics_temp[9] + sensor_control_struct_value.illumination_statistics_temp[8] + sensor_control_struct_value.illumination_statistics_temp[7];
							value = value/3;
							
							/*****如果低于设置的光照阈值,打开补光灯****/
							if(value <= sensor_control_struct_value.safe_illumination[0]){
								if(FlashGetFillLightMode() == 0 && sensor_control_struct_value.on == 1){//自动打开补光灯
									sensor_fill_light_set(1);//打开补光灯
									fill_light_auto_off_cnt = 0;
									printf("自动打开补光灯\r\n");
								}
								else{
									printf("非自动打开补光灯\r\n");
								}
							}
							
							//获取存储的光照强度, 存储的次数, 最后统计数据时的时间戳
							sensor_control_struct_value.illumination_statistics_sum = FlashGetIlluminationStatistics(&sensor_control_struct_value.illumination_statistics_count,&endtime);

							time_t_value = endtime;
							if(localtime(&time_t_value)->tm_hour > localtime(&timestamp_copy)->tm_hour)//统计的时间点大于当前时间点
							{
								FlashSetIlluminationStatistics(0,0,timestamp_copy);//清除数据
								sensor_control_struct_value.illumination_statistics_sum=0;
								sensor_control_struct_value.illumination_statistics_count=0;
							}
							else if(localtime(&time_t_value)->tm_hour == localtime(&timestamp_copy)->tm_hour)//小时相同
							{
								if(localtime(&time_t_value)->tm_min >= localtime(&timestamp_copy)->tm_min){//统计的分钟数大于当前分钟数
									FlashSetIlluminationStatistics(0,0,timestamp_copy);//清除数据
									sensor_control_struct_value.illumination_statistics_sum=0;
									sensor_control_struct_value.illumination_statistics_count=0;
								}
							}
							else
							{
								if(timestamp_copy - endtime > 86400)//当前时间比统计的时间超过了24小时
								{
									FlashSetIlluminationStatistics(0,0,timestamp_copy);//清除数据
									sensor_control_struct_value.illumination_statistics_sum=0;
									sensor_control_struct_value.illumination_statistics_count=0;
								}
							}
							sensor_control_struct_value.illumination_statistics_sum = sensor_control_struct_value.illumination_statistics_sum + value;//累加上这次的光照强度
							sensor_control_struct_value.illumination_statistics_count = sensor_control_struct_value.illumination_statistics_count + 1;//次数加一
							
							FlashSetIlluminationStatistics(sensor_control_struct_value.illumination_statistics_sum,sensor_control_struct_value.illumination_statistics_count,timestamp_copy);//存储数据
						}
					}
							if(FlashGetFillLightMode() == 0 && fill_light_auto_off_cnt >= 1800){//自动打开补光灯
								sensor_fill_light_set(0);//关闭补光灯
							}
				}
				else
				{
					//到达统计时间
					if(lcTime->tm_hour == sensor_control_struct_value.illumination_statistics_timer_stop)
					{
						//没有上报事件
						if(FlashGetIlluminationState() == 0)
						{
							FlashSetIlluminationState();//设置已经上报标志
							
							if(FlashGetFillLightMode() == 0){//自动打开补光灯
								sensor_fill_light_set(0);//关闭补光灯
								printf("关闭补光灯\r\n");
							}
							
							//获取存储的光照强度, 存储的次数, 最后统计数据时的时间戳
							sensor_control_struct_value.illumination_statistics_sum = FlashGetIlluminationStatistics(&sensor_control_struct_value.illumination_statistics_count,&endtime);

							FlashSetIlluminationStatistics(0,0,timestamp_copy);//清除数据
							//统计的光照强度小于设置的光照强度
							if(FlashGetLightAlarm()!=65535 && sensor_control_struct_value.illumination_statistics_sum/sensor_control_struct_value.illumination_statistics_count < FlashGetLightAlarm())
							{
								mcu_dp_fault_update(DPID_FAULT,FAULT_TOTAL_LIGHT);//上报事件
							}
						}
					}
					
					/*补光灯是打开的,检测光照强度*/
					if(fill_light_get())
					{
						if(sensor_control_struct_value.illumination_statistics_timer_delay1>1000*360)//1分钟
						{
							sensor_control_struct_value.illumination_statistics_timer_delay1 = 0;
							
							if(sensor_control_struct_value.illumination_statistics_cnt1<10)//采集 10 个数据
							{
								sensor_control_struct_value.illumination_statistics_temp1[sensor_control_struct_value.illumination_statistics_cnt1] = sensor_control_struct_value.illumination_value;
								sensor_control_struct_value.illumination_statistics_cnt1++;
							}
							else//采集了数据
							{
								sensor_control_struct_value.illumination_statistics_cnt1=0;
								
								SortMethod(sensor_control_struct_value.illumination_statistics_temp1,10,0);//排序
								
								value = sensor_control_struct_value.illumination_statistics_temp1[9] + sensor_control_struct_value.illumination_statistics_temp1[8] + sensor_control_struct_value.illumination_statistics_temp1[7];
								value = value/3;
								
								/*****如果低于设置的光照阈值,打开补光灯****/
								if(value <= sensor_control_struct_value.safe_illumination[0]){
									if(FlashGetFillLightMode() == 0 && sensor_control_struct_value.on == 1){//自动打开补光灯
										sensor_fill_light_set(1);//打开补光灯
										fill_light_auto_off_cnt = 0;
										printf("自动打开补光灯\r\n");
									}
									else{
										printf("非自动打开补光灯\r\n");
									}
								}
								else{ 
									if(FlashGetFillLightMode() == 0){//自动打开补光灯
								sensor_fill_light_set(0);//关闭补光灯
								}
								printf("关闭补光灯\r\n");
							}
							}
						}
					}
				}
			}
		}
	}		
}


/**
* @brief   事件上报函数
* @param   None
* @param   None
* @param   None
* @retval  None
* @warning None
* @example
**/
void event_report_fun(void){
	if(timer2_event_appear>TIMER_COMPARE_3HOUR){//3小时
//	if(timer2_event_appear>TIMER_COMPARE_3SEC){//3S
		
		timer2_event_appear=0;
		//种植了植物
		if(sensor_control_struct_value.botany_type != -1){
			//在早8点到晚8点,或者用户设置不限制时间段
			if((lcTime->tm_hour>=sensor_control_struct_value.safe_limit_time[0] && lcTime->tm_hour<sensor_control_struct_value.safe_limit_time[1]) || FlashGetReportEvent()==0){
				//温度低
				if(event_struct_value.low_temperature_timer_cnt >= TIMER_COMPARE_20MIN){
					event_struct_value.low_temperature_timer_cnt = 0;
					mcu_dp_fault_update(DPID_FAULT,FAULT_TEMPERATURE_LOW);//上报事件
				}
				//温度高
				if(event_struct_value.high_temperature_timer_cnt >= TIMER_COMPARE_20MIN){
					event_struct_value.high_temperature_timer_cnt = 0;
					mcu_dp_fault_update(DPID_FAULT,FAULT_TEMPERATURE_HIGH);//上报事件
				}
				
				//湿度低
				if(event_struct_value.low_humidity_timer_cnt >= TIMER_COMPARE_20MIN){
					event_struct_value.low_humidity_timer_cnt = 0;
					mcu_dp_fault_update(DPID_FAULT,FAULT_HUMIDITY_LOW);//上报事件
				}
				//湿度高
				if(event_struct_value.high_humidity_timer_cnt >= TIMER_COMPARE_20MIN){
					event_struct_value.high_humidity_timer_cnt = 0;
					mcu_dp_fault_update(DPID_FAULT,FAULT_HUMIDITY_HIGH);//上报事件
				}
				
				//肥力低
				if(event_struct_value.low_fertility_timer_cnt >= TIMER_COMPARE_20MIN){
					event_struct_value.low_fertility_timer_cnt = 0;
					mcu_dp_fault_update(DPID_FAULT,FAULT_FERTILITY_LOW);//上报事件
				}
				//肥力高
				if(event_struct_value.high_fertility_timer_cnt >= TIMER_COMPARE_20MIN){
					event_struct_value.high_fertility_timer_cnt = 0;
					mcu_dp_fault_update(DPID_FAULT,FAULT_FERTILITY_HIGH);//上报事件
				}
				
				//光照低
				if(event_struct_value.low_illumination_timer_cnt >= TIMER_COMPARE_20MIN){
					event_struct_value.low_illumination_timer_cnt = 0;
					mcu_dp_fault_update(DPID_FAULT,FAULT_ILLUMINATION_LOW);//上报事件
				}
				//光照高
				if(event_struct_value.high_illumination_timer_cnt >= TIMER_COMPARE_20MIN){
					event_struct_value.high_illumination_timer_cnt = 0;
					mcu_dp_fault_update(DPID_FAULT,FAULT_ILLUMINATION_HIGH);//上报事件
				}
				
				//缺水
				if(event_struct_value.hydropenia_timer_cnt >= TIMER_COMPARE_20MIN){
					event_struct_value.hydropenia_timer_cnt = 0;
					mcu_dp_fault_update(DPID_FAULT,FAULT_WATER_HYDROPENIA);//上报事件
				}
				//水位高
				if(event_struct_value.high_water_timer_cnt >= TIMER_COMPARE_20MIN){
					event_struct_value.high_water_timer_cnt = 0;
					mcu_dp_fault_update(DPID_FAULT,FAULT_WATER_HIGH);//上报事件
				}
			}
		}
	}
}

/**
* @brief   每培育1天,基质减一 , 爱心值累加10
* @param   None
* @param   None
* @param   None
* @retval  None
* @warning None
* @example 
**/
void NurtureDayFunction(void)
{
	//已经获取到时间戳, 有开始种植时间
	if(timestamp!=0 && sensor_control_struct_value.begin_date_value!=0){
		if(sensor_control_struct_value.botany_type != -1){//正在种植
			//获取存储记录的已经培育的天数
			sensor_control_struct_value.nurture_day = FlashGetNurtureDay();
			//获取实际培育的天数
			sensor_control_struct_value.nurture_day_copy = (timestamp - sensor_control_struct_value.begin_date_value)/(3600*24);
			//时间不一样
			if(sensor_control_struct_value.nurture_day != sensor_control_struct_value.nurture_day_copy){
				sensor_control_struct_value.nurture_day = sensor_control_struct_value.nurture_day_copy;
				
				FlashSetNurtureDay(sensor_control_struct_value.nurture_day);
				sensor_control_struct_value.love_value=sensor_control_struct_value.love_value+10;//爱心值累加
				FlashSetLoveValue(sensor_control_struct_value.love_value);//存储爱心值
				/*基质每隔1天减一*/
				sensor_control_struct_value.matrix_value = FlashGetMatrixData();
				if(sensor_control_struct_value.matrix_value>0){
					sensor_control_struct_value.matrix_value = sensor_control_struct_value.matrix_value -1;
					FlashSetMatrixData(sensor_control_struct_value.matrix_value);
				}
			}
		}
	}
}

/**
* @brief   循环浇水
* @param   None
* @param   None
* @param   None
* @retval  None
* @warning None
* @example 
**/
void WateringCirculationFunction(void)
{
	if(timer2_auto_watering>TIMER_COMPARE_1HOUR){//1小时
		timer2_auto_watering=0;		
		//种植了植物
		if(sensor_control_struct_value.botany_type != -1){
			//在早8点到晚8点,或者用户设置不限制时间段
			if((lcTime->tm_hour>=sensor_control_struct_value.safe_limit_time[0] && lcTime->tm_hour<sensor_control_struct_value.safe_limit_time[1]) || FlashGetReportEvent()==0){
				//湿度低
				if(sensor_control_struct_value.humidity_value<sensor_control_struct_value.safe_humidity[0] && 
					//自动模式 && 水箱有水
					sensor_control_struct_value.mode_value == 0 && sensor_control_struct_value.on == 1 /*&& sensor_control_struct_value.water_stage_value>SAFE_HYDROPENIA*/){
					sensor_pump_set(1);
				}
			}
		}
	}
}



//电压和湿度的换算,humidity由100改成50，然后扩大2倍
int humidity_conversion(float Value){
	static int humidity_copy=0;
	int humidity=0;
	
	if(Value>=2.7){
		humidity = 0;
		humidity_copy =2*humidity;
	}else if(Value>=1.8){
		humidity = -22*Value+60;
		if(humidity>50)humidity=50;
		if(humidity<0)humidity=0;
		humidity_copy = 2*humidity;
	}else if(Value>=1.6){
		humidity = -100*Value+200;
		if(humidity>50)humidity=50;
		if(humidity<0)humidity=0;
		humidity_copy = 2*humidity;
	}else if(Value>=1.5){
		humidity = -200*Value+360;
		if(humidity>50)humidity=50;
		if(humidity<0)humidity=0;
		humidity_copy = 2*humidity;
	}else if(Value>=1.44){
		humidity = -333*Value+560;
		if(humidity>50)humidity=50;
		if(humidity<0)humidity=0;
		humidity_copy = 2*humidity;
	}else if(Value>=1.2){
		humidity = -500*Value+800;
		if(humidity>50)humidity=50;
		if(humidity<0)humidity=0;
		humidity_copy = 2*humidity;
	}else{
		
	}
	
	return humidity_copy;
}

//控制浇水一段时间以后停止浇水
//浇水次数累加,记录当前浇水次数
void sensor_watering(void){
	//在浇水状态
	if(sensor_control_struct_value.pump_value){
		if(timer2_watering>5000){
			timer2_watering=0;
			sensor_pump_set(0);
			
			sensor_control_struct_value.watering_count = sensor_control_struct_value.watering_count+1;
			FlashSetWaterCount(sensor_control_struct_value.watering_count);
			printf("\r\n浇水次数=%d\r\n",sensor_control_struct_value.watering_count);
			
			sensor_control_struct_value.love_value = sensor_control_struct_value.love_value + 1;
			FlashSetLoveValue(sensor_control_struct_value.love_value);
		}
	}
	else{
		timer2_watering=0;
	}
}


//控制水泵
void sensor_pump_set(char value){
	RelaySet(value);
}
//控制指示灯
void sensor_light_set(char value){
	wifi_state_leda_set(value);
}
//控制补光灯
void sensor_fill_light_set(char value){
	fill_light_set(value);
}
//控制蜂鸣器
void sensor_buzzer_set(char value){
	BuzzerSet(value);
}
