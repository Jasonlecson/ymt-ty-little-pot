/**********************************Copyright (c)**********************************
**                       版权所有 (C), 2015-2020, 涂鸦科技
**
**                             http://www.tuya.com
**
*********************************************************************************/
/**
 * @file    protocol.c
 * @author  涂鸦综合协议开发组
 * @version v2.5.6
 * @date    2020.12.16
 * @brief                
 *                       *******非常重要，一定要看哦！！！********
 *          1. 用户在此文件中实现数据下发/上报功能
 *          2. DP的ID/TYPE及数据处理函数都需要用户按照实际定义实现
 *          3. 当开始某些宏定义后需要用户实现代码的函数内部有#err提示,完成函数后请删除该#err
 */

/****************************** 免责声明 ！！！ *******************************
由于MCU类型和编译环境多种多样，所以此代码仅供参考，用户请自行把控最终代码质量，
涂鸦不对MCU功能结果负责。
******************************************************************************/

/******************************************************************************
                                移植须知:
1:MCU必须在while中直接调用mcu_api.c内的wifi_uart_service()函数
2:程序正常初始化完成后,建议不进行关串口中断,如必须关中断,关中断时间必须短,关中断会引起串口数据包丢失
3:请勿在中断/定时器中断内调用上报函数
******************************************************************************/

#include "wifi.h"
#include "usart.h"

#include <stdlib.h>
#include "sensor_control.h"
#include "FlashData.h"
#include "cString.h"
#include "stmflash.h"
#include "System.h"
#include "delay.h"

#ifdef WEATHER_ENABLE
/**
 * @var    weather_choose
 * @brief  天气数据参数选择数组
 * @note   用户可以自定义需要的参数，注释或者取消注释即可，注意更改
 */
const char *weather_choose[WEATHER_CHOOSE_CNT] = {
    "temp",
    "humidity",
    "condition",
    "pm25",
    /*"pressure",
    "realFeel",
    "uvi",
    "tips",
    "windDir",
    "windLevel",
    "windSpeed",
    "sunRise",
    "sunSet",
    "aqi",
    "so2 ",
    "rank",
    "pm10",
    "o3",
    "no2",
    "co",
    "conditionNum",*/
};
#endif


/******************************************************************************
                              第一步:初始化
1:在需要使用到wifi相关文件的文件中include "wifi.h"
2:在MCU初始化中调用mcu_api.c文件中的wifi_protocol_init()函数
3:将MCU串口单字节发送函数填入protocol.c文件中uart_transmit_output函数内,并删除#error
4:在MCU串口接收函数中调用mcu_api.c文件内的uart_receive_input函数,并将接收到的字节作为参数传入
5:单片机进入while循环后调用mcu_api.c文件内的wifi_uart_service()函数
******************************************************************************/

/******************************************************************************
                        1:dp数据点序列类型对照表
          **此为自动生成代码,如在开发平台有相关修改请重新下载MCU_SDK**         
******************************************************************************/
const DOWNLOAD_CMD_S download_cmd[] =
{
	{DPID_ON, DP_TYPE_BOOL},
  {DPID_PUMP, DP_TYPE_BOOL},
  {DPID_LIGHT, DP_TYPE_BOOL},
  {DPID_TEMP_CURRENT, DP_TYPE_VALUE},
  {DPID_HUMIDITY_CURRENT, DP_TYPE_VALUE},
  {DPID_FAULT, DP_TYPE_BITMAP},
  {DPID_MODE, DP_TYPE_ENUM},
  {DPID_FERTILITY, DP_TYPE_VALUE},
  {DPID_WATER, DP_TYPE_VALUE},
  {DPID_WATER_COUNT, DP_TYPE_VALUE},
  {DPID_TYPE, DP_TYPE_VALUE},
  {DPID_ILLUMINATION, DP_TYPE_VALUE},
  {DPID_START_DATE, DP_TYPE_STRING},
  {DPID_MATRIX, DP_TYPE_VALUE},
  {DPID_LOVE_VALUE, DP_TYPE_VALUE},
  {DPID_WATER_COUNT_DEVICE, DP_TYPE_VALUE},
  {DPID_CUMULATIVE_HARVEST, DP_TYPE_VALUE},
  {DPID_BEGIN_DATE, DP_TYPE_STRING},
  {DPID_MANUAL_REFRESH, DP_TYPE_BOOL},
  {DPID_REPORT_NIGHT, DP_TYPE_BOOL},
  {DPID_SAFE_TEMPERATURE, DP_TYPE_STRING},
  {DPID_SAFE_HUMIDITY, DP_TYPE_STRING},
  {DPID_SAFE_ILLUMINATION, DP_TYPE_STRING},
  {DPID_SAFE_FERTILITY, DP_TYPE_STRING},
  {DPID_LIGHT_LOW_LIMIT, DP_TYPE_VALUE},
	{DPID_FILL_LIGHT_MODE, DP_TYPE_ENUM},
	{DPID_FILL_LIGHT, DP_TYPE_BOOL},
	{DPID_SOC, DP_TYPE_VALUE},
  {DPID_HUMIDITY_ENVIRONMENT, DP_TYPE_VALUE},
};



/******************************************************************************
                           2:串口单字节发送函数
请将MCU串口发送函数填入该函数内,并将接收到的数据作为参数传入串口发送函数
******************************************************************************/

/**
 * @brief  串口发送数据
 * @param[in] {value} 串口要发送的1字节数据
 * @return Null
 */
void uart_transmit_output(unsigned char value)
{
//    #error "请将MCU串口发送函数填入该函数,并删除该行"
    UsartOutChar(USART2, value);
/*
    //Example:
    extern void Uart_PutChar(unsigned char value);
    Uart_PutChar(value);	                                //串口发送函数
*/
}

/******************************************************************************
                           第二步:实现具体用户函数
1:APP下发数据处理
2:数据上报处理
******************************************************************************/

/******************************************************************************
                            1:所有数据上报处理
当前函数处理全部数据上报(包括可下发/可上报和只上报)
  需要用户按照实际情况实现:
  1:需要实现可下发/可上报数据点上报
  2:需要实现只上报数据点上报
此函数为MCU内部必须调用
用户也可调用此函数实现全部数据上报
******************************************************************************/

//自动化生成数据上报函数

/**
 * @brief  系统所有dp点信息上传,实现APP和muc数据同步
 * @param  Null
 * @return Null
 * @note   此函数SDK内部需调用，MCU必须实现该函数内数据上报功能，包括只上报和可上报可下发型数据
 */
void all_data_update(void)
{
//    #error "请在此处理可下发可上报数据及只上报数据示例,处理完成后删除该行"
	mcu_dp_bool_update(DPID_ON,sensor_control_struct_value.on);
	//当前浇水功能
	mcu_dp_bool_update(DPID_PUMP,sensor_control_struct_value.pump_value);
	//BOOL型数据上报;当前指示灯开关
	mcu_dp_bool_update(DPID_LIGHT,sensor_control_struct_value.light_value);
	//当前补光灯); //BOOL型数据上报;
	mcu_dp_bool_update(DPID_FILL_LIGHT,sensor_control_struct_value.fill_light_value);
	
	//当前当前温度
	mcu_dp_value_update(DPID_TEMP_CURRENT,sensor_control_struct_value.temperature_value);
	//VALUE型数据上报;当前相对湿度
	mcu_dp_value_update(DPID_HUMIDITY_CURRENT,sensor_control_struct_value.humidity_value);
	//当前光照度
	mcu_dp_value_update(DPID_ILLUMINATION,sensor_control_struct_value.illumination_value); //VALUE型数据上报;
	//当前水位
	mcu_dp_value_update(DPID_WATER,sensor_control_struct_value.water_stage_value); //VALUE型数据上报;
	//当前肥力
	mcu_dp_value_update(DPID_FERTILITY,sensor_control_struct_value.fertility_value); //VALUE型数据上报;
	//当前模式
	mcu_dp_enum_update(DPID_MODE,sensor_control_struct_value.mode_value); //枚举型数据上报;
	//浇水次数，每次上报1，服务器做统计，显示周/月统计曲线
//	mcu_dp_value_update(DPID_WATER_COUNT,sensor_control_struct_value.watering_count);
	//当前浇水次数
	mcu_dp_value_update(DPID_WATER_COUNT_DEVICE,sensor_control_struct_value.watering_count); //VALUE型数据上报;
	printf("watering_count=%d\r\n",sensor_control_struct_value.watering_count);
	//当前植物类型
	mcu_dp_value_update(DPID_TYPE,sensor_control_struct_value.botany_type); //VALUE型数据上报;
	//当前爱心值
	mcu_dp_value_update(DPID_LOVE_VALUE,sensor_control_struct_value.love_value); //VALUE型数据上报;
	//当前夜间报警
	mcu_dp_bool_update(DPID_REPORT_NIGHT,sensor_control_struct_value.report_night_value); //BOOL型数据上报;
	//当前温度安全值指针,当前温度安全值数据长度
	sensor_control_struct_value.public_int_value = 
	sprintf(sensor_control_struct_value.public_temp_char,"%d,%d",sensor_control_struct_value.safe_temperature[0],sensor_control_struct_value.safe_temperature[1]);
	mcu_dp_string_update(DPID_SAFE_TEMPERATURE,sensor_control_struct_value.public_temp_char,sensor_control_struct_value.public_int_value); //STRING型数据上报;
	//当前湿度安全值指针,当前湿度安全值数据长度
	sensor_control_struct_value.public_int_value = 
	sprintf(sensor_control_struct_value.public_temp_char,"%d,%d",sensor_control_struct_value.safe_humidity[0],sensor_control_struct_value.safe_humidity[1]);
	mcu_dp_string_update(DPID_SAFE_HUMIDITY,sensor_control_struct_value.public_temp_char,sensor_control_struct_value.public_int_value); //STRING型数据上报;
	//当前光照安全值指针,当前光照安全值数据长度
	sensor_control_struct_value.public_int_value = 
	sprintf(sensor_control_struct_value.public_temp_char,"%d,%d",sensor_control_struct_value.safe_illumination[0],sensor_control_struct_value.safe_illumination[1]);
	mcu_dp_string_update(DPID_SAFE_ILLUMINATION,sensor_control_struct_value.public_temp_char,sensor_control_struct_value.public_int_value); //STRING型数据上报;
	//当前肥力安全值指针,当前肥力安全值数据长度
	sensor_control_struct_value.public_int_value = 
	sprintf(sensor_control_struct_value.public_temp_char,"%d,%d",sensor_control_struct_value.safe_fertility[0],sensor_control_struct_value.safe_fertility[1]);
	mcu_dp_string_update(DPID_SAFE_FERTILITY,sensor_control_struct_value.public_temp_char,sensor_control_struct_value.public_int_value); //STRING型数据上报;
	//光照总量不足
	mcu_dp_value_update(DPID_LIGHT_LOW_LIMIT,sensor_control_struct_value.light_low_limit_value); //VALUE型数据上报;
	//当前植物开始日期指针,当前植物开始日期数据长度
	sensor_control_struct_value.public_int_value = 
	sprintf(sensor_control_struct_value.public_temp_char,"%d",sensor_control_struct_value.plant_value);
	mcu_dp_string_update(DPID_START_DATE,sensor_control_struct_value.public_temp_char,sensor_control_struct_value.public_int_value); //STRING型数据上报;
	//当前累计收获次数
	mcu_dp_value_update(DPID_CUMULATIVE_HARVEST,sensor_control_struct_value.cumulative_harvest_value); //VALUE型数据上报;
	//当前累计培育开始时间指针,当前累计培育开始时间数据长度
	sensor_control_struct_value.public_int_value = 
	sprintf(sensor_control_struct_value.public_temp_char,"%d",sensor_control_struct_value.begin_date_value);
	mcu_dp_string_update(DPID_BEGIN_DATE,sensor_control_struct_value.public_temp_char,sensor_control_struct_value.public_int_value); 
	//当前基质剩余量
	mcu_dp_value_update(DPID_MATRIX,sensor_control_struct_value.matrix_value); //VALUE型数据上报;
	//当前补光灯开启模式
	mcu_dp_enum_update(DPID_FILL_LIGHT_MODE,sensor_control_struct_value.fill_light_mode_value); //枚举型数据上报;
	//当前补光灯
	mcu_dp_bool_update(DPID_FILL_LIGHT,sensor_control_struct_value.fill_light_value); //BOOL型数据上报;
	
	//电池电量
	mcu_dp_value_update(DPID_SOC,sensor_control_struct_value.soc_value); //VALUE型数据上报;
	//空气湿度
  mcu_dp_value_update(DPID_HUMIDITY_ENVIRONMENT,sensor_control_struct_value.humidity1_value); //VALUE型数据上报;
	
    /*
    //此代码为平台自动生成，请按照实际数据修改每个可下发可上报函数和只上报函数
    //mcu_dp_bool_update(DPID_PUMP,当前浇水功能); //BOOL型数据上报;
    //mcu_dp_bool_update(DPID_LIGHT,当前指示灯开关); //BOOL型数据上报;
    //mcu_dp_value_update(DPID_TEMP_CURRENT,当前当前温度); //VALUE型数据上报;
    //mcu_dp_value_update(DPID_HUMIDITY_CURRENT,当前相对湿度); //VALUE型数据上报;
    mcu_dp_fault_update(DPID_FAULT,当前故障告警); //故障型数据上报;
    //mcu_dp_enum_update(DPID_MODE,当前模式); //枚举型数据上报;
    //mcu_dp_value_update(DPID_FERTILITY,当前肥力); //VALUE型数据上报;
    //mcu_dp_value_update(DPID_WATER,当前水位); //VALUE型数据上报;
    //mcu_dp_value_update(DPID_WATER_COUNT,当前浇水次数); //VALUE型数据上报;
    //mcu_dp_value_update(DPID_TYPE,当前植物类型); //VALUE型数据上报;
    //mcu_dp_value_update(DPID_ILLUMINATION,当前光照度); //VALUE型数据上报;
    //mcu_dp_string_update(DPID_START_DATE,当前植物开始日期指针,当前植物开始日期数据长度); //STRING型数据上报;
    //mcu_dp_value_update(DPID_MATRIX,当前基质剩余量); //VALUE型数据上报;
    //mcu_dp_value_update(DPID_LOVE_VALUE,当前爱心值); //VALUE型数据上报;
		//mcu_dp_value_update(DPID_WATER_COUNT_DEVICE,当前浇水次数); //VALUE型数据上报;
    //mcu_dp_value_update(DPID_CUMULATIVE_HARVEST,当前累计收获次数); //VALUE型数据上报;
    //mcu_dp_string_update(DPID_BEGIN_DATE,当前累计培育开始时间指针,当前累计培育开始时间数据长度); //STRING型数据上报;
    //mcu_dp_bool_update(DPID_MANUAL_REFRESH,当前手动刷新); //BOOL型数据上报;
    //mcu_dp_bool_update(DPID_REPORT_NIGHT,当前夜间报警); //BOOL型数据上报;
    //mcu_dp_string_update(DPID_SAFE_TEMPERATURE,当前温度安全值指针,当前温度安全值数据长度); //STRING型数据上报;
    //mcu_dp_string_update(DPID_SAFE_HUMIDITY,当前湿度安全值指针,当前湿度安全值数据长度); //STRING型数据上报;
    //mcu_dp_string_update(DPID_SAFE_ILLUMINATION,当前光照安全值指针,当前光照安全值数据长度); //STRING型数据上报;
    //mcu_dp_string_update(DPID_SAFE_FERTILITY,当前安全肥力指针,当前安全肥力数据长度); //STRING型数据上报;
    //mcu_dp_value_update(DPID_LIGHT_LOW_LIMIT,当前光照不足参数); //VALUE型数据上报;
    //mcu_dp_enum_update(DPID_FILL_LIGHT_MODE,当前补光灯开启模式); //枚举型数据上报;
    //mcu_dp_bool_update(DPID_FILL_LIGHT,当前补光灯); //BOOL型数据上报;
    */
}


/******************************************************************************
                                WARNING!!!    
                            2:所有数据上报处理
自动化代码模板函数,具体请用户自行实现数据处理
******************************************************************************/
//函数名称 : dp_download_switch_handle 
//功能描述 : 针对DPID_ON的处理函数 
//输入参数 : value:数据源数据 
//        : length:数据长度 
//返回参数 : 成功返回:SUCCESS/失败返回:ERROR 
//使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app 
//*****************************************************************************/ 
 
static unsigned char dp_download_switch_handle(const unsigned char value[], unsigned short length) 
{ 
    //示例:当前DP类型为BOOL 
    unsigned char ret; 
     
    unsigned char on; 
    on = mcu_get_dp_download_bool(value,length); 
	 
//		printf("\r\nfill_light_mode=%d\r\n",fill_light_mode); 
    switch(on) { 
        case 0: 
					FlashSetSwitch(0); 
					sensor_control_struct_value.on=0; 
					sensor_control_struct_value.on_copy=0; 
					sensor_light_set(0); 
					FlashSetLedState(0); 
				sensor_fill_light_set(0); 
				sensor_control_struct_value.fill_light_on = 0; 
				mcu_dp_bool_update(DPID_FILL_LIGHT,sensor_control_struct_value.fill_light_on); 
				mcu_dp_bool_update(DPID_LIGHT,sensor_control_struct_value.fill_light_on); 
					 
        break; 
         
        case 1: 
					FlashSetSwitch(1); 
					sensor_control_struct_value.on=1; 
					sensor_control_struct_value.on_copy=1; 
					sensor_light_set(1); 
					FlashSetLedState(1); 
				mcu_dp_bool_update(DPID_LIGHT,1); 
        break; 
         
        default: 
     
        break; 
    } 
     
    //处理完DP数据后应有反馈 
    ret = mcu_dp_bool_update(DPID_ON, on); 
    if(ret == SUCCESS) 
        return SUCCESS; 
    else 
        return ERROR; 
} 
/*****************************************************************************
函数名称 : dp_download_pump_handle
功能描述 : 针对DPID_PUMP的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_pump_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为BOOL
    unsigned char ret;
    //0:关/1:开
    unsigned char pump;
    
    pump = mcu_get_dp_download_bool(value,length);
    if(pump == 0) {
        //开关关
			sensor_pump_set(0);
			sensor_control_struct_value.pump_value=0;
			sensor_control_struct_value.pump_value_copy=0;
    }else {
        //开关开
			sensor_pump_set(1);
			sensor_control_struct_value.pump_value=1;
			sensor_control_struct_value.pump_value_copy=1;
    }
  
    //处理完DP数据后应有反馈
    ret = mcu_dp_bool_update(DPID_PUMP,pump);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_light_handle
功能描述 : 针对DPID_LIGHT的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_light_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为BOOL
    unsigned char ret;
    //0:关/1:开
    unsigned char light;

    light = mcu_get_dp_download_bool(value,length);
    if(light == 0) {
        //开关关
			sensor_light_set(0);
			FlashSetLedState(0);
    }else {
        //开关开
			sensor_light_set(1);
			FlashSetLedState(1);
    }
  
    //处理完DP数据后应有反馈
    ret = mcu_dp_bool_update(DPID_LIGHT,light);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_mode_handle
功能描述 : 针对DPID_MODE的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_mode_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为ENUM
    unsigned char ret;
    unsigned char mode;
    
    mode = mcu_get_dp_download_enum(value,length);
    switch(mode) {
        case 0:
					FlashSetModeAuto();
					sensor_control_struct_value.mode_value=0;
					sensor_control_struct_value.mode_value_copy=0;
        break;
        
        case 1:
					FlashSetModeManual();
					sensor_control_struct_value.mode_value=1;
					sensor_control_struct_value.mode_value_copy=1;
        break;
        
        default:
//					FlashSetModeAuto();
//					sensor_control_struct_value.mode_value=0;
        break;
    }
    
    //处理完DP数据后应有反馈
    ret = mcu_dp_enum_update(DPID_MODE, mode);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_type_handle
功能描述 : 针对DPID_TYPE的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_type_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为VALUE
    unsigned char ret;
    unsigned long type;
    
    type = mcu_get_dp_download_value(value,length);
		
		//种植植物
		if(type!=-1){
			//没有种植的状态
			if(sensor_control_struct_value.botany_type == -1){
				if(timestamp!=0){//需要有时间戳
					//存储种植的植物
					FlashSetBotanyType(type);
					sensor_control_struct_value.botany_type = type;
					//设置爱心值
					sensor_control_struct_value.love_value = sensor_control_struct_value.love_value + 20;
					FlashSetLoveValue(sensor_control_struct_value.love_value);
					//设置种植时间
					sensor_control_struct_value.plant_value = timestamp-28800;
					FlashSetDataPlant(sensor_control_struct_value.plant_value);
						//当前累计培育开始时间指针,当前累计培育开始时间数据长度
						sensor_control_struct_value.public_int_value = 
						sprintf(sensor_control_struct_value.public_temp_char,"%d",sensor_control_struct_value.begin_date_value);
				}
				else{
					
				}
			}
			else{//error
				type = sensor_control_struct_value.botany_type;
				printf("\r\n已经种植\r\n");
			}
		}
		else{//清除植物 / 收获植物
			//种植的状态
			if(sensor_control_struct_value.botany_type != -1){
				FlashSetBotanyType(type);
				sensor_control_struct_value.botany_type = type;
				//爱心值累加
				sensor_control_struct_value.love_value = sensor_control_struct_value.love_value + 20;
				FlashSetLoveValue(sensor_control_struct_value.love_value);
				//设置累计收获
				sensor_control_struct_value.cumulative_harvest_value++;
				FlashSetHarvestCount(sensor_control_struct_value.cumulative_harvest_value);
										//当前累计培育开始时间指针,当前累计培育开始时间数据长度
						sensor_control_struct_value.public_int_value = 
						sprintf(sensor_control_struct_value.public_temp_char,"%d",0);
			}
			else{//error
				type = sensor_control_struct_value.botany_type;
				printf("\r\n植物未种植\r\n");
			}
		}
		
    /*
    //VALUE类型数据处理
    
    */
        	sensor_control_struct_value.public_int_value =  
			sprintf(sensor_control_struct_value.public_temp_char,"%d",sensor_control_struct_value.plant_value); 
			mcu_dp_string_update(DPID_START_DATE,sensor_control_struct_value.public_temp_char,sensor_control_struct_value.public_int_value); //STRING型数据上报; 
    //处理完DP数据后应有反馈
    ret = mcu_dp_value_update(DPID_TYPE,type);
		mcu_dp_string_update(DPID_BEGIN_DATE,sensor_control_struct_value.public_temp_char,sensor_control_struct_value.public_int_value);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_matrix_handle
功能描述 : 针对DPID_MATRIX的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_matrix_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为VALUE
    unsigned char ret;
    unsigned long matrix;
    
    matrix = mcu_get_dp_download_value(value,length);
    /*
    //VALUE类型数据处理
    
    */
    sensor_control_struct_value.matrix_value = matrix;
		FlashSetMatrixData(sensor_control_struct_value.matrix_value);
    //处理完DP数据后应有反馈
    ret = mcu_dp_value_update(DPID_MATRIX,matrix);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_water_count_device_handle
功能描述 : 针对DPID_WATER_COUNT_DEVICE的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_water_count_device_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为VALUE
    unsigned char ret;
    unsigned long water_count_device;
    
    water_count_device = mcu_get_dp_download_value(value,length);
    /*
    //VALUE类型数据处理
    
    */
		sensor_control_struct_value.watering_count = water_count_device;
		FlashSetWaterCount(sensor_control_struct_value.watering_count);
    //处理完DP数据后应有反馈
    ret = mcu_dp_value_update(DPID_WATER_COUNT_DEVICE,water_count_device);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_cumulative_harvest_handle
功能描述 : 针对DPID_CUMULATIVE_HARVEST的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_cumulative_harvest_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为VALUE
    unsigned char ret;
    unsigned long cumulative_harvest;
    //累积的收获
    cumulative_harvest = mcu_get_dp_download_value(value,length);
    /*
    //VALUE类型数据处理
    
    */
    sensor_control_struct_value.cumulative_harvest_value = cumulative_harvest;
		FlashSetHarvestCount(sensor_control_struct_value.cumulative_harvest_value);
    //处理完DP数据后应有反馈
    ret = mcu_dp_value_update(DPID_CUMULATIVE_HARVEST,cumulative_harvest);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_begin_date_handle
功能描述 : 针对DPID_BEGIN_DATE的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_begin_date_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为STRING
    unsigned char ret;
	
	unsigned int len =0;
	unsigned int value1;
	if(length < public_temp_char_len){//缓存可以存下
		memset(sensor_control_struct_value.public_temp_char,0,public_temp_char_len);
		
		for(len=0;len<length;len++){//拷贝数据
			sensor_control_struct_value.public_temp_char[len] = value[len];
		}
		printf("begin_date=%s\r\n",sensor_control_struct_value.public_temp_char);
		value1 = atoi(sensor_control_struct_value.public_temp_char);
		sensor_control_struct_value.begin_date_value = value1;
		FlashSetOneDataPlant(sensor_control_struct_value.begin_date_value);
	}
	else{//缓存不够,错误
		
	}
	
	
    /*
    //STRING类型数据处理
    unsigned char string_data[8];
    
    string_data[0] = value[0];
    string_data[1] = value[1];
    string_data[2] = value[2];
    string_data[3] = value[3];
    string_data[4] = value[4];
    string_data[5] = value[5];
    string_data[6] = value[6];
    string_data[7] = value[7];
    */
    
    //处理完DP数据后应有反馈
    ret = mcu_dp_string_update(DPID_BEGIN_DATE,value, length);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_manual_refresh_handle
功能描述 : 针对DPID_MANUAL_REFRESH的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_manual_refresh_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为BOOL
    unsigned char ret;
    //0:关/1:开
    unsigned char manual_refresh;
    
    manual_refresh = mcu_get_dp_download_bool(value,length);
    if(manual_refresh == 0) {
        //开关关
    }else {
        //开关开
			all_data_update();
    }
  printf("\r\nmanual_refresh=%d\r\n",manual_refresh);
    //处理完DP数据后应有反馈
    ret = mcu_dp_bool_update(DPID_MANUAL_REFRESH,manual_refresh);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_report_night_handle
功能描述 : 针对DPID_REPORT_NIGHT的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_report_night_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为BOOL
    unsigned char ret;
    //0:关/1:开
    unsigned char report_night;
    
    report_night = mcu_get_dp_download_bool(value,length);
    if(report_night == 0) {
        //开关关
			FlashSetReportEvent(0);
			sensor_control_struct_value.report_night_value = 0;
    }else {
        //开关开
			FlashSetReportEvent(1);
			sensor_control_struct_value.report_night_value = 1;
    }
  
    //处理完DP数据后应有反馈
    ret = mcu_dp_bool_update(DPID_REPORT_NIGHT,report_night);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_safe_temperature_handle
功能描述 : 针对DPID_SAFE_TEMPERATURE的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_safe_temperature_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为STRING
    unsigned char ret;
    
    //STRING类型数据处理
	int len =0;
	int value1,value2;
	
	PutData(&rb_t_usart1_send,"\r\n1111111111\r\n",12);
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	
	if(length < public_temp_char_len){//缓存可以存下
		memset(sensor_control_struct_value.public_temp_char,0,public_temp_char_len);
		
		for(len=0;len<length;len++){//拷贝数据
			sensor_control_struct_value.public_temp_char[len] = value[len];
		}
//		printf("\r\nsafe_temperature=%s\r\n",sensor_control_struct_value.public_temp_char);

		len = split((char*)sensor_control_struct_value.public_temp_char,",",split_buff,MAXList);//
		if(len == 2){
			value1 = atoi(split_buff[0]);
			value2 = atoi(split_buff[1]);
			
			sensor_control_struct_value.safe_temperature[0] = value1;
			sensor_control_struct_value.safe_temperature[1] = value2;
			
			FlashSetSafe(sensor_control_struct_value.safe_temperature[0],sensor_control_struct_value.safe_temperature[1],safe_type_temperature);
			
//			printf("\r\nsafe_temperature=%d,%d\r\n",sensor_control_struct_value.safe_temperature[0],sensor_control_struct_value.safe_temperature[1]);
		}
		else{//数据解析错误
		
		}
	}
	else{//缓存不够,错误
		
	}
    
    //处理完DP数据后应有反馈
    ret = mcu_dp_string_update(DPID_SAFE_TEMPERATURE,value, length);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_safe_humidity_handle
功能描述 : 针对DPID_SAFE_HUMIDITY的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_safe_humidity_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为STRING
    unsigned char ret;
	
    //STRING类型数据处理
	int len =0;
	int value1,value2;
	
	
	PutData(&rb_t_usart1_send,"\r\nhumidity\r\n",12);
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	
	if(length < public_temp_char_len){//缓存可以存下
		memset(sensor_control_struct_value.public_temp_char,0,public_temp_char_len);
		
		for(len=0;len<length;len++){//拷贝数据
			sensor_control_struct_value.public_temp_char[len] = value[len];
		}
//		printf("\r\nsafe_humidity=%s\r\n",sensor_control_struct_value.public_temp_char);
		
		len = split((char*)sensor_control_struct_value.public_temp_char,",",split_buff,MAXList);//
		if(len == 2){
			value1 = atoi(split_buff[0]);
			value2 = atoi(split_buff[1]);
			
			sensor_control_struct_value.safe_humidity[0] = value1;
			sensor_control_struct_value.safe_humidity[1] = value2;
			
			FlashSetSafe(sensor_control_struct_value.safe_humidity[0],sensor_control_struct_value.safe_humidity[1],safe_type_humidity);
			
//			printf("\r\nsafe_humidity=%d,%d\r\n",sensor_control_struct_value.safe_humidity[0],sensor_control_struct_value.safe_humidity[1]);
		}
		else{//数据解析错误
		
		}
	}
	else{//缓存不够,错误
		
	}
    
    //处理完DP数据后应有反馈
    ret = mcu_dp_string_update(DPID_SAFE_HUMIDITY,value, length);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_safe_illumination_handle
功能描述 : 针对DPID_SAFE_ILLUMINATION的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_safe_illumination_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为STRING
    unsigned char ret;
    
    //STRING类型数据处理
	int len =0;
	int value1,value2;
	
	PutData(&rb_t_usart1_send,"\r\n0000000000\r\n",12);
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	
	if(length < public_temp_char_len){//缓存可以存下
		memset(sensor_control_struct_value.public_temp_char,0,public_temp_char_len);
		
		for(len=0;len<length;len++){//拷贝数据
			sensor_control_struct_value.public_temp_char[len] = value[len];
		}
//		printf("\r\nsafe_illumination=%s\r\n",sensor_control_struct_value.public_temp_char);

		len = split((char*)sensor_control_struct_value.public_temp_char,",",split_buff,MAXList);//
		if(len == 2){
			value1 = atoi(split_buff[0]);
			value2 = atoi(split_buff[1]);
			
			sensor_control_struct_value.safe_illumination[0] = value1;
			sensor_control_struct_value.safe_illumination[1] = value2;
			FlashSetSafe(sensor_control_struct_value.safe_illumination[0],sensor_control_struct_value.safe_illumination[1],safe_type_illumination);
//			printf("\r\nsafe_illumination=%d,%d\r\n",sensor_control_struct_value.safe_illumination[0],sensor_control_struct_value.safe_illumination[1]);
		}
		else{//数据解析错误
		
		}
	}
	else{//缓存不够,错误
		
	}
    //处理完DP数据后应有反馈
    ret = mcu_dp_string_update(DPID_SAFE_ILLUMINATION,value, length);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_safe_fertility_handle
功能描述 : 针对DPID_SAFE_FERTILITY的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_safe_fertility_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为STRING
    unsigned char ret;
    
    //STRING类型数据处理
	int len =0;
	int value1,value2;
	
	PutData(&rb_t_usart1_send,"\r\n2222222222\r\n",12);
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	
	if(length < public_temp_char_len){//缓存可以存下
		memset(sensor_control_struct_value.public_temp_char,0,public_temp_char_len);
		
		for(len=0;len<length;len++){//拷贝数据
			sensor_control_struct_value.public_temp_char[len] = value[len];
		}
//		printf("\r\nsafe_fertility=%s\r\n",sensor_control_struct_value.public_temp_char);

		len = split((char*)sensor_control_struct_value.public_temp_char,",",split_buff,MAXList);//
		if(len == 2){
			value1 = atoi(split_buff[0]);
			value2 = atoi(split_buff[1]);
			
			sensor_control_struct_value.safe_fertility[0] = value1;
			sensor_control_struct_value.safe_fertility[1] = value2;
			FlashSetSafe(sensor_control_struct_value.safe_fertility[0],sensor_control_struct_value.safe_fertility[1],safe_type_fertility);
//			printf("\r\nsafe_fertility=%d,%d\r\n",sensor_control_struct_value.safe_fertility[0],sensor_control_struct_value.safe_fertility[1]);
		}
		else{//数据解析错误
		
		}
	}
	else{//缓存不够,错误
		
	}
    
    //处理完DP数据后应有反馈
    ret = mcu_dp_string_update(DPID_SAFE_FERTILITY,value, length);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_light_low_limit_handle
功能描述 : 针对DPID_LIGHT_LOW_LIMIT的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_light_low_limit_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为VALUE
    unsigned char ret;
    unsigned long light_low_limit;
    
	PutData(&rb_t_usart1_send,"\r\n3333333333\r\n",12);
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	
    light_low_limit = mcu_get_dp_download_value(value,length);
    /*
    //VALUE类型数据处理
    
    */
	sensor_control_struct_value.light_low_limit_value = light_low_limit;
	FlashSetLightAlarm(sensor_control_struct_value.light_low_limit_value);
//	printf("\r\nlight_low_limit_value=%d\r\n",sensor_control_struct_value.light_low_limit_value);
    //处理完DP数据后应有反馈
    ret = mcu_dp_value_update(DPID_LIGHT_LOW_LIMIT,light_low_limit);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}

/*****************************************************************************
函数名称 : dp_download_fill_light_mode_handle
功能描述 : 针对DPID_FILL_LIGHT_MODE的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_fill_light_mode_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为ENUM
    unsigned char ret;
    unsigned char fill_light_mode;
    
    fill_light_mode = mcu_get_dp_download_enum(value,length);
	
//		printf("\r\nfill_light_mode=%d\r\n",fill_light_mode);
    switch(fill_light_mode) {
        case 0:
					FlashSetFillLightMode(0);
					sensor_control_struct_value.fill_light_mode_value=0;
        break;
        
        case 1:
					FlashSetFillLightMode(1);
					sensor_control_struct_value.fill_light_mode_value=1;
        break;
        
        default:
    
        break;
    }
    
    //处理完DP数据后应有反馈
    ret = mcu_dp_enum_update(DPID_FILL_LIGHT_MODE, fill_light_mode);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_fill_light_handle
功能描述 : 针对DPID_FILL_LIGHT的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_fill_light_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为BOOL
    unsigned char ret;
    //0:关/1:开
    unsigned char fill_light;
    
    fill_light = mcu_get_dp_download_bool(value,length);
//		printf("\r\nfill_light=%d\r\n",fill_light);
    if(fill_light == 0) {
        //开关关
			sensor_fill_light_set(0);
    }else {
        //开关开
			sensor_fill_light_set(1);
    }
  
    //处理完DP数据后应有反馈
    ret = mcu_dp_bool_update(DPID_FILL_LIGHT,fill_light);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}

/*****************************************************************************
函数名称 : dp_download_soc_handle
功能描述 : 针对DPID_SOC的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_soc_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为VALUE
    unsigned char ret;
    unsigned long soc;
    
    soc = mcu_get_dp_download_value(value,length);
    /*
    //VALUE类型数据处理
    
    */
    
    //处理完DP数据后应有反馈
    ret = mcu_dp_value_update(DPID_SOC,soc);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}

/*****************************************************************************
函数名称 : dp_download_humidity_environment_handle
功能描述 : 针对DPID_HUMIDITY_ENVIRONMENT的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_humidity_environment_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为VALUE
    unsigned char ret;
    unsigned long humidity_environment;
    
    humidity_environment = mcu_get_dp_download_value(value,length);
    /*
    //VALUE类型数据处理
    
    */
    
    //处理完DP数据后应有反馈
    ret = mcu_dp_value_update(DPID_HUMIDITY_ENVIRONMENT,humidity_environment);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}

/******************************************************************************
                                WARNING!!!                     
此部分函数用户请勿修改!!
******************************************************************************/

/**
 * @brief  dp下发处理函数
 * @param[in] {dpid} dpid 序号
 * @param[in] {value} dp数据缓冲区地址
 * @param[in] {length} dp数据长度
 * @return dp处理结果
 * -           0(ERROR): 失败
 * -           1(SUCCESS): 成功
 * @note   该函数用户不能修改
 */
unsigned char dp_download_handle(unsigned char dpid,const unsigned char value[], unsigned short length)
{
    /*********************************
    当前函数处理可下发/可上报数据调用                    
    具体函数内需要实现下发数据处理
    完成用需要将处理结果反馈至APP端,否则APP会认为下发失败
    ***********************************/
    unsigned char ret;
    switch(dpid) {
        case DPID_ON: 
            //浇水功能处理函数 
            ret = dp_download_switch_handle(value,length); 
        break; 
        case DPID_PUMP:
            //浇水功能处理函数
            ret = dp_download_pump_handle(value,length);
        break;
        case DPID_LIGHT:
            //指示灯开关处理函数
            ret = dp_download_light_handle(value,length);
        break;
        case DPID_MODE:
            //模式处理函数
            ret = dp_download_mode_handle(value,length);
        break;
        case DPID_TYPE:
            //植物类型处理函数
            ret = dp_download_type_handle(value,length);
        break;
        case DPID_MATRIX:
            //基质剩余量处理函数
            ret = dp_download_matrix_handle(value,length);
        break;
        case DPID_WATER_COUNT_DEVICE:
            //浇水次数处理函数
            ret = dp_download_water_count_device_handle(value,length);
        break;
        case DPID_CUMULATIVE_HARVEST:
            //累计收获次数处理函数
            ret = dp_download_cumulative_harvest_handle(value,length);
        break;
        case DPID_BEGIN_DATE:
            //累计培育开始时间处理函数
            ret = dp_download_begin_date_handle(value,length);
        break;
        case DPID_MANUAL_REFRESH:
            //手动刷新处理函数
            ret = dp_download_manual_refresh_handle(value,length);
        break;
        case DPID_REPORT_NIGHT:
            //夜间报警处理函数
            ret = dp_download_report_night_handle(value,length);
        break;
        case DPID_SAFE_TEMPERATURE:
            //温度安全值处理函数
            ret = dp_download_safe_temperature_handle(value,length);
        break;
        case DPID_SAFE_HUMIDITY:
            //湿度安全值处理函数
            ret = dp_download_safe_humidity_handle(value,length);
        break;
        case DPID_SAFE_ILLUMINATION:
            //光照安全值处理函数
            ret = dp_download_safe_illumination_handle(value,length);
        break;
        case DPID_SAFE_FERTILITY:
            //安全肥力处理函数
            ret = dp_download_safe_fertility_handle(value,length);
        break;
        case DPID_LIGHT_LOW_LIMIT:
            //光照不足参数处理函数
            ret = dp_download_light_low_limit_handle(value,length);
        break;
				case DPID_FILL_LIGHT_MODE:
            //补光灯开启模式处理函数
            ret = dp_download_fill_light_mode_handle(value,length);
        break;
        case DPID_FILL_LIGHT:
            //补光灯处理函数
            ret = dp_download_fill_light_handle(value,length);
        break;
				case DPID_SOC:
            //剩余电量处理函数
            ret = dp_download_soc_handle(value,length);
        break;
        case DPID_HUMIDITY_ENVIRONMENT:
            //环境湿度处理函数
            ret = dp_download_humidity_environment_handle(value,length);
        break;
        
        default:
        break;
    }
    return ret;
}

/**
 * @brief  获取所有dp命令总和
 * @param[in] Null
 * @return 下发命令总和
 * @note   该函数用户不能修改
 */
unsigned char get_download_cmd_total(void)
{
    return(sizeof(download_cmd) / sizeof(download_cmd[0]));
}


/******************************************************************************
                                WARNING!!!                     
此代码为SDK内部调用,请按照实际dp数据实现函数内部数据
******************************************************************************/

#ifdef SUPPORT_MCU_FIRM_UPDATE
/**
 * @brief  升级包大小选择
 * @param[in] {package_sz} 升级包大小
 * @ref           0x00: 256byte (默认)
 * @ref           0x01: 512byte
 * @ref           0x02: 1024byte
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void upgrade_package_choose(unsigned char package_sz)
{
//    #error "请自行实现请自行实现升级包大小选择代码,完成后请删除该行"
    unsigned short send_len = 0;
    send_len = set_wifi_uart_byte(send_len, package_sz);
    wifi_uart_write_frame(UPDATE_START_CMD, MCU_TX_VER, send_len);
}

/**
 * @brief  MCU进入固件升级模式
 * @param[in] {value} 固件缓冲区
 * @param[in] {position} 当前数据包在于固件位置
 * @param[in] {length} 当前固件包长度(固件包长度为0时,表示固件包发送完成)
 * @return Null
 * @note   MCU需要自行实现该功能
 */
unsigned char mcu_firm_update_handle(const unsigned char value[],unsigned long position,unsigned short length)
{
	unsigned long addr;
	u16 ReadDat[2];

	printf("update...\r\n");
	
    if(length == 0) {
			delay_ms(10);
        //固件数据发送完成
      //固件数据发送完成
			
			//写入升级标志
			ReadDat[0]=0x5555;
			ReadDat[1]=0x5555;
			STMFLASH_Write(FLASH_UPDATE_FLAGE_ADDR,ReadDat,2);
			
			STMFLASH_Read(FLASH_UPDATE_FLAGE_ADDR,ReadDat,2);
			if(ReadDat[0] != 0x5555 || ReadDat[1] != 0x5555){
				return ERROR;
			}
			
			Reset_MCU();
//			Reset();
    }else {
      //固件数据处理
			addr = FIREWARE_ADDR_H;
			mcu_updata_flag = 1;
			mcu_updata_flag_clear_cnt=0;
			if(position % 1024 == 0)
			{
				if(Earse_Flash(addr + position) == ERROR){
					printf("error111\r\n");
					return ERROR;
				}
			}
			
			if(Write_Flash(addr + position,(unsigned char *)value,length) == ERROR){
				printf("error\r\n");
				return ERROR;
			}
		}
    
//			printf("SUCCESS\r\n");
    return SUCCESS;
}
#endif

#ifdef SUPPORT_GREEN_TIME
/**
 * @brief  获取到的格林时间
 * @param[in] {time} 获取到的格林时间数据
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void mcu_get_greentime(unsigned char time[])
{
//	int year=20;
//	struct tm tm_struct;//
//    #error "请自行完成相关代码,并删除该行"
    /*
    time[0] 为是否获取时间成功标志，为 0 表示失败，为 1表示成功
    time[1] 为年份，0x00 表示 2000 年
    time[2] 为月份，从 1 开始到12 结束
    time[3] 为日期，从 1 开始到31 结束
    time[4] 为时钟，从 0 开始到23 结束
    time[5] 为分钟，从 0 开始到59 结束
    time[6] 为秒钟，从 0 开始到59 结束
    */
//    if(time[0] == 1) {
//			//正确接收到wifi模块返回的格林数据
//			if(1){//需要改变前两位时间
//				
//			}
//			year = year*100 + time[1];
//			year = year - 1900;
//			
//			tm_struct.tm_year = year;
//			tm_struct.tm_mon = time[2] - 1;
//			tm_struct.tm_mday = time[3];
//			tm_struct.tm_hour = time[4];
//			tm_struct.tm_min = time[5];
//			tm_struct.tm_sec = time[6];
//			
//			timestamp = mktime(&tm_struct);//获取时间戳
//			
//			printf("\r\ntime=%d,%d,%d,%d,%d,%d\r\n",year+1900,time[2],time[3],time[4],time[5],time[6]);
//			
//    }else {
//        //获取格林时间出错,有可能是当前wifi模块未联网
//    }
}
#endif

#ifdef SUPPORT_MCU_RTC_CHECK
/**
 * @brief  MCU校对本地RTC时钟
 * @param[in] {time} 获取到的格林时间数据
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void mcu_write_rtctime(unsigned char time[])
{
	int year=20;
	struct tm tm_struct;//
//    #error "请自行完成RTC时钟写入代码,并删除该行"
    /*
    Time[0] 为是否获取时间成功标志，为 0 表示失败，为 1表示成功
    Time[1] 为年份，0x00 表示 2000 年
    Time[2] 为月份，从 1 开始到12 结束
    Time[3] 为日期，从 1 开始到31 结束
    Time[4] 为时钟，从 0 开始到23 结束
    Time[5] 为分钟，从 0 开始到59 结束
    Time[6] 为秒钟，从 0 开始到59 结束
    Time[7] 为星期，从 1 开始到 7 结束，1代表星期一
   */
    if(time[0] == 1) {
        //正确接收到wifi模块返回的本地时钟数据
//			printf("\r\n%d,%d,%d,%d,%d,%d,%d\r\n",time[0],time[1],time[2],time[3],time[4],time[5],time[6]);
			
			if(1){//需要改变前两位时间
				
			}
			year = year*100 + time[1];
			year = year - 1900;
			
			tm_struct.tm_year = year;
			tm_struct.tm_mon = time[2] - 1;
			tm_struct.tm_mday = time[3];
			tm_struct.tm_hour = time[4];
			tm_struct.tm_min = time[5];
			tm_struct.tm_sec = time[6];
			
			timestamp = mktime(&tm_struct);//获取时间戳
			
			printf("\r\ntime=%d,%d,%d,%d,%d,%d\r\n",year+1900,time[2],time[3],time[4],time[5],time[6]);
			
    }else {
        //获取本地时钟数据出错,有可能是当前wifi模块未联网
    }
}
#endif

#ifdef WIFI_TEST_ENABLE
/**
 * @brief  wifi功能测试反馈
 * @param[in] {result} wifi功能测试结果
 * @ref       0: 失败
 * @ref       1: 成功
 * @param[in] {rssi} 测试成功表示wifi信号强度/测试失败表示错误类型
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void wifi_test_result(unsigned char result,unsigned char rssi)
{
    #error "请自行实现wifi功能测试成功/失败代码,完成后请删除该行"
    if(result == 0) {
        //测试失败
        if(rssi == 0x00) {
            //未扫描到名称为tuya_mdev_test路由器,请检查
        }else if(rssi == 0x01) {
            //模块未授权
        }
    }else {
        //测试成功
        //rssi为信号强度(0-100, 0信号最差，100信号最强)
    }
}
#endif

#ifdef WEATHER_ENABLE
/**
* @brief  mcu打开天气服务
 * @param  Null
 * @return Null
 */
void mcu_open_weather(void)
{
    int i = 0;
    char buffer[13] = {0};
    unsigned char weather_len = 0;
    unsigned short send_len = 0;
    
    weather_len = sizeof(weather_choose) / sizeof(weather_choose[0]);
      
    for(i=0;i<weather_len;i++) {
        buffer[0] = sprintf(buffer+1,"w.%s",weather_choose[i]);
        send_len = set_wifi_uart_buffer(send_len, (unsigned char *)buffer, buffer[0]+1);
    }
    
    #error "请根据提示，自行完善打开天气服务代码，完成后请删除该行"
    /*
    //当获取的参数有和时间有关的参数时(如:日出日落)，需要搭配t.unix或者t.local使用，需要获取的参数数据是按照格林时间还是本地时间
    buffer[0] = sprintf(buffer+1,"t.unix"); //格林时间   或使用  buffer[0] = sprintf(buffer+1,"t.local"); //本地时间
    send_len = set_wifi_uart_buffer(send_len, (unsigned char *)buffer, buffer[0]+1);
    */
    
    buffer[0] = sprintf(buffer+1,"w.date.%d",WEATHER_FORECAST_DAYS_NUM);
    send_len = set_wifi_uart_buffer(send_len, (unsigned char *)buffer, buffer[0]+1);
    
    wifi_uart_write_frame(WEATHER_OPEN_CMD, MCU_TX_VER, send_len);
}

/**
 * @brief  打开天气功能返回用户自处理函数
 * @param[in] {res} 打开天气功能返回结果
 * @ref       0: 失败
 * @ref       1: 成功
 * @param[in] {err} 错误码
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void weather_open_return_handle(unsigned char res, unsigned char err)
{
    #error "请自行完成打开天气功能返回数据处理代码,完成后请删除该行"
    unsigned char err_num = 0;
    
    if(res == 1) {
        //打开天气返回成功
    }else if(res == 0) {
        //打开天气返回失败
        //获取错误码
        err_num = err; 
    }
}

/**
 * @brief  天气数据用户自处理函数
 * @param[in] {name} 参数名
 * @param[in] {type} 参数类型
 * @ref       0: int 型
 * @ref       1: string 型
 * @param[in] {data} 参数值的地址
 * @param[in] {day} 哪一天的天气  0:表示当天 取值范围: 0~6
 * @ref       0: 今天
 * @ref       1: 明天
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void weather_data_user_handle(char *name, unsigned char type, const unsigned char *data, char day)
{
    #error "这里仅给出示例，请自行完善天气数据处理代码,完成后请删除该行"
    int value_int;
    char value_string[50];//由于有的参数内容较多，这里默认为50。您可以根据定义的参数，可以适当减少该值
    
    my_memset(value_string, '\0', 50);
    
    //首先获取数据类型
    if(type == 0) { //参数是INT型
        value_int = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
    }else if(type == 1) {
        my_strcpy(value_string, data);
    }
    
    //注意要根据所选参数类型来获得参数值！！！
    if(my_strcmp(name, "temp") == 0) {
        printf("day:%d temp value is:%d\r\n", day, value_int);          //int 型
    }else if(my_strcmp(name, "humidity") == 0) {
        printf("day:%d humidity value is:%d\r\n", day, value_int);      //int 型
    }else if(my_strcmp(name, "pm25") == 0) {
        printf("day:%d pm25 value is:%d\r\n", day, value_int);          //int 型
    }else if(my_strcmp(name, "condition") == 0) {
        printf("day:%d condition value is:%s\r\n", day, value_string);  //string 型
    }
}
#endif

#ifdef MCU_DP_UPLOAD_SYN
/**
 * @brief  状态同步上报结果
 * @param[in] {result} 结果
 * @ref       0: 失败
 * @ref       1: 成功
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void get_upload_syn_result(unsigned char result)
{
    #error "请自行完成状态同步上报结果代码,并删除该行"
      
    if(result == 0) {
        //同步上报出错
    }else {
        //同步上报成功
    }
}
#endif

#ifdef GET_WIFI_STATUS_ENABLE
/**
 * @brief  获取 WIFI 状态结果
 * @param[in] {result} 指示 WIFI 工作状态
 * @ref       0x00: wifi状态 1 smartconfig 配置状态
 * @ref       0x01: wifi状态 2 AP 配置状态
 * @ref       0x02: wifi状态 3 WIFI 已配置但未连上路由器
 * @ref       0x03: wifi状态 4 WIFI 已配置且连上路由器
 * @ref       0x04: wifi状态 5 已连上路由器且连接到云端
 * @ref       0x05: wifi状态 6 WIFI 设备处于低功耗模式
 * @ref       0x06: wifi状态 7 WIFI 设备处于smartconfig&AP配置状态
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void get_wifi_status(unsigned char result)
{
//  #error "请自行完成获取 WIFI 状态结果代码,并删除该行"
	wifi_state = result;
    switch(result) {
        case 0:
					printf("smartconfig\r\n");
            //wifi工作状态1
        break;
    
        case 1:
					printf("AP config\r\n");
            //wifi工作状态2
        break;
        
        case 2:
					printf("config no connected\r\n");
            //wifi工作状态3
        break;
        
        case 3:
					printf("config and connected\r\n");
            //wifi工作状态4
        break;
        
        case 4:
					printf("connected and cloud\r\n");
            //wifi工作状态5
        break;
        
        case 5:
					printf("low power\r\n");
            //wifi工作状态6
        break;
      
        case 6:
					printf("smartconfig&AP\r\n");
            //wifi工作状态7
        break;
        
        default:break;
    }
}
#endif

#ifdef WIFI_STREAM_ENABLE
/**
 * @brief  流服务发送结果
 * @param[in] {result} 结果
 * @ref       0x00: 成功
 * @ref       0x01: 流服务功能未开启
 * @ref       0x02: 流服务器未连接成功
 * @ref       0x03: 数据推送超时
 * @ref       0x04: 传输的数据长度错误
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void stream_trans_send_result(unsigned char result)
{
    #error "这里仅给出示例，请自行完善流服务发送结果处理代码,完成后请删除该行"
    switch(result) {
        case 0x00:
            //成功
        break;
        
        case 0x01:
            //流服务功能未开启
        break;
        
        case 0x02:
            //流服务器未连接成功
        break;
        
        case 0x03:
            //数据推送超时
        break;
        
        case 0x04:
            //传输的数据长度错误
        break;
        
        default:break;
    }
}

/**
 * @brief  多地图流服务发送结果
 * @param[in] {result} 结果
 * @ref       0x00: 成功
 * @ref       0x01: 失败
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void maps_stream_trans_send_result(unsigned char result)
{
    #error "这里仅给出示例，请自行完善多地图流服务发送结果处理代码,完成后请删除该行"
    switch(result) {
        case 0x00:
            //成功
        break;
        
        case 0x01:
            //失败
        break;
        
        default:break;
    }
}
#endif

#ifdef WIFI_CONNECT_TEST_ENABLE
/**
 * @brief  路由信息接收结果通知
 * @param[in] {result} 模块是否成功接收到正确的路由信息
 * @ref       0x00: 失败
 * @ref       0x01: 成功
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void wifi_connect_test_result(unsigned char result)
{
    #error "请自行实现wifi功能测试成功/失败代码,完成后请删除该行"
    if(result == 0) {
        //路由信息接收失败，请检查发出的路由信息包是否是完整的JSON数据包
    }else {
        //路由信息接收成功，产测结果请注意WIFI_STATE_CMD指令的wifi工作状态
    }
}
#endif

#ifdef GET_MODULE_MAC_ENABLE
/**
 * @brief  获取模块mac结果
 * @param[in] {mac} 模块 MAC 数据
 * @ref       mac[0]: 为是否获取mac成功标志，0x00 表示成功，0x01 表示失败
 * @ref       mac[1]~mac[6]: 当获取 MAC地址标志位如果mac[0]为成功，则表示模块有效的MAC地址
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void mcu_get_mac(unsigned char mac[])
{
    #error "请自行完成mac获取代码,并删除该行"
    /*
    mac[0]为是否获取mac成功标志，0x00 表示成功，为0x01表示失败
    mac[1]~mac[6]:当获取 MAC地址标志位如果mac[0]为成功，则表示模块有效的MAC地址
   */
   
    if(mac[0] == 1) {
        //获取mac出错
    }else {
        //正确接收到wifi模块返回的mac地址
    }
}
#endif

#ifdef GET_IR_STATUS_ENABLE
/**
 * @brief  获取红外状态结果
 * @param[in] {result} 指示红外状态
 * @ref       0x00: 红外状态 1 正在发送红外码
 * @ref       0x01: 红外状态 2 发送红外码结束
 * @ref       0x02: 红外状态 3 红外学习开始
 * @ref       0x03: 红外状态 4 红外学习结束
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void get_ir_status(unsigned char result)
{
    #error "请自行完成红外状态代码,并删除该行"
    switch(result) {
        case 0:
            //红外状态 1
        break;
      
        case 1:
            //红外状态 2
        break;
          
        case 2:
            //红外状态 3
        break;
          
        case 3:
            //红外状态 4
        break;
          
        default:break;
    }
    
    wifi_uart_write_frame(GET_IR_STATUS_CMD, MCU_TX_VER, 0);
}
#endif

#ifdef IR_TX_RX_TEST_ENABLE
/**
 * @brief  红外进入收发产测结果通知
 * @param[in] {result} 模块是否成功接收到正确的信息
 * @ref       0x00: 失败
 * @ref       0x01: 成功
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void ir_tx_rx_test_result(unsigned char result)
{
    #error "请自行实现红外进入收发产测功能测试成功/失败代码,完成后请删除该行"
    if(result == 0) {
        //红外进入收发产测成功
    }else {
        //红外进入收发产测失败，请检查发出的数据包
    }
}
#endif

#ifdef FILE_DOWNLOAD_ENABLE
/**
 * @brief  文件下载包大小选择
 * @param[in] {package_sz} 文件下载包大小
 * @ref       0x00: 256 byte (默认)
 * @ref       0x01: 512 byte
 * @ref       0x02: 1024 byte
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void file_download_package_choose(unsigned char package_sz)
{
    #error "请自行实现请自行实现文件下载包大小选择代码,完成后请删除该行"
    unsigned short send_len = 0;
    send_len = set_wifi_uart_byte(send_len, package_sz);
    wifi_uart_write_frame(FILE_DOWNLOAD_START_CMD, MCU_TX_VER, send_len);
}

/**
 * @brief  文件包下载模式
 * @param[in] {value} 数据缓冲区
 * @param[in] {position} 当前数据包在于文件位置
 * @param[in] {length} 当前文件包长度(长度为0时,表示文件包发送完成)
 * @return 数据处理结果
 * -           0(ERROR): 失败
 * -           1(SUCCESS): 成功
 * @note   MCU需要自行实现该功能
 */
unsigned char file_download_handle(const unsigned char value[],unsigned long position,unsigned short length)
{
    #error "请自行完成文件包下载代码,完成后请删除该行"
    if(length == 0) {
        //文件包数据发送完成
        
    }else {
        //文件包数据处理
      
    }
    
    return SUCCESS;
}
#endif

#ifdef MODULE_EXPANDING_SERVICE_ENABLE
/**
 * @brief  打开模块时间服务通知结果
 * @param[in] {value} 数据缓冲区
 * @param[in] {length} 数据长度
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void open_module_time_serve_result(const unsigned char value[], unsigned short length)
{
    #error "请自行实现模块时间服务通知结果代码,完成后请删除该行"
    unsigned char sub_cmd = value[0];
    
    switch(sub_cmd) {
        case 0x01: { //子命令  打开模块时间服务通知
            if(0x02 != length) {
                //数据长度错误
                return;
            }
            
            if(value[1] == 0) {
                //服务开启成功
            }else {
                //服务开启失败
            }
        }
        break;
        
        case 0x02: {  //子命令  模块时间服务通知
            if(0x09 != length) {
                //数据长度错误
                return;
            }
            
            unsigned char time_type = value[1]; //0x00:格林时间  0x01:本地时间
            unsigned char time_data[7];
            
            my_memcpy(time_data, value + 2, length - 2);
            /*
            Data[0]为年份, 0x00表示2000年
            Data[1]为月份，从1开始到12结束
            Data[2]为日期，从1开始到31结束
            Data[3]为时钟，从0开始到23结束
            Data[4]为分钟，从0开始到59结束
            Data[5]为秒钟，从0开始到15结束
            Data[6]为星期，从1开始到7结束，1代表星期一
            */
            
            //在此处添加时间数据处理代码，time_type为时间类型
            
            unsigned short send_len = 0;
            send_len = set_wifi_uart_byte(send_len,sub_cmd);
            wifi_uart_write_frame(MODULE_EXTEND_FUN_CMD, MCU_TX_VER, send_len);
        }
        break;
        
        case 0x03: {  //子命令  主动请求天气服务数据
            if(0x02 != length) {
                //数据长度错误
                return;
            }
            
            if(value[1] == 0) {
                //成功
            }else {
                //失败
            }
        }
        break;
        
        case 0x04: {  //子命令  打开模块重置状态通知
            if(0x02 != length) {
                //数据长度错误
                return;
            }
            
            if(value[1] == 0) {
                //成功
            }else {
                //失败
            }
        }
        break;
        
        case 0x05: {  //子命令  模块重置状态通知
            if(0x02 != length) {
                //数据长度错误
                return;
            }
            
            switch(value[1]) {
                case 0x00:
                    //模块本地重置
                    
                break;
                case 0x01:
                    //APP远程重置
                    
                break;
                case 0x02:
                    //APP恢复出厂重置
                    
                break;
                default:break;
            }
            
            unsigned short send_len = 0;
            send_len = set_wifi_uart_byte(send_len, sub_cmd);
            wifi_uart_write_frame(MODULE_EXTEND_FUN_CMD, MCU_TX_VER, send_len);
        }
        break;
        
        default:break;
    }
}
#endif

#ifdef BLE_RELATED_FUNCTION_ENABLE
/**
 * @brief  蓝牙功能性测试结果
 * @param[in] {value} 数据缓冲区
 * @param[in] {length} 数据长度
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void BLE_test_result(const unsigned char value[], unsigned short length)
{
    #error "请自行实现蓝牙功能性测试结果代码,完成后请删除该行"
    unsigned char sub_cmd = value[0];
    
    if(0x03 != length) {
        //数据长度错误
        return;
    }
    
    if(0x01 != sub_cmd) {
        //子命令错误
        return;
    }
    
    unsigned char result = value[1];
    unsigned char rssi = value[2];
        
    if(result == 0) {
        //测试失败
        if(rssi == 0x00) {
            //未扫描到名称为 ty_mdev蓝牙信标,请检查
        }else if(rssi == 0x01) {
            //模块未授权
        }
    }else if(result == 0x01) {
        //测试成功
        //rssi为信号强度(0-100, 0信号最差，100信号最强)
    }
}
#endif

#ifdef VOICE_MODULE_PROTOCOL_ENABLE
/**
 * @brief  获取语音状态码结果
 * @param[in] {result} 语音状态码
 * @ref       0x00: 空闲
 * @ref       0x01: mic静音状态
 * @ref       0x02: 唤醒
 * @ref       0x03: 正在录音
 * @ref       0x04: 正在识别
 * @ref       0x05: 识别成功
 * @ref       0x06: 识别失败
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void get_voice_state_result(unsigned char result)
{
    #error "请自行实现获取语音状态码结果处理代码,完成后请删除该行"
    switch(result) {
        case 0:
            //空闲
        break;
    
        case 1:
            //mic静音状态
        break;
        
        case 2:
            //唤醒
        break;
        
        case 3:
            //正在录音
        break;
        
        case 4:
            //正在识别
        break;
    
        case 5:
            //识别成功
        break;
        
        case 6:
            //识别失败
        break;
        
      default:break;
    }
}

/**
 * @brief  MIC静音设置结果
 * @param[in] {result} 语音状态码
 * @ref       0x00: mic 开启
 * @ref       0x01: mic 静音
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void set_voice_MIC_silence_result(unsigned char result)
{
    #error "请自行实现MIC静音设置处理代码,完成后请删除该行"
    if(result == 0) {
        //mic 开启
    }else {
        //mic 静音
    }
}

/**
 * @brief  speaker音量设置结果
 * @param[in] {result} 音量值
 * @ref       0~10: 音量范围
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void set_speaker_voice_result(unsigned char result)
{
    #error "请自行实现speaker音量设置结果处理代码,完成后请删除该行"
    
}

/**
 * @brief  音频产测结果
 * @param[in] {result} 音频产测状态
 * @ref       0x00: 关闭音频产测
 * @ref       0x01: mic1音频环路测试
 * @ref       0x02: mic2音频环路测试
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void voice_test_result(unsigned char result)
{
    #error "请自行实现音频产测结果处理代码,完成后请删除该行"
    if(result == 0x00) {
        //关闭音频产测
    }else if(result == 0x01) {
        //mic1音频环路测试
    }else if(result == 0x02) {
        //mic2音频环路测试
    }
}

/**
 * @brief  唤醒产测结果
 * @param[in] {result} 唤醒返回值
 * @ref       0x00: 唤醒成功
 * @ref       0x01: 唤醒失败(10s超时失败)
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void voice_awaken_test_result(unsigned char result)
{
    #error "请自行实现唤醒产测结果处理代码,完成后请删除该行"
    if(result == 0x00) {
        //唤醒成功
    }else if(result == 0x01) {
        //唤醒失败
    }
}

/**
 * @brief  语音模组扩展功能
 * @param[in] {value} 数据缓冲区
 * @param[in] {length} 数据长度
 * @return Null
 * @note   MCU需要自行实现该功能
 */
void voice_module_extend_fun(const unsigned char value[], unsigned short length)
{
    unsigned char sub_cmd = value[0];
    unsigned char play;
    unsigned char bt_play;
    unsigned short send_len = 0;
  
    switch(sub_cmd) {
        case 0x00: { //子命令  MCU功能设置
            if(0x02 != length) {
                //数据长度错误
                return;
            }
            
            if(value[1] == 0) {
                //成功
            }else {
                //失败
            }
        }
        break;
        
        case 0x01: {  //子命令  状态通知
            if(0x02 > length) {
                //数据长度错误
                return;
            }
            
            unsigned char play = 0xff;
            unsigned char bt_play = 0xff;
            
            const char *str_buff = (const char *)&value[1];
            const char *str_result = NULL;
            
            str_result = strstr(str_buff,"play") + my_strlen("play") + 2;
            if(NULL == str_result) {
                //数据错误
                goto ERR_EXTI;
            }
            
            if(0 == memcmp(str_result, "true", my_strlen("true"))) {
                play = 1;
            }else if(0 == memcmp(str_result, "false", my_strlen("false"))) {
                play = 0;
            }else {
                //数据错误
                goto ERR_EXTI;
            }
            
            str_result = strstr(str_buff,"bt_play") + my_strlen("bt_play") + 2;
            if(NULL == str_result) {
                //数据错误
                goto ERR_EXTI;
            }
            
            if(0 == memcmp(str_result, "true", my_strlen("true"))) {
                bt_play = 1;
            }else if(0 == memcmp(str_result, "false", my_strlen("false"))) {
                bt_play = 0;
            }else {
                //数据错误
                goto ERR_EXTI;
            }
            
            #error "请自行实现语音模组状态通知处理代码,完成后请删除该行"
            //MCU设置暂仅支持”播放/暂停” ”蓝牙开关”
            //play    播放/暂停功能  1(播放) / 0(暂停)
            //bt_play 蓝牙开关功能   1(开)   / 0(关)
            
            
            
            send_len = 0;
            send_len = set_wifi_uart_byte(send_len, sub_cmd);
            send_len = set_wifi_uart_byte(send_len, 0x00);
            wifi_uart_write_frame(MODULE_EXTEND_FUN_CMD, MCU_TX_VER, send_len);
        }
        break;

        default:break;
    }
    
    return;

ERR_EXTI:
    send_len = 0;
    send_len = set_wifi_uart_byte(send_len, sub_cmd);
    send_len = set_wifi_uart_byte(send_len, 0x01);
    wifi_uart_write_frame(MODULE_EXTEND_FUN_CMD, MCU_TX_VER, send_len);
    return;
}
#endif




