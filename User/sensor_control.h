#ifndef SENSOR_CONTROL_Cc_
#define SENSOR_CONTROL_Cc_

#ifndef SENSOR_CONTROL_Cx_//如果没有定义
#define SENSOR_CONTROL_Cx_ extern
#else
#define SENSOR_CONTROL_Cx_
#endif

#include "stm32f10x.h"
#include <time.h>

//故障点上报
#define fault_none 0      //无故障
#define fault_hardware 1  //硬件故障
#define FAULT_FERTILITY_HIGH 2 //肥力高
#define FAULT_FERTILITY_LOW 4 //肥力低
#define FAULT_HUMIDITY_HIGH 8  //湿度高
#define FAULT_HUMIDITY_LOW 16  //湿度低
#define FAULT_TEMPERATURE_HIGH 32  //温度高
#define FAULT_TEMPERATURE_LOW 64  //温度低
#define FAULT_ILLUMINATION_HIGH 128  //光照高
#define FAULT_ILLUMINATION_LOW 256  //光照低
#define FAULT_WATER_HIGH 512  //水位高
#define FAULT_WATER_HYDROPENIA 2048  //缺水
#define FAULT_TOTAL_LIGHT 1024 //光照总量不足


//浇水/报警时间(默认)
#define  SAFE_LIMIT_TIME_START 8
#define  SAFE_LIMIT_TIME_STOP  20
//光照总量统计时间段(默认)
#define ILLUMINATION_STATISTICS_TIMER_START  8 //开始时间
#define ILLUMINATION_STATISTICS_TIMER_STOP 20 //结束时间

//安全值-水位
#define SAFE_WATER_HIGH 100 //水满
#define SAFE_WATER_LOW 10 //水位低
#define SAFE_HYDROPENIA 0 //缺水


SENSOR_CONTROL_Cx_ int wifi_state;//模组状态
SENSOR_CONTROL_Cx_ int wifi_state_copy;//模组状态
SENSOR_CONTROL_Cx_ int mcu_updata_flag;//0-正常运行; 1-更新中
SENSOR_CONTROL_Cx_ int mcu_updata_flag_clear_cnt;//清除更新状态标记

SENSOR_CONTROL_Cx_ struct tm *lcTime;//时间戳转时间
SENSOR_CONTROL_Cx_ time_t timestamp; //时间戳
SENSOR_CONTROL_Cx_ time_t timestamp_copy; //时间戳
SENSOR_CONTROL_Cx_ char timezone; //时区偏差(默认8,北京时间)
//用户按下按钮启动SmartConfig配网绑定
SENSOR_CONTROL_Cx_ char SmartConfigStartFlag;

//配网设置标志,连接上路由器以后清除此标志
//并恢复出厂设置
SENSOR_CONTROL_Cx_ char SmartConfigFlag;

//事件
typedef struct event_struct
{
	/*温度*/
	u32 low_temperature_timer_cnt;
	u32 high_temperature_timer_cnt;
	/*湿度*/
	u32 low_humidity_timer_cnt;
	u32 high_humidity_timer_cnt;
	/*光照*/
	u32 low_illumination_timer_cnt;
	u32 high_illumination_timer_cnt;
	/*肥力*/
	u32 low_fertility_timer_cnt;
	u32 high_fertility_timer_cnt;
	/*缺水*/
	u32 hydropenia_timer_cnt;
	/*水位低*/
	u32 low_water_timer_cnt;
	/*水位高*/
	u32 high_water_timer_cnt;
}event_struct;

SENSOR_CONTROL_Cx_ event_struct event_struct_value;

#define public_temp_char_len 100//通用变量长度
//传感器数据
typedef struct sensor_control_struct
{
	int  public_int_value;//通用变量
	char public_temp_char[public_temp_char_len];//通用变量
	char pump_value;//水泵
	char pump_value_copy;//水泵
	char pump_err_flag;//水泵传感器异常
	
	char light_value;//LED
	char light_value_copy;//LED
	char light_err_flag;//LED异常
	//补光灯
	char fill_light_value;//LED
	char fill_light_value_copy;//LED
	char fill_light_err_flag;//LED异常
		//补光灯模式
	char fill_light_mode_value;
	char fill_light_mode_value_copy;
	
	//湿度
	float humidity_v;//湿度传感器电压
	int humidity_value;//湿度传感器值
	int humidity_value_copy;//湿度传感器值
	char humidity_err_count;//传感器错误采集计数
	char humidity_err_flag;//湿度传感器异常
	//温度
	int temperature_value;//温度传感器值
	int temperature_value_copy;//温度传感器值
	char temperature_err_count;//传感器错误采集计数
	char temperature_err_flag;//温度传感器异常
	//空气湿度
	int humidity1_value;//湿度传感器值
	int humidity1_value_copy;//湿度传感器值
	char humidity1_err_count;//传感器错误采集计数
	char humidity1_err_flag;//湿度传感器异常
	
	//光照
	int illumination_value;//光照传感器值
	int illumination_value_copy;//光照传感器值
	char illumination_err_count;//传感器错误采集计数
	char illumination_err_flag;//温度传感器异常
	//水位
	int water_stage_value;//传感器值
	int water_stage_value_copy;//传感器值
	char water_stage_err_count;//传感器错误采集计数
	char water_stage_err_flag;//传感器异常
	//肥力
	int fertility_value;//传感器值
	int fertility_value_copy;//传感器值
	char fertility_err_count;//传感器错误采集计数
	char fertility_err_flag;//传感器异常
	
	//模式
	int mode_value;//浇水模式
	int mode_value_copy;//浇水模式
	
	//浇水次数
	int watering_count;
	int watering_count_copy;
	
	//植物类型
	long botany_type;
	long botany_type_copy;
	
	//爱心值
	int love_value;
	int love_value_copy;
	
	//夜间不报警浇水
	int report_night_value;
	int report_night_value_copy;
	
	//安全值-温度
	unsigned short int safe_temperature[2];
	
	//安全值-湿度
	unsigned short int safe_humidity[2];
	
	//安全值-光照
	unsigned short int safe_illumination[2];
	
	//安全值-肥力
	unsigned short int safe_fertility[2];
	
	//安全值-浇水报警时间
	unsigned short int safe_limit_time[2];
	
	//光照总量不足比较值
	int light_low_limit_value;
	//光照总量统计时间段
	char illumination_statistics_timer_start;//开始时间
	char illumination_statistics_timer_stop;//结束时间
	//光照统计相关
	int illumination_statistics_timer_delay;//在定时器里面累加,设置统计时间间隔
	u16 illumination_statistics_temp[10];//存储一个时间段采集的光照强度
	int illumination_statistics_cnt;
	u32 illumination_statistics_sum;//光照统计总量
	u16 illumination_statistics_count;//光照统计次数
	
	
	//光照统计相关
	int illumination_statistics_timer_delay1;//在定时器里面累加,设置统计时间间隔
	u16 illumination_statistics_temp1[10];//存储一个时间段采集的光照强度
	int illumination_statistics_cnt1;
	
	//基质剩余量
	char matrix_value;
	char matrix_value_copy;
	
	//更换植物种植的时间
	time_t plant_value;
	
	//累计收获
	long cumulative_harvest_value;
	long cumulative_harvest_value_copy;
	
	//开始种植时下发的时间
	time_t begin_date_value;
	
	//培育天数
	unsigned int nurture_day;
	unsigned int nurture_day_copy;
	
	//水满/缺水,控制蜂鸣器滴滴
	char water_full_flag;//水满标志
	char water_lack_flag;//缺水标志
	
	//电池电量
	int soc_value;
	int soc_value_copy;
	
} sensor_control_struct;


SENSOR_CONTROL_Cx_ sensor_control_struct sensor_control_struct_value;


//统计光照强度
void illumination_intensity_statistics(void);
//事件上报函数
void event_report_fun(void);
//事件采集函数(放在1ms定时器)
void event_loop_fun(void);
//每培育1天,基质减一 , 爱心值累加10
void NurtureDayFunction(void);
//循环浇水
void WateringCirculationFunction(void);
//控制浇水
void sensor_watering(void);
//上报传感器数据
void sensor_data_appear(void);
//初始化传感器数据
void sensor_data_value_init(void);
//电压和湿度的换算
int humidity_conversion(float Value);
//轮训采集传感器数据
void sensor_data_collection(void);
//控制水泵
void sensor_pump_set(char value);
//控制指示灯
void sensor_light_set(char value);
//控制补光灯
void sensor_fill_light_set(char value);
//控制蜂鸣器
void sensor_buzzer_set(char value);
//恢复出厂设置
void factory_reset(void);
#endif
