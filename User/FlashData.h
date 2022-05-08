#ifndef __FLASHDATA_H__
#define __FLASHDATA_H__
#include <stm32f10x.h>

typedef enum {/*设置/获取的阈值类型*/
	safe_type_temperature = 0,
  safe_type_humidity = 1,
  safe_type_illumination = 2,
	safe_type_fertility = 3,
}safe_type_typedef;

//浇水模式
void FlashSetModeAuto(void);
void FlashSetModeManual(void);
u16 FlashGetModeWatering(void);

//浇水次数
//浇水次数:设置
void FlashSetWaterCount(uint16_t data);
//浇水次数:获取
u16 FlashGetWaterCount(void);

//设置植物类型
void FlashSetBotanyType(long long data);
//获取植物类型
long long FlashGetBotanyType(void);

//爱心值:设置
void FlashSetLoveValue(uint16_t data);
//爱心值:获取
u16 FlashGetLoveValue(void);

//夜间不报警浇水 0:是  1:否
void FlashSetReportEvent(uint16_t data);
//夜间不报警浇水 0:是  1:否
u16 FlashGetReportEvent(void);
//补光灯打开模式 0:自动  1:手动
void FlashSetFillLightMode(uint16_t data);
//补光灯打开模式 0:自动  1:手动
u16 FlashGetFillLightMode(void);


void FlashSetSafe(uint16_t L,uint16_t H,safe_type_typedef safe_type);
void FlashGetSafe(uint16_t* L,uint16_t* H,safe_type_typedef safe_type);

void FlashSetSwitch(uint16_t data);////////////// 
//??????g? 0:???  1:??? 
u16 FlashGetSwitch(void); 
 
//设置光照强度报警值
void FlashSetLightAlarm(int value);
//获取光照强度报警值
int FlashGetLightAlarm(void);

//更改基质剩余量
void FlashSetMatrixData(long long data);
//获取更改基质剩余量
long long FlashGetMatrixData(void);

//设置种植时间
void FlashSetDataPlant(long long data);
//获取种植时间
long long FlashGetDataPlant(void);
//培育天数
void FlashSetNurtureDay(uint16_t data);
//培育天数
u16 FlashGetNurtureDay(void);

//累计收获:设置
void FlashSetHarvestCount(uint16_t data);
//累计收获:获取
u16 FlashGetHarvestCount(void);
//设置种植时间(首次)
void FlashSetOneDataPlant(long long data);
//获取种植时间(首次)
long long FlashGetOneDataPlant(void);


//设置LED状态
void FlashSetLedState(uint16_t data);
//获取LED状态
u16 FlashGetLedState(void);



//更改光照统计
//data 统计的光照强度
//统计的次数
void FlashSetIlluminationStatistics(u32 data,u16 count,long long time);

//获取光照统计
//count 返回的统计次数
//return 统计的光照强度
u32 FlashGetIlluminationStatistics(u16* count,long long *time);



//设置光照强度统计上报状态(清除)
void FlashClearIlluminationState(void);

//设置光照强度统计上报状态(设置)
void FlashSetIlluminationState(void);

//获取光照强度统计上报状态
u16 FlashGetIlluminationState(void);

//获取设备的版本
u16 IAPGetHardwareVersion(void);
#endif

