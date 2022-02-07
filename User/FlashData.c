
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stmflash.h"
#include "FlashData.h"
#include "DataParse.h"
#include "flash_helper.h"


//浇水模式:自动
void FlashSetModeAuto(void)
{
	u16 ReadDat = 0;
  STMFLASH_Write(FLASH_USER_WATER_MODE_ADDR,&ReadDat,1);
	printf("mode = ModeAuto\r\n");
}
//浇水模式:手动
void FlashSetModeManual(void)
{
	u16 ReadDat = 1;
  STMFLASH_Write(FLASH_USER_WATER_MODE_ADDR,&ReadDat,1);
	printf("mode = ModeManual\r\n");
}
//获取浇水模式
u16 FlashGetModeWatering(void)
{
	u16 ReadDat = 0;
  STMFLASH_Read(FLASH_USER_WATER_MODE_ADDR,&ReadDat,1);
	return ReadDat == 65535 ? 0 : ReadDat;
}

//浇水次数:设置
void FlashSetWaterCount(uint16_t data)
{
	u16 ReadDat = data;
	flash_helper_struct.FlashHelperData[1] = data;
	FlashHelperUpdate();//刷新
}
//浇水次数:获取
u16 FlashGetWaterCount(void)
{
	u16 ReadDat = 0;
	ReadDat = flash_helper_struct.FlashHelperData[1];
	return ReadDat == 65535 ? 0 : ReadDat;
}


//设置LED状态
void FlashSetLedState(uint16_t data)
{
	u16 ReadDat = data;
	flash_helper_struct.FlashHelperData[15] = data;
	FlashHelperUpdate();//刷新
}
//获取LED状态
u16 FlashGetLedState(void)
{
	u16 ReadDat = 0;
	ReadDat = flash_helper_struct.FlashHelperData[15];
	return ReadDat == 65535 ? 0 : ReadDat;
}

//设置植物类型
void FlashSetBotanyType(long long data)
{
	u16 ReadDat[4];
	Resolve_Data.lld_data = data;
	ReadDat[0] = Resolve_Data.u16_table[0];
	ReadDat[1] = Resolve_Data.u16_table[1];
	ReadDat[2] = Resolve_Data.u16_table[2];
	ReadDat[3] = Resolve_Data.u16_table[3];
	
  STMFLASH_Write(FLASH_USER_BOTANY_ADDR,&ReadDat[0],4);
}
//获取植物类型
long long FlashGetBotanyType(void)
{
	u16 ReadDat[4];
  STMFLASH_Read(FLASH_USER_BOTANY_ADDR,ReadDat,4);
	
	if(ReadDat[0] == 65535 && ReadDat[1] == 65535 && ReadDat[2] == 65535 && ReadDat[3] == 65535){
		 ReadDat[0]=0; ReadDat[1]=0; ReadDat[2]=0; ReadDat[3]=0;
		return -1;
	}
	
	Resolve_Data.u16_table[0] = ReadDat[0];
	Resolve_Data.u16_table[1] = ReadDat[1];
	Resolve_Data.u16_table[2] = ReadDat[2];
	Resolve_Data.u16_table[3] = ReadDat[3];
	
	return Resolve_Data.lld_data;
}

//爱心值:设置
void FlashSetLoveValue(uint16_t data)
{
	u16 ReadDat = data;
	flash_helper_struct.FlashHelperData[2] = data;
	FlashHelperUpdate();//刷新
}

//爱心值:获取
u16 FlashGetLoveValue(void)
{
	u16 ReadDat = 0;
	ReadDat = flash_helper_struct.FlashHelperData[2];
	return ReadDat == 65535 ? 0 : ReadDat; 
}

//夜间不报警浇水 1:是  0:否
void FlashSetReportEvent(uint16_t data)
{
	u16 ReadDat = data;
  STMFLASH_Write(FLASH_USE_FLASH_REPORTEVENT,&ReadDat,1);
}
//夜间不报警浇水 1:是  0:否
u16 FlashGetReportEvent(void)
{
	u16 ReadDat = 0;
  STMFLASH_Read(FLASH_USE_FLASH_REPORTEVENT,&ReadDat,1);
	return ReadDat == 65535 ? 1 : ReadDat;
}


//补光灯打开模式 0:自动  1:手动
void FlashSetFillLightMode(uint16_t data)
{
	u16 ReadDat = data;
  STMFLASH_Write(FLASH_USE_FLASH_FILLLIGHT,&ReadDat,1);
}
//补光灯打开模式 0:自动  1:手动
u16 FlashGetFillLightMode(void)
{
	u16 ReadDat = 0;
  STMFLASH_Read(FLASH_USE_FLASH_FILLLIGHT,&ReadDat,1);
	return ReadDat == 65535 ? 0 : ReadDat;
}


//设置阈值
void FlashSetSafe(uint16_t L,uint16_t H,safe_type_typedef safe_type)
{
	u16 ReadDat[2];
	ReadDat[0]=L;ReadDat[1]=H;
	if(safe_type == safe_type_temperature){
		STMFLASH_Write(FLASH_USER_TEMP_ADDR,&ReadDat[0],2);
	}
	else if(safe_type == safe_type_humidity){
		STMFLASH_Write(FLASH_USER_HUMI_ADDR,&ReadDat[0],2);
	}
  else if(safe_type == safe_type_illumination){
		STMFLASH_Write(FLASH_USER_ILL_ADDR,&ReadDat[0],2);
	}
	else if(safe_type == safe_type_fertility){
		STMFLASH_Write(FLASH_USER_FERT_ADDR,&ReadDat[0],2);
	}
}


//获取阈值
void FlashGetSafe(uint16_t* L,uint16_t* H,safe_type_typedef safe_type)
{
	u16 ReadDat[2];
	if(safe_type == safe_type_temperature){
		STMFLASH_Read(FLASH_USER_TEMP_ADDR,&ReadDat[0],2);
	}
	else if(safe_type == safe_type_humidity){
		STMFLASH_Read(FLASH_USER_HUMI_ADDR,&ReadDat[0],2);
	}
  else if(safe_type == safe_type_illumination){
		STMFLASH_Read(FLASH_USER_ILL_ADDR,&ReadDat[0],2);
	}
	else if(safe_type == safe_type_fertility){
		STMFLASH_Read(FLASH_USER_FERT_ADDR,&ReadDat[0],2);
	}
	
	if(ReadDat[0] == 65535) ReadDat[0]=0;
	if(ReadDat[1] == 65535) ReadDat[1]=0;
	*L = ReadDat[0];
	*H = ReadDat[1];
}

//设置光照强度报警值
void FlashSetLightAlarm(int value)
{
	u16 ReadDat[2];
	ReadDat[0] = (value>>16)&0x0000FFFF;
	ReadDat[1] = value&0x0000FFFF;
	
  STMFLASH_Write(FLASH_USE_FLASH_LIGHTALARM,&ReadDat[0],2);
}

//获取光照强度报警值
int FlashGetLightAlarm(void)
{
	u16 ReadDat[2];
	int value=0;
  STMFLASH_Read(FLASH_USE_FLASH_LIGHTALARM,ReadDat,2);
	
	value = ReadDat[0];
	value = ReadDat[0]<<16;
	value = value | ReadDat[1];
	
	if(ReadDat[0] == 0xFFFF && ReadDat[1] == 0xFFFF)
	{
		value = 60000;
	}
	return value;
}
//更改基质剩余量
void FlashSetMatrixData(long long data)
{
	u16 ReadDat[4];
	Resolve_Data.lld_data = data;
	ReadDat[0] = Resolve_Data.u16_table[0];
	ReadDat[1] = Resolve_Data.u16_table[1];
	ReadDat[2] = Resolve_Data.u16_table[2];
	ReadDat[3] = Resolve_Data.u16_table[3];
	
	flash_helper_struct.FlashHelperData[3] = ReadDat[0];
	flash_helper_struct.FlashHelperData[4] = ReadDat[1];
	flash_helper_struct.FlashHelperData[5] = ReadDat[2];
	flash_helper_struct.FlashHelperData[6] = ReadDat[3];
	FlashHelperUpdate();//刷新
}

//获取更改基质剩余量
long long FlashGetMatrixData(void)
{
	u16 ReadDat[4];
	Resolve_Data.u16_table[0] = flash_helper_struct.FlashHelperData[3];
	Resolve_Data.u16_table[1] = flash_helper_struct.FlashHelperData[4];
	Resolve_Data.u16_table[2] = flash_helper_struct.FlashHelperData[5];
	Resolve_Data.u16_table[3] = flash_helper_struct.FlashHelperData[6];
	return Resolve_Data.lld_data;
}
//设置种植时间
void FlashSetDataPlant(long long data)
{
	u16 ReadDat[4];
	Resolve_Data.lld_data = data;
	ReadDat[0] = Resolve_Data.u16_table[0];
	ReadDat[1] = Resolve_Data.u16_table[1];
	ReadDat[2] = Resolve_Data.u16_table[2];
	ReadDat[3] = Resolve_Data.u16_table[3];
	
  STMFLASH_Write(FLASH_USER_DATA_ADDR,&ReadDat[0],4);
}

//获取种植时间
long long FlashGetDataPlant(void)
{
	u16 ReadDat[4];
  STMFLASH_Read(FLASH_USER_DATA_ADDR,ReadDat,4);
	
	if(ReadDat[0] == 65535 && ReadDat[1] == 65535 && ReadDat[2] == 65535 && ReadDat[3] == 65535){
		 ReadDat[0]=0; ReadDat[1]=0; ReadDat[2]=0; ReadDat[3]=0;
	}
	
	Resolve_Data.u16_table[0] = ReadDat[0];
	Resolve_Data.u16_table[1] = ReadDat[1];
	Resolve_Data.u16_table[2] = ReadDat[2];
	Resolve_Data.u16_table[3] = ReadDat[3];
	
	return Resolve_Data.lld_data;
}

//累计收获:设置
void FlashSetHarvestCount(uint16_t data)
{
	u16 ReadDat = data;
  STMFLASH_Write(FLASH_USER_HARVEST_ADDR,&ReadDat,1);
}

//累计收获:获取
u16 FlashGetHarvestCount(void)
{
	u16 ReadDat = 0;
  STMFLASH_Read(FLASH_USER_HARVEST_ADDR,&ReadDat,1);
	return ReadDat == 65535 ? 0 : ReadDat;
}






////基质:设置
//void FlashSetMatrix(uint16_t data)
//{
//	u16 ReadDat = data;
//  STMFLASH_Write(FLASH_USER_MATRIX_ADDR,&ReadDat,1);
//}

////基质:获取
//u16 FlashGetMatrix(void)
//{
//	u16 ReadDat = 0;
//  STMFLASH_Read(FLASH_USER_MATRIX_ADDR,&ReadDat,1);
//	return ReadDat == 65535 ? 0 : ReadDat;
//}


//培育天数
void FlashSetNurtureDay(uint16_t data)
{
	u16 ReadDat = data;
	flash_helper_struct.FlashHelperData[7] = data;
	FlashHelperUpdate();//刷新
}

//培育天数
u16 FlashGetNurtureDay(void)
{
	u16 ReadDat = 0;
	ReadDat = flash_helper_struct.FlashHelperData[7];
	return ReadDat == 65535 ? 0 : ReadDat; 
}




//设置种植时间(首次)
void FlashSetOneDataPlant(long long data)
{
	u16 ReadDat[4];
	Resolve_Data.lld_data = data;
	ReadDat[0] = Resolve_Data.u16_table[0];
	ReadDat[1] = Resolve_Data.u16_table[1];
	ReadDat[2] = Resolve_Data.u16_table[2];
	ReadDat[3] = Resolve_Data.u16_table[3];
	
  STMFLASH_Write(FLASH_USER_ONEDATA_ADDR,&ReadDat[0],4);
}


//获取种植时间(首次)
long long FlashGetOneDataPlant(void)
{
	u16 ReadDat[4];
  STMFLASH_Read(FLASH_USER_ONEDATA_ADDR,ReadDat,4);
	
	if(ReadDat[0] == 65535 && ReadDat[1] == 65535 && ReadDat[2] == 65535 && ReadDat[3] == 65535){
		 ReadDat[0]=0; ReadDat[1]=0; ReadDat[2]=0; ReadDat[3]=0;
	}
	
//	if(ReadDat[0] == 65535) ReadDat[0]=0;
//	if(ReadDat[1] == 65535) ReadDat[1]=0;
//	if(ReadDat[2] == 65535) ReadDat[2]=0;
//	if(ReadDat[3] == 65535) ReadDat[3]=0;
	
	Resolve_Data.u16_table[0] = ReadDat[0];
	Resolve_Data.u16_table[1] = ReadDat[1];
	Resolve_Data.u16_table[2] = ReadDat[2];
	Resolve_Data.u16_table[3] = ReadDat[3];
	
	return Resolve_Data.lld_data;
}


//设置光照强度统计上报状态(清除)
void FlashClearIlluminationState(void)
{
	u16 ReadDat = 0;
  STMFLASH_Write(FLASH_USE_FLASH_ILLEVENT,&ReadDat,1);
}
//设置光照强度统计上报状态(设置)
void FlashSetIlluminationState(void)
{
	u16 ReadDat = 1;
  STMFLASH_Write(FLASH_USE_FLASH_ILLEVENT,&ReadDat,1);
}

//获取光照强度统计上报状态
u16 FlashGetIlluminationState(void)
{
	u16 ReadDat = 0;
  STMFLASH_Read(FLASH_USE_FLASH_ILLEVENT,&ReadDat,1);
	return ReadDat == 65535 ? 0 : ReadDat;
}



//更改光照统计
//data 统计的光照强度
//count 统计的次数
//time 时间戳
void FlashSetIlluminationStatistics(u32 data,u16 count,long long time)
{
	u16 ReadDat[2];//光照强度
	ReadDat[0] = (data>>16)&0xffff;
	ReadDat[1] = data&0xffff;
	flash_helper_struct.FlashHelperData[8] = ReadDat[0];
	flash_helper_struct.FlashHelperData[9] = ReadDat[1];
	//统计次数
	flash_helper_struct.FlashHelperData[10] = count;
	//时间戳
	Resolve_Data.lld_data = time;
	flash_helper_struct.FlashHelperData[11] = Resolve_Data.u16_table[0];
	flash_helper_struct.FlashHelperData[12] = Resolve_Data.u16_table[1];
	flash_helper_struct.FlashHelperData[13] = Resolve_Data.u16_table[2];
	flash_helper_struct.FlashHelperData[14] = Resolve_Data.u16_table[3];
	
	FlashHelperUpdate();//刷新
}
//获取光照统计
//count 返回的统计次数
//time  返回的统计时的时间戳
//return 统计的光照强度
u32 FlashGetIlluminationStatistics(u16* count,long long *time)
{
	u32 ReadDat;//光照强度
	ReadDat = flash_helper_struct.FlashHelperData[8];
	ReadDat = ReadDat<<16;
	ReadDat = ReadDat | flash_helper_struct.FlashHelperData[9];
	//统计次数
	*count = flash_helper_struct.FlashHelperData[10];
	//时间戳
	Resolve_Data.u16_table[0] = flash_helper_struct.FlashHelperData[11];
	Resolve_Data.u16_table[1] = flash_helper_struct.FlashHelperData[12];
	Resolve_Data.u16_table[2] = flash_helper_struct.FlashHelperData[13];
	Resolve_Data.u16_table[3] = flash_helper_struct.FlashHelperData[14];
	*time =  Resolve_Data.lld_data;
	return ReadDat;
}



//获取设备的版本
u16 IAPGetHardwareVersion(void)
{
	u16 ReadDat = 0;
  STMFLASH_Read(FLASH_UPDATE_VERSION2_ADDR,&ReadDat,1);
	return ReadDat == 65535 ? 0 : ReadDat;
}
