#include "stmflash.h"

FLASH_Status FlashStatus;


//读取指定地址的半字(16位数据)
//faddr:读地址(此地址必须为2的倍数!!)
//返回值:对应数据.
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
	return *(vu16*)faddr; 
}

#if STM32_FLASH_WREN	//如果使能了写   
//不检查的写入
//WriteAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数   
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)   
{ 			 		 
	u16 i;
	for(i=0;i<NumToWrite;i++)
	{
		FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
		WriteAddr+=2;//地址增加2.
	}  
} 
//从指定地址开始写入指定长度的数据
//WriteAddr:起始地址(此地址必须为2的倍数!!)
//pBuffer:数据指针
//NumToWrite:半字(16位)数(就是要写入的16位数据的个数.)
#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024 //字节
#else 
#define STM_SECTOR_SIZE	2048
#endif		 
u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//最多是2K字节
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)	
{
	u32 secpos;	   //扇区地址
	u16 secoff;	   //扇区内偏移地址(16位字计算)
	u16 secremain; //扇区内剩余地址(16位字计算)	   
 	u16 i;    
	u32 offaddr;   //去掉0X08000000后的地址
	if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//非法地址
	FLASH_Unlock();						//解锁
	offaddr=WriteAddr-STM32_FLASH_BASE;		//实际偏移地址.
	secpos=offaddr/STM_SECTOR_SIZE;			//扇区地址  0~127 for STM32F103RBT6
	secoff=(offaddr%STM_SECTOR_SIZE)/2;		//在扇区内的偏移(2个字节为基本单位.)
	secremain=STM_SECTOR_SIZE/2-secoff;		//扇区剩余空间大小   
	if(NumToWrite<=secremain)secremain=NumToWrite;//不大于该扇区范围
	while(1) 
	{	
		STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//读出整个扇区的内容
		for(i=0;i<secremain;i++)//校验数据
		{
			if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//需要擦除  	  
		}
		if(i<secremain)//需要擦除
		{
			FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//擦除这个扇区
			
			for(i=0;i<STM_SECTOR_SIZE;i+=2)//再次判断是否有擦除失败
			{
			  if(STMFLASH_ReadHalfWord(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE+i)!=0XFFFF)
				{
				  FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//擦除这个扇区
					break;
				}
			}
			
			for(i=0;i<secremain;i++)//复制
			{
				STMFLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//写入整个扇区  
		}
		else 
		{
			 STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//写已经擦除了的,直接写入扇区剩余区间. 
		}
				
		if(NumToWrite==secremain)break;//写入结束了
		else//写入未结束
		{
			secpos++;				//扇区地址增1
			secoff=0;				//偏移位置为0 	 
			pBuffer+=secremain;  	//指针偏移
			WriteAddr+=secremain;	//写地址偏移	   
			NumToWrite-=secremain;	//字节(16位)数递减
			if(NumToWrite>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2;//下一个扇区还是写不完
			else secremain=NumToWrite;//下一个扇区可以写完了
		}	 
	};	
	FLASH_Lock();//上锁
}
#endif




void WriteFlashHalfWords(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)   
{ 			 		 
	u16 i;
	
	FlashStatus = FLASH_BUSY;//设置为忙
	if(FLASH_GetStatus() == FLASH_COMPLETE)//可以操作Flash
	{
		FLASH_Unlock();		
		for(i=0;i<NumToWrite;i++)
		{
			FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
			WriteAddr+=2;//地址增加2.
		}  
		FLASH_Lock();//上锁
	}
} 


/**
* @brief  指定地址写入一个16位数据
* @waing  写入的地址请先擦除
* @param  WriteAddress  写入的地址
* @param  data          写入的数据
* @param  None
* @retval 0:成功
* @example 
**/
char WriteFlashHalfWord(uint32_t WriteAddress,u16 data)
{
	FlashStatus = FLASH_BUSY;//设置为忙
	if(FLASH_GetStatus() == FLASH_COMPLETE)//可以操作Flash
	{
		FLASH_Unlock();		
		FlashStatus = FLASH_ProgramHalfWord(WriteAddress,data);//写入数据
		
		if(FlashStatus == FLASH_COMPLETE)//操作完成
		{
			if(STMFLASH_ReadHalfWord(WriteAddress) == data)//读出的和写入的一致
				FlashStatus = 0;//读出和写入的一致
			else
				FlashStatus = 5;
		}
		FLASH_Lock();//上锁
	}
	return FlashStatus;
} 


//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)   	
{
	u16 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//读取2个字节.
		ReadAddr+=2;//偏移2个字节.	
	}
}


/**
* @brief  擦除Flash  
* @param  EraseAddress  擦除的地址
* @param  PageCnt       擦除连续的几页(一页按照1024计算)
* @param  None
* @retval 4:成功
* @example 
* @waing  如果芯片的Flash是按照2048分页,擦除1页第二个参数应该填写2
**/
char FlashErasePage(uint32_t EraseAddress,u16 PageCnt)
{
	u32 i=0;
	u32 secpos;	   //扇区地址
	if(EraseAddress<STM32_FLASH_BASE||(EraseAddress>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return 0;//非法地址
	secpos = EraseAddress-STM32_FLASH_BASE;//实际地址
	secpos = secpos/STM_SECTOR_SIZE;//扇区地址 
	if(STM_SECTOR_SIZE==2048){PageCnt=PageCnt/2;}
	if(FLASH_GetStatus() == FLASH_COMPLETE)//可以操作Flash
	{
		FLASH_Unlock();
		for(i=0;i<PageCnt;i++)
		{
			FlashStatus = FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//擦除这个扇区
			if(FlashStatus != FLASH_COMPLETE) break;
			secpos++;
		}
		FLASH_Lock();//上锁
	}	
	return FlashStatus;
} 



/**
* @brief  擦除Flash  
* @param  EraseAddress  擦除的地址
* @param  PageCnt       擦除连续的几页
* @param  None
* @retval 4:成功
* @example 
* @waing  
**/
char FlashErasePages(uint32_t EraseAddress,u16 PageCnt)
{
	u32 i=0;
	u32 secpos;	   //扇区地址
	if(EraseAddress<STM32_FLASH_BASE||(EraseAddress>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return 0;//非法地址
	secpos = EraseAddress-STM32_FLASH_BASE;//实际地址
	secpos = secpos/STM_SECTOR_SIZE;//扇区地址 
//	if(STM_SECTOR_SIZE==2048){PageCnt=PageCnt/2;}
	if(FLASH_GetStatus() == FLASH_COMPLETE)//可以操作Flash
	{
		FLASH_Unlock();
		for(i=0;i<PageCnt;i++)
		{
			FlashStatus = FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//擦除这个扇区
			if(FlashStatus != FLASH_COMPLETE) break;
			secpos++;
		}
		FLASH_Lock();//上锁
	}	
	return FlashStatus;
}



/*****************************************************************************
函数名称 : FlashProgramU8
功能描述 : 对Flash进行字节编程
输入参数 : pulData:写入的源数据
           ulAddress:FLASH的目标起始地址
           ulCount:写入的字节数
返回参数: SUCCESS:写入成功
          ERROR:写入失败
*****************************************************************************/
static uint8_t FlashProgramU32(uint8_t *pulData,uint32_t ulAddress,uint32_t ulCount)
{
  uint32_t Wrdata;
  
  if(ulCount % 4 != 0)
    ulCount += (4 - ulCount % 4);                                      //调整字节个数为4的倍数   
  
  while(ulCount) 
  {
    Wrdata  = *pulData ++;                                             
    Wrdata |= (*pulData ++) << 8;
    Wrdata |= (*pulData ++) << 16;
    Wrdata |= (*pulData ++) << 24;                                 //将4个字节组合成一个字数据    
    
    FLASH_ProgramWord(ulAddress,Wrdata);
    
    ulAddress += 4;                                                 
    ulCount -= 4;
  }
  
  return(SUCCESS);
}
/*****************************************************************************
函数名称 : VerifyData
功能描述 : 校验写入FLASH的数据
输入参数 : pulData       源数据
           ulAddress     FLASH的起始地址
           ulCount       校验的字节数
返回参数 : true          校验成功
           false         校验失败
*****************************************************************************/
static uint8_t VerifyData(uint8_t *pulData,uint32_t ulAddress,uint32_t ulCount)
{
  uint32_t ReData;
  if(ulCount % 4 != 0)
    ulCount += (4 - ulCount % 4);                                         /* 调整校验的字节个数为4的倍数  */

  while(ulCount) 
  {
    ReData = (*(__IO uint32_t*)ulAddress);
    
    if(*pulData != (ReData &0xff)) 
    {                         /* 校验字节数据                 */
      return ERROR;
    }
    if(*(++pulData) != ((ReData >> 8) &0xff)) 
    {                         /* 校验字节数据                 */
      return ERROR;
    }
    if(*(++pulData) != ((ReData >> 16) &0xff)) 
    {                        /* 校验字节数据                 */
      return ERROR;
    }
    if(*(++pulData) != ((ReData >> 24) &0xff)) 
    {                        /* 校验字节数据                 */
      return ERROR;
    }
    
    ulAddress += 4;                                                     /* 调整指定的Flash地址          */
    ulCount -= 4;                                                       /* 调整待校验的字节个数         */
    pulData++;                                                          /* 调整源数据指针               */
  }
  return SUCCESS;
}

/*****************************************************************************
函数名称 : Write_Flash
功能描述 : 对Flash进行字节编程
输入参数 : Addr_Sour:写入的源数据
           Addr_Obj:FLASH的目标起始地址
           Length:写入的字节数
返回参数 : SUCCESS:写入成功
            ERROR:写入失败
使用说明 : 无
*****************************************************************************/
uint8_t Write_Flash(uint32_t Addr_Obj,uint8_t *Addr_Sour,uint16_t Length)
{
  FLASH_Unlock();
  
  if(FlashProgramU32(Addr_Sour,Addr_Obj,Length) == ERROR) 
  {
    FLASH_Lock();
    return ERROR;
  } 
  
  if(VerifyData(Addr_Sour,Addr_Obj,Length) == ERROR) 
  {
    FLASH_Lock();
    return ERROR;
  } 
  FLASH_Lock();
  
  return SUCCESS;
}
/*****************************************************************************
函数名称 : Earse_Flash
功能描述 : 对Flash擦除
输入参数 : Addr_Obj:FLASH的目标起始地址
返回参数 : SUCCESS:擦除成功
            ERROR:擦除失败
使用说明 : 无
*****************************************************************************/
uint8_t Earse_Flash(uint32_t Addr_Obj)
{
  FLASH_Unlock();
  
  if(FLASH_ErasePage(Addr_Obj) != FLASH_COMPLETE)
  {
    FLASH_Lock();
    return ERROR;
  }
  
  FLASH_Lock();
  
  return SUCCESS;
}

/*****************************************************************************
函数名称 : Read_Flash
功能描述 : 读取FLASH的数据
输入参数 : pulData:源数据
           ulAddress:FLASH的起始地址
           Length:读取的字节数
返回参数 : true:读取成功
           false:读取失败
*****************************************************************************/
uint8_t Read_Flash(uint8_t *pulData,uint32_t ulAddress,uint32_t Length)
{
  uint32_t ReData;
  uint32_t cnt = 0;
  
  while(cnt < Length)
  {
    ReData = (*(__IO uint32_t*)ulAddress);
    
    *pulData ++ = (ReData & 0xff); 
    cnt ++;
    if(cnt >= Length)
      break;
    
    *pulData ++ = ((ReData >> 8) & 0xff);
    cnt ++;
    if(cnt >= Length)
      break;

    *pulData ++ = (ReData >> 16) & 0xff;
    cnt ++;
    if(cnt >= Length)
      break;

    *pulData ++ = (ReData >> 24) & 0xff;
    cnt ++;
   
    ulAddress += 4;
  }
  return SUCCESS;
}
