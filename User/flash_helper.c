/**
  ******************************************************************************
  * @author  feng wu yang
  * @version V1.0.0  
	* @version V1.0.1 修改  FlashHelperUpdate 中获取尾部数据bug
	* @version V1.1.0 修改  FlashHelperUpdate 中获取尾部数据bug
  * @date    2020/09/02
  * @brief   flash 分区轮训存储(用于提高flash使用寿命)
	* @document https://www.cnblogs.com/yangfengwu/p/13616968.html
  ******************************************************************************
	1.数组首位FlashHelperData[第一位] 和 FlashHelperData[最后一位] 作为数据标识,用户不可使用!
  ******************************************************************************
  */


#define FLASH_HELPER_C_
#include <stdio.h>
#include "flash_helper.h"
#include "stmflash.h"

#ifdef FlashHelperDebug
#include "usart.h" //使用printf打印数据
#endif

flash_helper_typedef flash_helper_struct;

//读
void FlashHelperReadDataEx(u32 addr,u16 *data, u16 datalen)
{
	STMFLASH_Read(addr,data,datalen);
}
//写
void FlashHelperWriteDataEx(u32 addr,u16 *data, u16 datalen)
{
	 WriteFlashHalfWords(addr,data,datalen);
}
//擦除
char FlashHelperErasePageEx(u32 EraseAddress,u16 PageCnt)
{
	FlashErasePages(EraseAddress,PageCnt);
}

/**
* @brief   初始化
* @param   ADDR 缓存区
* @param   None
* @param   None
* @retval  None
* @warning None
* @example 
**/
void FlashHelperInit(u32 ADDR1,u32 ADDR2,u32 ADDR3,u32 ADDR4)
{
	u16 i,j;
	u16 data[FlashHelperDataLen];
	u16 a1,a2,a3,a4;
	u16 b1,b2,b3,b4;
	
	
	flash_helper_struct.FlashHelperPageAddr[0] = ADDR1 + FlashHelperAddressOffset;
	flash_helper_struct.FlashHelperPageAddr[1] = ADDR2 + FlashHelperAddressOffset;
	flash_helper_struct.FlashHelperPageAddr[2] = ADDR3 + FlashHelperAddressOffset;
	flash_helper_struct.FlashHelperPageAddr[3] = ADDR4 + FlashHelperAddressOffset;
	
	#ifdef FlashHelperDebug
	printf("\r\n page0: 0x%x",flash_helper_struct.FlashHelperPageAddr[0]);
	printf("\r\n page1: 0x%x",flash_helper_struct.FlashHelperPageAddr[1]);
	printf("\r\n page2: 0x%x",flash_helper_struct.FlashHelperPageAddr[2]);
	printf("\r\n page3: 0x%x",flash_helper_struct.FlashHelperPageAddr[3]);
	#endif
	
	
	flash_helper_struct.FlashHelperData[0] = 0xAA;
	flash_helper_struct.FlashHelperData[FlashHelperDataLen-1] = 0x55;
	
	flash_helper_struct.FlashHelperTail = FlashHelperMaxLen/FlashHelperDataLen;
	flash_helper_struct.FlashHelperTail = flash_helper_struct.FlashHelperTail*FlashHelperDataLen;
	
	/*第0页*/
	FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[0],&a1,1);//首地址数据
	FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[0]+(flash_helper_struct.FlashHelperTail<<1)-2,&b1,1);//尾地址数据
	/*第1页*/
	FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[1],&a2,1);//首地址数据
	FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[1]+(flash_helper_struct.FlashHelperTail<<1)-2,&b2,1);//尾地址数据	
	/*第2页*/
	FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[2],&a3,1);//首地址数据
	FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[2]+(flash_helper_struct.FlashHelperTail<<1)-2,&b3,1);//尾地址数据
	/*第3页*/
	FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[3],&a4,1);//首地址数据
	FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[3]+(flash_helper_struct.FlashHelperTail<<1)-2,&b4,1);//尾地址数据
	
	
	if(a1==0xFFFF && b1==0xFFFF && a2==0xFFFF && b2==0xFFFF && a3==0xFFFF && b3==0xFFFF && a4==0xFFFF && b4==0xFFFF)
	{
		flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperPageAddr[1];
		#ifdef FlashHelperDebug
		printf("\r\n no save data");
		#endif
	}
	else if(a1==0xAA && b1 == 0xFFFF)//data save page0
	{
		#ifdef FlashHelperDebug
		printf("\r\n data save page0\r\n");
		#endif
		for(i=0;i<FlashHelperMaxLen/FlashHelperDataLen;i++)
		{
			flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperPageAddr[0]+i*FlashHelperDataLen*2;
			
			FlashHelperReadDataEx(flash_helper_struct.FlashHelperFlashAddr,data,FlashHelperDataLen);
			
			if(data[0]==0xFFFF && data[FlashHelperDataLen-1]==0xFFFF)
			{
				#ifdef FlashHelperDebug
				for(j=0;j<FlashHelperDataLen;j++){printf("%x ",flash_helper_struct.FlashHelperData[j]);}		
				#endif
				
				break;
			}
			for(j=0;j<FlashHelperDataLen;j++){flash_helper_struct.FlashHelperData[j] = data[j];}				
		}
	}
	else if(a2==0xAA && b2 == 0xFFFF)//data save page1
	{
		#ifdef FlashHelperDebug
		printf("\r\n data save page1\r\n");
		#endif
		for(i=0;i<FlashHelperMaxLen/FlashHelperDataLen;i++)
		{
			flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperPageAddr[1]+i*FlashHelperDataLen*2;
			
			FlashHelperReadDataEx(flash_helper_struct.FlashHelperFlashAddr,data,FlashHelperDataLen);
			
			if(data[0]==0xFFFF && data[FlashHelperDataLen-1]==0xFFFF)
			{
				#ifdef FlashHelperDebug
				for(j=0;j<FlashHelperDataLen;j++){printf("%x ",flash_helper_struct.FlashHelperData[j]);}		
				#endif
				break;
			}
			for(j=0;j<FlashHelperDataLen;j++){flash_helper_struct.FlashHelperData[j] = data[j];}	
		}
	}
	else if(a3==0xAA && b3 == 0xFFFF)//data save page2
	{
		#ifdef FlashHelperDebug
		printf("\r\n data save page2\r\n");
		#endif
		for(i=0;i<FlashHelperMaxLen/FlashHelperDataLen;i++)
		{
			flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperPageAddr[2]+i*FlashHelperDataLen*2;
			
			FlashHelperReadDataEx(flash_helper_struct.FlashHelperFlashAddr,data,FlashHelperDataLen);
			
			if(data[0]==0xFFFF && data[FlashHelperDataLen-1]==0xFFFF)
			{
				#ifdef FlashHelperDebug
				for(j=0;j<FlashHelperDataLen;j++){printf("%x ",flash_helper_struct.FlashHelperData[j]);}		
				#endif
				break;
			}
			for(j=0;j<FlashHelperDataLen;j++){flash_helper_struct.FlashHelperData[j] = data[j];}	
		}
	}
	else if(a4==0xAA && b4 == 0xFFFF)//data save page3
	{
		#ifdef FlashHelperDebug
		printf("\r\n data save page3\r\n");
		#endif
		for(i=0;i<FlashHelperMaxLen/FlashHelperDataLen;i++)
		{
			flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperPageAddr[3]+i*FlashHelperDataLen*2;
			
			FlashHelperReadDataEx(flash_helper_struct.FlashHelperFlashAddr,data,FlashHelperDataLen);
			
			if(data[0]==0xFFFF && data[FlashHelperDataLen-1]==0xFFFF)
			{
				#ifdef FlashHelperDebug
				for(j=0;j<FlashHelperDataLen;j++){printf("%x ",flash_helper_struct.FlashHelperData[j]);}		
				#endif
				break;
			}
			for(j=0;j<FlashHelperDataLen;j++){flash_helper_struct.FlashHelperData[j] = data[j];}	
		}
	}
	else if(a1==0xAA && b1 == 0x55)//data save page0 ?
	{
		#ifdef FlashHelperDebug
		printf("\r\n data save page0 ?\r\n");
		#endif
		
		if(a2==0xAA && b2 == 0xFFFF)//data save page1
		{
			#ifdef FlashHelperDebug
			printf("\r\n data save page1\r\n");
			#endif
			for(i=0;i<FlashHelperMaxLen/FlashHelperDataLen;i++)
			{
				flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperPageAddr[1]+i*FlashHelperDataLen*2;
				
				FlashHelperReadDataEx(flash_helper_struct.FlashHelperFlashAddr,data,FlashHelperDataLen);

				if(data[0]==0xFFFF && data[FlashHelperDataLen-1]==0xFFFF)
				{
					#ifdef FlashHelperDebug
					for(j=0;j<FlashHelperDataLen;j++){printf("%x ",flash_helper_struct.FlashHelperData[j]);}		
					#endif
					break;
				}
				for(j=0;j<FlashHelperDataLen;j++){flash_helper_struct.FlashHelperData[j] = data[j];}
			}
		}
		else //data save page0 but page0 is full
		{
			#ifdef FlashHelperDebug
			printf("\r\n ata save page0 but page0 is full\r\n");
			#endif
			FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[0]+(FlashHelperMaxLen/FlashHelperDataLen-1)*FlashHelperDataLen*2,data,FlashHelperDataLen);
			for(j=0;j<FlashHelperDataLen;j++){
				flash_helper_struct.FlashHelperData[j] = data[j];
				#ifdef FlashHelperDebug
				printf("%x ",flash_helper_struct.FlashHelperData[j]);
				#endif
			}
			flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperPageAddr[1];
		}
	}
	else if(a2==0xAA && b2 == 0x55)//data save page1 ?
	{
		#ifdef FlashHelperDebug
		printf("\r\n data save page1 ?\r\n");
		#endif
		
		if(a3==0xAA && b3 == 0xFFFF)//data save page2
		{
			#ifdef FlashHelperDebug
			printf("\r\n data save page2\r\n");
			#endif
			for(i=0;i<FlashHelperMaxLen/FlashHelperDataLen;i++)
			{
				flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperPageAddr[2]+i*FlashHelperDataLen*2;
				
				FlashHelperReadDataEx(flash_helper_struct.FlashHelperFlashAddr,data,FlashHelperDataLen);

				if(data[0]==0xFFFF && data[FlashHelperDataLen-1]==0xFFFF)
				{
					#ifdef FlashHelperDebug
					for(j=0;j<FlashHelperDataLen;j++){printf("%x ",flash_helper_struct.FlashHelperData[j]);}		
					#endif
					break;
				}
				for(j=0;j<FlashHelperDataLen;j++){flash_helper_struct.FlashHelperData[j] = data[j];}
			}
		}
		else //data save page1 but page1 is full
		{
			#ifdef FlashHelperDebug
			printf("\r\n data save page1 but page1 is full\r\n");
			#endif
			FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[1]+(FlashHelperMaxLen/FlashHelperDataLen-1)*FlashHelperDataLen*2,data,FlashHelperDataLen);
			for(j=0;j<FlashHelperDataLen;j++){
				flash_helper_struct.FlashHelperData[j] = data[j];
				#ifdef FlashHelperDebug
				printf("%x ",flash_helper_struct.FlashHelperData[j]);
				#endif
			}
			flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperPageAddr[2];
		}
	}
	else if(a3==0xAA && b3 == 0x55)//data save page2 ?
	{
		#ifdef FlashHelperDebug
		printf("\r\n data save page2 ?\r\n");
		#endif
		
		if(a4==0xAA && b4 == 0xFFFF)//data save page3
		{
			#ifdef FlashHelperDebug
			printf("\r\n data save page3\r\n");
			#endif
			for(i=0;i<FlashHelperMaxLen/FlashHelperDataLen;i++)
			{
				flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperPageAddr[3]+i*FlashHelperDataLen*2;
				
				FlashHelperReadDataEx(flash_helper_struct.FlashHelperFlashAddr,data,FlashHelperDataLen);

				if(data[0]==0xFFFF && data[FlashHelperDataLen-1]==0xFFFF)
				{
					#ifdef FlashHelperDebug
					for(j=0;j<FlashHelperDataLen;j++){printf("%x ",flash_helper_struct.FlashHelperData[j]);}		
					#endif
					break;
				}
				for(j=0;j<FlashHelperDataLen;j++){flash_helper_struct.FlashHelperData[j] = data[j];}
			}
		}
		else //data save page2 but page2 is full
		{
			#ifdef FlashHelperDebug
			printf("\r\n data save page2 but page2 is full\r\n");
			#endif
			FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[2]+(FlashHelperMaxLen/FlashHelperDataLen-1)*FlashHelperDataLen*2,data,FlashHelperDataLen);
			for(j=0;j<FlashHelperDataLen;j++){
				flash_helper_struct.FlashHelperData[j] = data[j];
				#ifdef FlashHelperDebug
				printf("%x ",flash_helper_struct.FlashHelperData[j]);
				#endif
			}
			flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperPageAddr[3];
		}
	}
	else if(a4==0xAA && b4 == 0x55)//data save page3 ?
	{
		#ifdef FlashHelperDebug
		printf("\r\n data save page3 ?\r\n");
		#endif
		
		if(a1==0xAA && b1 == 0xFFFF)//data save page0
		{
			#ifdef FlashHelperDebug
			printf("\r\n data save page0\r\n");
			#endif
			for(i=0;i<FlashHelperMaxLen/FlashHelperDataLen;i++)
			{
				flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperPageAddr[0]+i*FlashHelperDataLen*2;
				
				FlashHelperReadDataEx(flash_helper_struct.FlashHelperFlashAddr,data,FlashHelperDataLen);

				if(data[0]==0xFFFF && data[FlashHelperDataLen-1]==0xFFFF)
				{
					#ifdef FlashHelperDebug
					for(j=0;j<FlashHelperDataLen;j++){printf("%x ",flash_helper_struct.FlashHelperData[j]);}		
					#endif
					break;
				}
				for(j=0;j<FlashHelperDataLen;j++){flash_helper_struct.FlashHelperData[j] = data[j];}
			}
		}
		else //data save page3 but page3 is full
		{
			#ifdef FlashHelperDebug
			printf("\r\n data save page3 but page3 is full\r\n");
			#endif
			FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[3]+(FlashHelperMaxLen/FlashHelperDataLen-1)*FlashHelperDataLen*2,data,FlashHelperDataLen);
			for(j=0;j<FlashHelperDataLen;j++){
				flash_helper_struct.FlashHelperData[j] = data[j];
				#ifdef FlashHelperDebug
				printf("%x ",flash_helper_struct.FlashHelperData[j]);
				#endif
			}
			flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperPageAddr[0];
		}
	}
}


void FlashHelperUpdate(void)
{
	u16 a,b;//判断上一页是否擦除
	
	/*处于第0页*/
	if( flash_helper_struct.FlashHelperFlashAddr >= flash_helper_struct.FlashHelperPageAddr[0] &&
			flash_helper_struct.FlashHelperFlashAddr < flash_helper_struct.FlashHelperPageAddr[1]
		)
	{
		//此页不够存储
		if((flash_helper_struct.FlashHelperFlashAddr + (FlashHelperDataLen<<1)) > (flash_helper_struct.FlashHelperPageAddr[0]+(FlashHelperMaxLen<<1)))
		{
			flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperPageAddr[1];//存储到第1页
			flash_helper_struct.FlashHelperFlashAddrCopy = flash_helper_struct.FlashHelperFlashAddr;
		}
		else
		{
			flash_helper_struct.FlashHelperFlashAddrCopy = flash_helper_struct.FlashHelperFlashAddr;//存储到此页
		}
		flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperFlashAddr + (FlashHelperDataLen<<1);
		FlashHelperWriteDataEx(flash_helper_struct.FlashHelperFlashAddrCopy,flash_helper_struct.FlashHelperData,FlashHelperDataLen);
		#ifdef FlashHelperDebug
		printf("\r\n FlashHelperFlashAddr: 0x%x",flash_helper_struct.FlashHelperFlashAddr);
		#endif
		/*擦除上一页*/
		FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[3],&a,1);//首地址数据
		FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[3]+(flash_helper_struct.FlashHelperTail<<1)-2,&b,1);//尾地址数据
		if(a != 0xFFFF || b != 0xFFFF)
		{
			#ifdef FlashHelperDebug
			printf("\r\n ErasePag3\r\n");
			#endif
			FlashHelperErasePageEx(flash_helper_struct.FlashHelperPageAddr[3],1);
		}
	}
	/*处于第1页*/
	else if(flash_helper_struct.FlashHelperFlashAddr >= flash_helper_struct.FlashHelperPageAddr[1] &&
			flash_helper_struct.FlashHelperFlashAddr < flash_helper_struct.FlashHelperPageAddr[2]
		)
	{
		//此页不够存储
		if((flash_helper_struct.FlashHelperFlashAddr + (FlashHelperDataLen<<1)) > (flash_helper_struct.FlashHelperPageAddr[1]+(FlashHelperMaxLen<<1)))
		{
			#ifdef FlashHelperDebug
			printf("\r\n FlashHelperMaxLen=%d\r\n",FlashHelperMaxLen);
			#endif
			flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperPageAddr[2];//存储到第2页
			flash_helper_struct.FlashHelperFlashAddrCopy = flash_helper_struct.FlashHelperFlashAddr;
		}
		else
		{
			flash_helper_struct.FlashHelperFlashAddrCopy = flash_helper_struct.FlashHelperFlashAddr;//存储到此页
		}
		flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperFlashAddr + (FlashHelperDataLen<<1);
		FlashHelperWriteDataEx(flash_helper_struct.FlashHelperFlashAddrCopy,flash_helper_struct.FlashHelperData,FlashHelperDataLen);
		
		#ifdef FlashHelperDebug
		printf("\r\n FlashHelperFlashAddr: 0x%x",flash_helper_struct.FlashHelperFlashAddr);
		#endif
		
		/*擦除上一页*/
		FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[0],&a,1);//首地址数据
		FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[0]+(flash_helper_struct.FlashHelperTail<<1)-2,&b,1);//尾地址数据
		if(a != 0xFFFF || b != 0xFFFF)
		{
			#ifdef FlashHelperDebug
			printf("\r\n TailAddr: 0x%x",flash_helper_struct.FlashHelperPageAddr[0]+(flash_helper_struct.FlashHelperTail<<1)-2);
			#endif
			
			
			#ifdef FlashHelperDebug
			printf("\r\n ErasePag0\r\n");
			#endif
			FlashHelperErasePageEx(flash_helper_struct.FlashHelperPageAddr[0],1);
		}
	}
	/*处于第2页*/
	else if(flash_helper_struct.FlashHelperFlashAddr >= flash_helper_struct.FlashHelperPageAddr[2] &&
		flash_helper_struct.FlashHelperFlashAddr < flash_helper_struct.FlashHelperPageAddr[3]
	)
	{
		//此页不够存储
		if((flash_helper_struct.FlashHelperFlashAddr + (FlashHelperDataLen<<1)) > (flash_helper_struct.FlashHelperPageAddr[2]+(FlashHelperMaxLen<<1)))
		{
			flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperPageAddr[3];//存储到第3页
			flash_helper_struct.FlashHelperFlashAddrCopy = flash_helper_struct.FlashHelperFlashAddr;
		}
		else
		{
			flash_helper_struct.FlashHelperFlashAddrCopy = flash_helper_struct.FlashHelperFlashAddr;//存储到此页
		}
		flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperFlashAddr + (FlashHelperDataLen<<1);
		FlashHelperWriteDataEx(flash_helper_struct.FlashHelperFlashAddrCopy,flash_helper_struct.FlashHelperData,FlashHelperDataLen);
		
		#ifdef FlashHelperDebug
		printf("\r\n FlashHelperFlashAddr: 0x%x",flash_helper_struct.FlashHelperFlashAddr);
		#endif
		
		/*擦除上一页*/
		FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[1],&a,1);//首地址数据
		FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[1]+(flash_helper_struct.FlashHelperTail<<1)-2,&b,1);//尾地址数据
		if(a != 0xFFFF || b != 0xFFFF)
		{
			#ifdef FlashHelperDebug
			printf("\r\n TailAddr: 0x%x",flash_helper_struct.FlashHelperPageAddr[1]+(flash_helper_struct.FlashHelperTail<<1)-2);
			#endif
			
			#ifdef FlashHelperDebug
			printf("\r\n ErasePag1\r\n");
			#endif
			FlashHelperErasePageEx(flash_helper_struct.FlashHelperPageAddr[1],1);
		}
	}
	else //处于第三页
	{	
		//此页不够存储
		if((flash_helper_struct.FlashHelperFlashAddr + (FlashHelperDataLen<<1)) > (flash_helper_struct.FlashHelperPageAddr[3]+(FlashHelperMaxLen<<1)))
		{
			flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperPageAddr[0];//存储到第0页
			flash_helper_struct.FlashHelperFlashAddrCopy = flash_helper_struct.FlashHelperFlashAddr;
		}
		else
		{
			flash_helper_struct.FlashHelperFlashAddrCopy = flash_helper_struct.FlashHelperFlashAddr;//存储到此页
		}
		flash_helper_struct.FlashHelperFlashAddr = flash_helper_struct.FlashHelperFlashAddr + (FlashHelperDataLen<<1);
		FlashHelperWriteDataEx(flash_helper_struct.FlashHelperFlashAddrCopy,flash_helper_struct.FlashHelperData,FlashHelperDataLen);
		
		#ifdef FlashHelperDebug
		printf("\r\n FlashHelperFlashAddr: 0x%x",flash_helper_struct.FlashHelperFlashAddr);
		#endif
		
		/*擦除上一页*/
		FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[2],&a,1);//首地址数据
		FlashHelperReadDataEx(flash_helper_struct.FlashHelperPageAddr[2]+(flash_helper_struct.FlashHelperTail<<1)-2,&b,1);//尾地址数据
		if(a != 0xFFFF || b != 0xFFFF)
		{
			#ifdef FlashHelperDebug
			printf("\r\n ErasePag2\r\n");
			#endif
			FlashHelperErasePageEx(flash_helper_struct.FlashHelperPageAddr[2],1);
		}
	}
}



/**
* @brief   获取索引处的数据
* @param   index 索引值
* @param   None
* @param   None
* @retval  None
* @warning 调用完刷新函数以后需要立即调用此函数,才是读出的刚刚存入的真实数据
* @example 
**/
u16 FlashHelperGetIndex(u16 index)
{
	u16 data;
	FlashHelperReadDataEx(flash_helper_struct.FlashHelperFlashAddrCopy+(index<<1),&data,1);
	return data;
}
