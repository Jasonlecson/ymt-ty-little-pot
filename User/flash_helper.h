#ifndef FLASH_HELPER_H_
#define FLASH_HELPER_H_

#ifndef FLASH_HELPER_C_//如果没有定义
#define FLASH_HELPER_Cx_ extern
#else
#define FLASH_HELPER_Cx_
#endif
#include "stm32f10x.h"
#include "stmflash.h"


/*默认擦写寿命*/
/*FlashHelperMaxLen/FlashHelperDataLen*4*Flash擦写次数*/
/*510/20*4*10000 = 100万次*/
/*1022/20*4*10000 = 204万次*/

//#define FlashHelperDebug  

#if STM32_FLASH_SIZE<256  //Flash容量
#define FlashHelperMaxLen 510
#else 
#define FlashHelperMaxLen 1022
#endif	

#define FlashHelperDataLen 20 //最大存储的数据个数(u16),根据用户实际修改.


#define FlashHelperPageNumber  4   //几块分区(修改此参数,需要修改增加底层程序)
#define FlashHelperAddressOffset 4 //存储地址偏移字节数(不可修改)

typedef struct flash_helper
{
	/*存储用户数据,用户把数据存储在此数组*/
	u16 FlashHelperData[FlashHelperDataLen];
	u16 FlashHelperTail;
	/*保存数据用的分区首地址*/
	u32 FlashHelperPageAddr[FlashHelperPageNumber];
	/*记录当前操作的地址*/	
	u32 FlashHelperFlashAddr;
	u32 FlashHelperFlashAddrCopy;
	
} flash_helper_typedef;

FLASH_HELPER_Cx_ flash_helper_typedef flash_helper_struct;


void FlashHelperUpdate(void);
void FlashHelperInit(u32 ADDR1,u32 ADDR2,u32 ADDR3,u32 ADDR4);
u16 FlashHelperGetIndex(u16 index);
#endif



