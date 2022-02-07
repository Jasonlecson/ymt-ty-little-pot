#ifndef __USER_FLASH_H__
#define __USER_FLASH_H__



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
uint8_t Write_Flash(uint32_t Addr_Obj,uint8_t *Addr_Sour,uint16_t Length);

/*****************************************************************************
函数名称 : Earse_Flash
功能描述 : 对Flash擦除
输入参数 : Addr_Obj:FLASH的目标起始地址
返回参数 : SUCCESS:擦除成功
            ERROR:擦除失败
使用说明 : 无
*****************************************************************************/
uint8_t Earse_Flash(uint32_t Addr_Obj);

/*****************************************************************************
函数名称 : Read_Flash
功能描述 : 读取FLASH的数据
输入参数 : pulData:源数据
           ulAddress:FLASH的起始地址
           Length:读取的字节数
返回参数 : true:读取成功
           false:读取失败
*****************************************************************************/
uint8_t Read_Flash(uint8_t *pulData,uint32_t ulAddress,uint32_t Length);




//读取指定地址的半字(16位数据)
//faddr:读地址(此地址必须为2的倍数!!)
//返回值:对应数据.
u16 STMFLASH_ReadHalfWord(u32 faddr);


//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);


//不检查的写入
//WriteAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数   
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);

void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);

#endif
