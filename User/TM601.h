#ifndef __TM601_I2cSet_H_
#define __TM601_I2cSet_H_

#include <stm32f10x.h>
#include "System.h"

#define TM601_SDA_OUT	PBout(10)
#define TM601_SDA_IN	PBin(10)
#define SCL			PBout(10)
#define CON_ADDR    0x40



char	i2c_read_direct(unsigned dev_addr,unsigned char * dest_buf,unsigned char len);
char	i2c_write(unsigned dev_addr, unsigned reg_addr, unsigned char * src_buf, unsigned char len);

char ReadThresholdFrom601(unsigned char chnelnum,unsigned int *sample);
char SetThresholdTo601(unsigned char chnelNum,unsigned int *sample);

#endif


