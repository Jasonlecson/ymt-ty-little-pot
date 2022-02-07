#define DATAPARSE_C_

#include "DataParse.h"

Resolve_Typedef Resolve_Data;

unsigned char CRC8Table[256]={
  0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
  157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
  35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
  190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
  70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
  219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
  101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
  248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
  140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
  17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
  175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
  50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
  202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
  87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
  233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
  116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

/**
* @brief  CRC8
* @param  p:源数据
* @param  counter:数据长度
* @param  None
* @retval CRC8 校验值
* @example 
**/
unsigned char CRC8(unsigned char *p,int counter)
{
	unsigned char crc8 = 0;

	for( ; counter > 0; counter--){
		crc8 = CRC8Table[crc8^*p];
		p++;
	}
	return(crc8);
}


/**
* @brief  排序函数
* @param  *table--要排序的数组首地址
* @param  DataNumber--要排序的数据个数
* @param  
* @param  
* @param  LostNumber -- 左右丢弃个数
* @retval 排序丢弃后剩余的和
* @example 
**/
unsigned short int SortMethod(unsigned short int *r,unsigned short int DataNumber,char LostNumber)
{
	unsigned short int low = 0;   
	unsigned short int high= DataNumber -1; //设置变量的初始值  
	unsigned short int tmp,j;  
	unsigned short int sum = 0;
	while (low < high) {  
			for (j= low; j< high; ++j) //正向冒泡,找到最大者  
					if (r[j]> r[j+1]) {  
							tmp = r[j]; r[j]=r[j+1];r[j+1]=tmp;  
					}   
			--high;                 //修改high值, 前移一位  
			for ( j=high; j>low; --j) //反向冒泡,找到最小者  
					if (r[j]<r[j-1]) {  
							tmp = r[j]; r[j]=r[j-1];r[j-1]=tmp;  
					}  
			++low;                  //修改low值,后移一位  
	} 
  for(j=LostNumber;j<DataNumber-LostNumber;j++)
	{
    sum+=r[j];
  }	
	
	return sum;
}






