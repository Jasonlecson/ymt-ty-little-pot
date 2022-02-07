#ifndef CSTRING_H_
#define CSTRING_H_
#include <stm32f10x.h>
#ifndef CSTRING_C_//如果没有定义
#define CSTRING_Ex_ extern
#else
#define CSTRING_Ex_
#endif

#define MAXList 20
CSTRING_Ex_ char *split_buff[MAXList];

void cStringFree(void);
void cStringRestore(void);
char *StrBetwString(char *Str,char *StrBegin,char *StrEnd);
int split(char *src,const char *separator,char **dest,int DestLen);
void HexToStr(char *pbSrc,char *pbDest,int nlen);
void StrToHex(char *pbSrc, char *pbDest, int nLen);
#endif

