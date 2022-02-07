#ifndef DATAPARSE_Cc_
#define DATAPARSE_Cc_

#ifndef DATAPARSE_Cx_
#define DATAPARSE_Cx_ extern
#else
#define DATAPARSE_Cx_
#endif

typedef union Resolve
{
  long long lld_data;
  unsigned short int  u16_table[4];
}Resolve_Typedef;

DATAPARSE_Cx_ Resolve_Typedef Resolve_Data;

unsigned char CRC8(unsigned char *p,int counter);

unsigned short int SortMethod(unsigned short int *r,unsigned short int DataNumber,char LostNumber);
#endif

