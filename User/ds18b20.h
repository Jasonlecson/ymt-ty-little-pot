#ifndef DS18B20_Cc_
#define DS18B20_Cc_

#ifndef DS18B20_Cx_//如果没有定义
#define DS18B20_Cx_ extern
#else
#define DS18B20_Cx_
#endif

/*config pin*/
#define DS18B20_CLK    RCC_APB2Periph_GPIOA
#define DS18B20_PIN    GPIO_Pin_7
#define DS18B20_PORT   GPIOA


//初始化DS18B20的IO口,检测传感器是否存在(移植请替换内部函数) 
char DS18B20Init(void);
//复位DS18B20
void DS18B20_Rst(void);
//等待DS18B20的回应
//返回1:未检测到DS18B20的存在
//返回0:存在
char DS18B20_Check(void);

short DS18B20_Get_Temp(void);
#endif

