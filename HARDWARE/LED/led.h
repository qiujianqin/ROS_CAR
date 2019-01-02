#ifndef __LED_H               //这样写头文件可以避免重复定义
#define __LED_H	 
#include "sys.h"

#define LED0 PAout(5)// PA5
#define LED1 PCout(13)// PC13
void LED_Init(void);//初始化

		 				    
#endif
