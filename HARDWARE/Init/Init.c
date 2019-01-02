#include "Init.h"
#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "mpu6050.h"  
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "ANO.h"
#include "ANO_Drv_Nrf24l01.h"
#include "stmflash.h"
#include "Parameter.h"
#include "exti.h"

void Init_sys(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
//	uart_init(115200);	 	//串口初始化为500000
	uart2_init(921600);
	delay_init();	//延时初始化 
	Param_Init();
//	LED_Init();		  			//初始化与LED连接的硬件接口
//	KEY_Init();					//初始化按键
	MPU_Init();					//初始化MPU6050
//	Param_Init();				//上电初始化从flash读取参数 
	PS2_Init();			 //驱动端口初始化
	PS2_SetInit();		 //配配置初始化,配置“红绿灯模式”，并选择是否可以修改
	                     //开启震动模式 
	ANO_SPI_Init();	
	MiniBalance_PWM_Init(7199,0);
	
	if(mpu_dmp_init())   //MPU_DMP初始化
 	{
		printf("MPU6050 Error\r\n");
 		delay_ms(100);
	}
	else	
		printf("MPU6050 OK\r\n");
	
	MPU6050_EXTI_Init();

}



