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
#include "Init.h"
u8 Tx_Rx_mode = Tx_mode;  //24l01模式 Tx_mode接受 Rx_mode发送
float pit,roll,yaw; 		//欧拉角
short aacx,aacy,aacz;		//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据
short temp;					//温度	
 int main(void)
 {	 
	u16 i=0,j=0;	
	 
	Init_sys();  //初始化
 	
 	while(1)
	{	

				///////////////////////////*分时上传*///////////////////////////////////////////
				i++;
				if(i==1)
					ANO_DT_Send_Status(-roll, pit, -yaw, 0, 0, 0);///飞控姿态等基本信息
//				if(i==2)
//					ANO_DT_Send_Senser(aacx,aacy,aacz,gyrox,gyroy,gyroz,0,0,0); //飞机传感器数据//发送加速度和陀螺仪原始数据
//				if(i==3)
					ANO_DT_Send_MotoPWM(set_speed,(int)Turn_Target,0,0,0,0,0,0);				
//				if(i==4)
//					ANO_DT_Send_User1( );	
				if(i>=1) i=0;
				
				j++;
				if(j==2500)//温度长时间才上传一次
				{	
					j=0;
					temp=MPU_Get_Temperature();
					ANO_DT_Send_Power(111, temp);  //这里电流实际为MPU6050的温度传感器
				}
				delay_ms(1);
	}			
 
}

	 

 
