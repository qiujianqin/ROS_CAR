#include "Parameter.h" 
u16 PID_Param[PID_Param_LEN];//PID参数保存数组
float Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki,Turn_Kp,Turn_Ki,Turn_Kd,Way_Angle,set_speed,ZHONGZHI,Turn_Target=0;
u8 set_speed1=1,set_speed2=0,set_speed3=0,set_speed4=0;
void Param_Read(void)//从flash读取参数
{
	STMFLASH_Read(FLASH_SAVE_ADDR,(u16*)PID_Param,PID_Param_LEN);
	if(PID_Param[0]==65535&&PID_Param[1]==65535&&PID_Param[2]==65535&&PID_Param[3]==65535)
	{
	Balance_Kp=300;
	Balance_Kd=1;
	Velocity_Kp=80;
	Velocity_Ki=0.4;
	Turn_Kp=25;//第一组直线行走控制（也可以叫转向位置式）PID的参数 注意与速度配套
	Turn_Ki=0.5;
	Turn_Kd=0.5;
	Way_Angle=0;  
	set_speed =4500;
	ZHONGZHI=0;
	Turn_Target=0;
	}
	else
	{		
		Balance_Kp=(float)PID_Param[PID_KP1]/100; //还原为float型 真实数值
		Balance_Kd=(float)PID_Param[PID_KD1]/100;
		
		Velocity_Kp=(float)PID_Param[PID_KP2]/100;
		Velocity_Ki=(float)PID_Param[PID_KI2]/100;
		
		Turn_Kp=(float)PID_Param[PID_KP3]/100;
		Turn_Ki=(float)PID_Param[PID_KI3]/100;
		Turn_Kd=(float)PID_Param[PID_KD3]/100;
		
		Way_Angle=(u8)PID_Param[PID4_Way_Angle];

		set_speed=(int)PID_Param[PID5_set_speed];
		ZHONGZHI=(u16)PID_Param[PID5_ZHONGZHI];
//		Turn_Target=(float)PID_Param[PID5_Turn_Target]/100;
	}

}

void Param_Save(void)//向flash写入参数
{
	PID_Param[PID_KP1]=Balance_Kp*100;	//放大100存储，避免出现浮点数
	PID_Param[PID_KD1]=Balance_Kd*100;	//放大100存储，避免出现浮点数
	
	PID_Param[PID_KP2]=Velocity_Kp*100;	//放大100存储，避免出现浮点数
	PID_Param[PID_KI2]=Velocity_Ki*100;	//放大100存储，避免出现浮点数
	
	PID_Param[PID_KP3]=Turn_Kp*100;	//放大100存储，避免出现浮点数
	PID_Param[PID_KI3]=Turn_Ki*100;	//放大100存储，避免出现浮点数
	PID_Param[PID_KD3]=Turn_Kd*100;	//放大100存储，避免出现浮点数	
	
	PID_Param[PID4_Way_Angle]=Way_Angle;
	
	PID_Param[PID5_set_speed]=set_speed;
	PID_Param[PID5_ZHONGZHI]=ZHONGZHI;
//	PID_Param[PID5_Turn_Target]=Turn_Target*100;
	
	STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)PID_Param,PID_Param_LEN);	
}

void Para_ResetToFactorySetup(void) //恢复默认参数
{
	Balance_Kp=300;
	Balance_Kd=1;
	Velocity_Kp=80;
	Velocity_Ki=0.4;
	Turn_Kp=25;//第一组直线行走控制（也可以叫转向位置式）PID的参数 注意与速度4500配套
	Turn_Ki=0.5;
	Turn_Kd=0.5;
	Way_Angle=0;  
	set_speed =4500;
	ZHONGZHI=0;
	Turn_Target=0;
	
	Param_Save();	//保存到flash
}
void Param_Init(void)//上电从flash读取参数
{
	Param_Read();

}



