#include "control.h"		
u8 key_PS2=0,PS_PSS_RY=0,PS_PSS_RX=0,PS_PSS_LX=0,PS_PSS_LY=0;
u8 Flag_5ms=0,Flag_10ms=0,Flag_stop=1,Angle_warning=0;
int Moto1,Moto2,Moto3,Moto4;   
int turn_PWM = 0;
/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	if(INT==0)		
	{   
		EXTI->PR=1<<14;                                                      //清除中断标志位	
		Flag_5ms=!Flag_5ms;
		if(Flag_5ms==1)	//5ms读取一次陀螺仪和加速度计的值，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果
		{
			if(mpu_dmp_get_data(&pit,&roll,&yaw)==0)
			{
//					MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
//					MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			}
			return 0;
		}
		
		//10ms控制一次，为了保证M法测速的时间基准，首先读取编码器数据		

		if(mpu_dmp_get_data(&pit,&roll,&yaw)==0)//更新姿态
		{
//			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
//			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
		}
		  turn_PWM=turn(Turn_Target,-yaw);
		  KEY_ps2_handle();//PS2按键扫描处理
		
		 if(yaw>=175||yaw<=-175)//角度异常
		 {
			Angle_warning=1;//角度异常标志位
			Set_Pwm(0,0,0,0); //停车
			PS2_Vibration(254,254);//手柄震动警告
		 }
		 if(Angle_warning==1)//角度异常处理 复位MPU6050 以当前朝向为目标角度
		 {
			Angle_warning=0;
			mpu_dmp_init();//复位6050
			mpu_dmp_get_data(&pit,&roll,&yaw);
			Turn_Target=0;//校准当前目标角度
			PS2_Vibration(0,0);
		 }
		 
   		  Xianfu_Pwm();                                                       //===PWM限幅
		
		  if(Flag_stop==1)//停车
		  {
			Set_Pwm(0,0,0,0); 
		  }
		  else
			Set_Pwm(Moto1,Moto2,Moto3,Moto4);                                               //===赋值给PWM寄存器  
	}       	
	 return 0;	  
} 

/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修改set_speed
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
作    者：平衡小车之家
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{  
     static float Velocity;

	  return Velocity;
}

/**************************************************************************
函数功能：转向控制  
入口参数：左轮编码器、右轮编码器、Z轴陀螺仪
返回  值：转向控制PWM
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
*************
**************************************************************************/
int turn(float Target,float Yaw)//转向控制 输入 Target为目标航向角 yaw是实际航向角
{
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	
		if(Flag_stop==1) //停车清空PID参数 避免累积
	 {
		 Bias=0;
		 Pwm=0;
		 Integral_bias=0;
		 Last_Bias=0;
	 }
	 
	 Bias=-yaw-Target;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 if(Integral_bias>7200) Integral_bias=7200;        //积分限幅
	 if(Integral_bias<-7200) Integral_bias=-7200;
	 Pwm=Turn_Kp*Bias+Turn_Ki*Integral_bias+Turn_Kd*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //输出PWM
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
注释：车轮转动方向判断 面对车轮 PWM为正时顺时针旋转 PWM为负时逆时针旋转
**************************************************************************/
void Set_Pwm(int moto1,int moto2,int moto3,int moto4)
{
    	if(moto1>0)			AIN2=0,			AIN1=1;
			else 	          AIN2=1,			AIN1=0;
			PWMA=myabs(moto1);
		  if(moto2>0)	BIN1=0,			BIN2=1;
			else        BIN1=1,			BIN2=0;
			PWMB=myabs(moto2);
    	if(moto3>0)			CIN2=0,			CIN1=1;
			else 	          CIN2=1,			CIN1=0;
			PWMC=myabs(moto3);
    	if(moto4>0)			DIN2=1,			DIN1=0;
			else 	          DIN2=0,			DIN1=1;
			PWMD=myabs(moto4);	
}

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int maxSpeed=7100;    //===PWM满幅是7200 限制在7100
      if(Moto1<-maxSpeed)
	  {		  
		  Moto1=-maxSpeed;	
	  }
	  else if(Moto1>maxSpeed)  
	 {

		  Moto1=maxSpeed;	
	 }
      if(Moto2<-maxSpeed)
	  {		  
		  Moto2=-maxSpeed;	
	  }
	  else if(Moto2>maxSpeed)  
	 {

		  Moto2=maxSpeed;	
	 }
	 if(Moto3<-maxSpeed)
	  {		  
		  Moto3=-maxSpeed;	
	  }
	  else if(Moto1>maxSpeed)  
	 {

		  Moto3=maxSpeed;	
	 }
	 if(Moto4<-maxSpeed)
	  {		  
		  Moto4=-maxSpeed;	
	  }
	  else if(Moto4>maxSpeed)  
	 {

		  Moto4=maxSpeed;	
	 }	 
}



	
/**************************************************************************
函数功能：获取角度 三种算法经过我们的调校，都非常理想 
入口参数：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/
void Get_Angle(u8 way)
{ 

}
/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
函数功能：ps2手柄按键处理函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
void KEY_ps2_handle(void)
{
	key_PS2=PS2_DataKey();									//获取PS手柄按键信息
	PS_PSS_RX=PS2_AnologData(PSS_RX);						//获取摇杆模拟量0-254
	PS_PSS_RY=PS2_AnologData(PSS_RY);
	PS_PSS_LX=PS2_AnologData(PSS_LX);
	PS_PSS_LY=PS2_AnologData(PSS_LY);
	switch(key_PS2)//按键处理
	{
		case 0://none
			Moto1=0;
			Moto2=0;
			Moto3=0;
			Moto4=0;
			break;
		case 1: //select
			Turn_Target=-yaw;
			break;
		case 2://L摇杆中键
			Moto1=0;
			Moto2=0;
			Moto3=0;
			Moto4=0;
			break;
		case 3:	//R摇杆中键							
			Moto1=0;
			Moto2=0;
			Moto3=0;
			Moto4=0;
			break;
		case 4://start
			mpu_dmp_init();//复位6050
			Turn_Target=0;//校准当前目标角度
			break;
		case 5:   //上
			Moto1=-set_speed+turn_PWM;
			Moto2=set_speed+turn_PWM;
			Moto3=set_speed+turn_PWM;
			Moto4=-set_speed+turn_PWM;
			break;
		case 6://右
			Moto1=-set_speed+turn_PWM;
			Moto2=-set_speed+turn_PWM;
			Moto3=set_speed+turn_PWM;
			Moto4=set_speed+turn_PWM;
			break;
		case 7://下
			Moto1=set_speed+turn_PWM;
			Moto2=-set_speed+turn_PWM;
			Moto3=-set_speed+turn_PWM;
			Moto4=set_speed+turn_PWM;
			break;
		case 8://左
			Moto1=set_speed+turn_PWM;
			Moto2=set_speed+turn_PWM;
			Moto3=-set_speed+turn_PWM;
			Moto4=-set_speed+turn_PWM;
			break;
		case 9://L2
			Moto1=0;
			Moto2=0;
			Moto3=0;
			Moto4=0;
			break;
		case 10://R2
			Moto1=0;
			Moto2=0;
			Moto3=0;
			Moto4=0;
			break;	
		case 11://L1
			Moto1=4000;
			Moto2=4000;
			Moto3=4000;
			Moto4=4000;
			Turn_Target=-yaw;
			break;
		case 12://R1
			Moto1=-4000;
			Moto2=-4000;
			Moto3=-4000;
			Moto4=-4000;
			Turn_Target=-yaw;
			break;
		case 13://三角形
			set_speed=5000;		
			break;
		case 14://圆形
			set_speed=6000;		
			break;
		case 15://X
			set_speed=7000;		
			break;
		case 16://正方形
			set_speed=4500;		
			break;
		default : 
			break; 		
	}
       /********摇杆处理*********/
	if(PS_PSS_LY>131)
	{
		Flag_stop = 0;		
			Moto1=set_speed+turn_PWM;
			Moto2=-set_speed+turn_PWM;
			Moto3=-set_speed+turn_PWM;
			Moto4=set_speed+turn_PWM;


		}
	else if(PS_PSS_LY<125)
	{
		Flag_stop = 0;			
			Moto1=-set_speed+turn_PWM;
			Moto2=set_speed+turn_PWM;
			Moto3=set_speed+turn_PWM;
			Moto4=-set_speed+turn_PWM;

	}
	
	if(PS_PSS_RX>131)
	{
			Flag_stop = 0;
		//Turn_Target=(float)(PS_PSS_RX-128)/128*50;
			Moto1=-set_speed+turn_PWM;
			Moto2=-set_speed+turn_PWM;
			Moto3=set_speed+turn_PWM;
			Moto4=set_speed+turn_PWM;		
	}
	else if(PS_PSS_RX<125)
	{
			Flag_stop = 0;
		//Turn_Target = -(float)(128-PS_PSS_RX)/128*50;
			Moto1=set_speed+turn_PWM;
			Moto2=set_speed+turn_PWM;
			Moto3=-set_speed+turn_PWM;
			Moto4=-set_speed+turn_PWM;		
	}
	else if(PS_PSS_LY>125&&PS_PSS_LY<131&&PS_PSS_RX>125&&PS_PSS_RX<131&&key_PS2==0) 
	{
			Flag_stop = 1;
	}
	if(key_PS2==11||key_PS2==12)
	{
			Flag_stop = 0;
	}
		
	
	
	
	


//		  


}




