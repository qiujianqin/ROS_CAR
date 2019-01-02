/******************** (C) COPYRIGHT 2016 ANO Tech ***************************
 * 作者		 ：小丘科技
 * 文件名  ：ANO.c
 * 描述    ：数据传输函数
*****************************************************************************/
#include "ANO.h"
#include "Parameter.h"
#include "sys.h"
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (u8 *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (u8 *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (u8 *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (u8 *)(&dwTemp) + 3) )
//	data_to_send[_cnt++]=KD_test; 8位数据

//	data_to_send[_cnt++]=BYTE1(KD_test); 16位数据
//  data_to_send[_cnt++]=BYTE0(KD_test);

//	data_to_send[_cnt++]=BYTE3(KD_test); 32位数据
//  data_to_send[_cnt++]=BYTE2(KD_test);	
//  data_to_send[_cnt++]=BYTE1(KD_test); 
//  data_to_send[_cnt++]=BYTE0(KD_test);
/******************************************************************************/
u8 data_to_send[50];			//发送数据缓存
//extern u8 NRF24L01_2_RXDATA_LEN;

//移植时主要修改这里的串口发送函数
void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
	u8 i;
	for(i=0;i<length;i++)
				{
					while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕    //这是发送为16进制的数显示
					USART_SendData(USART2,dataToSend[i]); 
				}

}

void Uart1_Put_Buf(u8 *dataToSend , u8 length)   //24l01收到的数据发送至串口
{
//	u8 i;
//	for(i=0;i<NRF24L01_2_RXDATA_LEN;i++)
//				{
//					

//					while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕    //这是发送为16进制的数显示
//					USART_SendData(USART1,NRF24L01_2_RXDATA[i]); 
//				}
//	
}

//发送检验 告诉上位机发送成功
static void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	u8 sum = 0;
	u8 i=0;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	for(i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}
/*
static void ANO_DT_Send_Msg(u8 id, u8 data)
{
	u8 sum = 0;
	u8 i=0;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEE;
	data_to_send[3]=2;
	data_to_send[4]=id;
	data_to_send[5]=data;
	
	for(i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}
*/
//飞控姿态等基本信息
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)  
{
	u8 _cnt=0;
	u8 sum=0,i = 0;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);  //alt
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z) //飞控传感器原始数据
{
	u8 _cnt=0;
	u8 sum=0,i = 0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
/////////////////////////////////////////
	_temp = 0;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

//电压，电流
void ANO_DT_Send_Power(u16 votage, u16 current)
{
	u8 _cnt=0;	
	u8 sum = 0;
	u8 i=0;
	u16 temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
//电机输出PWM
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
//PID数据帧 注意group就是第几帧PID 如意group = 1 即是PID1
void ANO_DT_Send_PID(u8 group,s16 p1_p,s16 p1_i,s16 p1_d,s16 p2_p,s16 p2_i,s16 p2_d,s16 p3_p,s16 p3_i,s16 p3_d)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	_temp = p1_p ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}
//用户数据1
void ANO_DT_Send_User1(void)   
{
	u8 _cnt=0;
	u8 sum=0,i = 0;
	
	data_to_send[_cnt++]=0xAA; 
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xf1; //用户数据
	data_to_send[_cnt++]=0;
	
//	data_to_send[_cnt++]=BYTE1(KP_test);
//	data_to_send[_cnt++]=BYTE0(KP_test);
//	
//	data_to_send[_cnt++]=BYTE1(KI_test);
//	data_to_send[_cnt++]=BYTE0(KI_test);
//	
//	data_to_send[_cnt++]=BYTE1(KD_test);
//	data_to_send[_cnt++]=BYTE0(KD_test);
//	
//	data_to_send[_cnt++]=BYTE1(KP_test1);
//	data_to_send[_cnt++]=BYTE0(KP_test1);
//	data_to_send[_cnt++]=BYTE1(KP_test2);
//	data_to_send[_cnt++]=BYTE0(KP_test2);
//	data_to_send[_cnt++]=BYTE1(KP_test3);
//	data_to_send[_cnt++]=BYTE0(KP_test3);		
//	data_to_send[_cnt++]=BYTE1(KP_test4);
//	data_to_send[_cnt++]=BYTE0(KP_test4);		
//	data_to_send[_cnt++]=BYTE1(KP_test5);
//	data_to_send[_cnt++]=BYTE0(KP_test5);		
//	data_to_send[_cnt++]=BYTE1(KP_test6);
//	data_to_send[_cnt++]=BYTE0(KP_test6);		
//	data_to_send[_cnt++]=BYTE1(KP_test7);
//	data_to_send[_cnt++]=BYTE0(KP_test7);		
//	data_to_send[_cnt++]=BYTE1(KP_test8);
//	data_to_send[_cnt++]=BYTE0(KP_test8);		
//	data_to_send[_cnt++]=BYTE1(KP_test9);
//	data_to_send[_cnt++]=BYTE0(KP_test9);		
//	data_to_send[_cnt++]=BYTE1(KP_test10);
//	data_to_send[_cnt++]=BYTE0(KP_test10);		

	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}
//////////////////
void ANO_DT_Send_PID_ALL(void)
{
			Param_Read();//从flash中读取参数
//			for(j=0;j<10;j++) //注意这里是多发几次，避免上位机接收失败或者传输干扰！
//			{
				ANO_DT_Send_PID(1,
								PID_Param[PID_KP1],
								PID_Param[PID_KI1],
								PID_Param[PID_KD1],
								PID_Param[PID_KP2],
								PID_Param[PID_KI2],
								PID_Param[PID_KD2],
								PID_Param[PID_KP3],
								PID_Param[PID_KI3],
								PID_Param[PID_KD3]); //上传PID1-3组参数
				ANO_DT_Send_PID(2,
								PID_Param[PID4_Way_Angle],
								0,
								0,
								PID_Param[PID5_set_speed],
								PID_Param[PID5_ZHONGZHI],
								(int)Turn_Target*100,
								0,
								0,
								0); //上传PID4-6组参数			
//			}
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数ANO_DT_Data_Receive（）；
void ANO_DT_Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
//此函数可作为指令控制，在下面函数中添加指令即可
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	u8 i=0;
	u8 j = 0;
	for(i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
		{
//			sensor.acc_CALIBRATE = 1;//ACC校准
			//sensor.Cali_3d = 1;
//			ANO_Flag_Stop = 1;  //停车 保持直立 并且在上位机中显示为锁定 修改上位机按键功能
		}
		else if(*(data_buf+4)==0X02)
		{
//			sensor.gyr_CALIBRATE = 1;//GYRO校准
//			ANO_Flag_Stop = 0;  //启动  前进后退 并且在上位机中显示为解锁 修改上位机按键功能
		}
		else if(*(data_buf+4)==0X03)
		{
//			sensor.acc_CALIBRATE = 1;//ALT高度校准（仅用于光流模块）	
//			sensor.gyr_CALIBRATE = 1;			
		}
		else if(*(data_buf+4)==0X31)
		{
	/*		ANO_Uart1_DeInit();
			ANO_UART3_DeInit();
			Delay_us(100);
			ANO_Uart1_Init(576000);
			ANO_UART3_Init(576000);
			Esp8266_Init();
			Esp8266_SetGpio0(0);
			Esp8266_SetEnable(0);
			Delay_us(100);
			Esp8266_SetEnable(1);
			flag.espDownloadMode = 1;
			ANO_LED_0_ON();
			ANO_LED_1_ON();*/
		}
		else if(*(data_buf+4)==0XA0)//飞控锁定（仅用于手机蓝牙控制）
		{
//			fly_ready = 0;			
		}
		else if(*(data_buf+4)==0XA1)//飞控解锁（仅用于手机蓝牙控制）
		{
//			fly_ready = 1;			
		}
	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)//读取PID请求（返回AAAA 10\11\12\13\14\15数据帧）
		{
			ANO_DT_Send_PID_ALL();  //重新上传所有参数

		}
		if(*(data_buf+4)==0X02)//读取飞行模式设置请求（返回AAAA 0A数据帧）
		{
			
		}
		if(*(data_buf+4)==0XA0)		//读取版本信息
		{
//			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//恢复默认参数
		{
			Para_ResetToFactorySetup();
			
			ANO_DT_Send_PID_ALL();  //重新上传所有参数			
		}
	}

	if(*(data_buf+2)==0X03)//飞行控制数据(仅用于微型飞机，无刷飞机请用航模遥控控制
	{
/*		flag.NS = 2;
		
		RX_CH[THR] = (vs16)(*(data_buf+4)<<8)|*(data_buf+5) ;
		RX_CH[YAW] = (vs16)(*(data_buf+6)<<8)|*(data_buf+7) ;
		RX_CH[ROL] = (vs16)(*(data_buf+8)<<8)|*(data_buf+9) ;
		RX_CH[PIT] = (vs16)(*(data_buf+10)<<8)|*(data_buf+11) ;
		RX_CH[AUX1] = (vs16)(*(data_buf+12)<<8)|*(data_buf+13) ;
		RX_CH[AUX2] = (vs16)(*(data_buf+14)<<8)|*(data_buf+15) ;
		RX_CH[AUX3] = (vs16)(*(data_buf+16)<<8)|*(data_buf+17) ;
		RX_CH[AUX4] = (vs16)(*(data_buf+18)<<8)|*(data_buf+19) ;*/
	}
//////////////////////////////////////////////////////////////	
//接收PID数据帧 1到6
//共有18组PID，每3组PID组成一个PID数据帧，所以共6个PID数据帧
	if(*(data_buf+2)==0X10)								//PID1
    {
		PID_Param[PID_KP1] =  ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
//		PID_Param[PID_KI1] =  ( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		PID_Param[PID_KD1] =  ( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
      
        PID_Param[PID_KP2] = ( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        PID_Param[PID_KI2] = ( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//        PID_Param[PID_KD2] = ( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
		
        PID_Param[PID_KP3] 	= ( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        PID_Param[PID_KI3] 	= ( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        PID_Param[PID_KD3] 	= ( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		
		Balance_Kp=(float)PID_Param[PID_KP1]/100; //还原为float型 真实数值
		Balance_Kd=(float)PID_Param[PID_KD1]/100;
		Velocity_Kp=(float)PID_Param[PID_KP2]/100;
		Velocity_Ki=(float)PID_Param[PID_KI2]/100;
		Turn_Kp=(float)PID_Param[PID_KP3]/100;
		Turn_Ki=(float)PID_Param[PID_KI3]/100;
		Turn_Kd=(float)PID_Param[PID_KD3]/100;
	
		
		for(j=0;j<1;j++) //注意这里是多发几次，避免上位机接收失败或者传输干扰！
		{	
			ANO_DT_Send_Check(*(data_buf+2),sum); // 返回校验 发送成功
		}
		Param_Save();
		ANO_DT_Send_PID_ALL();  //重新上传所有参数
    }
    if(*(data_buf+2)==0X11)								//PID2
    {
         PID_Param[PID4_Way_Angle] = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
//         set_speed = 				 ( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
//        ANO_Param.PID_rol.kd = ( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
         PID_Param[PID5_set_speed] = ( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
		 PID_Param[PID5_ZHONGZHI] = ( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
         PID_Param[PID5_Turn_Target] = ( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//        ANO_Param.PID_yaw.kp 	= ( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//        ANO_Param.PID_yaw.ki 	= ( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//        ANO_Param.PID_yaw.kd 	= ( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		
		Way_Angle=(u8)PID_Param[PID4_Way_Angle];//修改运行模式
		set_speed=(int)PID_Param[PID5_set_speed];
		ZHONGZHI=(u16)PID_Param[PID5_ZHONGZHI];
		Turn_Target=(float)PID_Param[PID5_Turn_Target]/100;
		
		for(j=0;j<1;j++) //注意这里是多发几次，避免上位机接收失败或者传输干扰！
		{	
			ANO_DT_Send_Check(*(data_buf+2),sum); // 返回校验 发送成功
		}

		Param_Save();
		
		ANO_DT_Send_PID_ALL();  //重新上传所有参数
    }
    if(*(data_buf+2)==0X12)								//PID3
    {	
 /*       ANO_Param.PID_hs.kp  = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        ANO_Param.PID_hs.ki  = ( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        ANO_Param.PID_hs.kd  = ( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );*/
//			
//        pid_setup.groups.hc_height.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//        pid_setup.groups.hc_height.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//        pid_setup.groups.hc_height.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//			
//        pid_setup.groups.ctrl3.kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//        pid_setup.groups.ctrl3.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//        pid_setup.groups.ctrl3.kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		for(j=0;j<1;j++) //注意这里是多发几次，避免上位机接收失败或者传输干扰！
		{	
			ANO_DT_Send_Check(*(data_buf+2),sum); // 返回校验 发送成功
		}
		Param_Save();
    }
	if(*(data_buf+2)==0X13)								//PID4
	{
//		    pid_setup.groups.ctrl4.kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
//        pid_setup.groups.ctrl4.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
//        pid_setup.groups.ctrl4.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
			
//         pid_setup.groups.hc_height.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//         pid_setup.groups.hc_height.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//         pid_setup.groups.hc_height.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
// 			
//         pid_setup.groups.ctrl3.kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//         pid_setup.groups.ctrl3.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//         pid_setup.groups.ctrl3.kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		for(j=0;j<1;j++) //注意这里是多发几次，避免上位机接收失败或者传输干扰！
		{	
			ANO_DT_Send_Check(*(data_buf+2),sum); // 返回校验 发送成功
		}
		Param_Save();
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
				Param_Save();
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
				Param_Save();
	}
	
			
	
//////////////////////////////////////////////////////////////	
}
