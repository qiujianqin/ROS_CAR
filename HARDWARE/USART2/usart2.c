#include "usart2.h"
u8 Usart2_Receive;

/**************************************************************************
函数功能：串口2初始化
入口参数： bound:波特率
返回  值：无
**************************************************************************/
void uart2_init(u32 bound)
{  	 
	  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//使能UGPIOB时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//使能USART3时钟
	//USART2_TX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PB.10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);
   
  //USART3_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PB11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //Usart3 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure);     //初始化串口3
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART2, ENABLE);                    //使能串口3 
}

/**************************************************************************
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/
u8 TxBuffer_U2[256];
u8 TxCounter_U2=0;
u8 count_U2=0;
static u8 RxBuffer_U2[256];  //串口接收缓存
static u8 RxState_U2 = 0;

void USART2_IRQHandler(void)
{	
	//u8 i = 0;
	if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)//
    {
        USART_ReceiveData(USART2);
    }
		
	//发送中断
	if((USART2->SR & (1<<7))&&(USART2->CR1 & USART_CR1_TXEIE))//if(USART_GetITStatus(USART1,USART_IT_TXE)!=RESET)
	{
		USART2->DR = TxBuffer_U2[TxCounter_U2++]; //写DR清除中断标志          
		if(TxCounter_U2 == count_U2)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE中断
			//USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
		}
	}
	//接收中断 (接收寄存器非空) 
	if(USART2->SR & (1<<5))//if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)    
	{
		u8 com_data = USART2->DR;
		static u8 _data_len = 0,_data_cnt = 0;
		
		

		
		///////////////////////////*上位机部分*/////////////////////////////
		if(RxState_U2==0&&com_data==0xAA)      //判断帧头AAAF
		{
			RxState_U2=1;
			RxBuffer_U2[0]=com_data;
		}
		else if(RxState_U2==1&&com_data==0xAF)
		{
			RxState_U2=2;
			RxBuffer_U2[1]=com_data;
		}
		else if(RxState_U2==2&&com_data<=0XF1) //功能字
		{
			RxState_U2=3;
			RxBuffer_U2[2]=com_data;
		}
		else if(RxState_U2==3&&com_data<255) // 长度LEN
		{
			RxState_U2 = 4;
			RxBuffer_U2[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(RxState_U2==4&&_data_len>0) //数据
		{
			_data_len--;
			RxBuffer_U2[4+_data_cnt++]=com_data;
			if(_data_len==0)
				RxState_U2 = 5;
		}
		else if(RxState_U2==5)
		{
			RxState_U2 = 0;
			RxBuffer_U2[4+_data_cnt]=com_data;
//			ANO_NRF_TxPacket(RxBuffer,5+_data_cnt);   //传递到NRF发送
			ANO_DT_Data_Receive_Anl(RxBuffer_U2,5+_data_cnt);   //传递到数据分析
//				for(i=0;i<5+_data_cnt;i++)  //串口打印收到的数据，调试时用，可屏蔽
//				{
//					while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕    //这是发送为16进制的数显示
//					USART_SendData(USART1,RxBuffer[i]); 
//				}
			
		}
		else
			RxState_U2 = 0;
		
		//////////////////*遥控器*/////////////////
//		if(com_data>10)  //默认使用
//		{			
//			if(com_data==0x5A)	Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////刹车
//			else if(com_data==0x41)	Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0,Movement=50,Turn_Target=0;//////////////前
//			else if(com_data==0x45)	Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0,Movement=-50,Turn_Target=0;//////////////后
//			else if(com_data==0x42||com_data==0x43||com_data==0x44)	
//														Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1,Turn_Target=-50,Movement=0;  //左
//			else if(com_data==0x46||com_data==0x47||com_data==0x48)	    //右
//														Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0,Turn_Target=50,Movement=0;
//			else Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0,Turn_Target=0,Movement=0;//////////////刹车
//		}
		
	}
} 



