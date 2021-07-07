#include "exti.h"
#include "SysTick.h"
#include "gps.h"
#include "stdio.h"
#include "time.h"
#include "usart.h"
#include "bsp_ads1256.h"

u8 unsync = 1;
/*******************************************************************************
* 函 数 名         : My_EXTI_Init
* 函数功能		   : 外部中断初始化
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void My_EXTI_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	
	
	
	//EXTI0 NVIC 配置
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//EXTI0中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	/*
	//EXTI2 NVIC 配置
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource2);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;//EXTI2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	EXTI_InitStructure.EXTI_Line=EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	*/
}

/*******************************************************************************
* 函 数 名         : EXTI0_IRQHandler
* 函数功能		   : 外部中断0函数，GPS秒脉冲产生中断
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void EXTI0_IRQHandler(void)
{
	OS_ERR err;
	extern u16 count;
	extern u16 excount;
	if(EXTI_GetITStatus(EXTI_Line0)==1)
	{
		GPS_Analysis(&gpsx,(u8*)USART3_RX_BUF);//ADD By Yao
		if (count != 500) 
		{
			printf("count: %d excount:%d\r\n", count, excount);
			printf("数据采集出错！\r\n");//Add By Yao
			printf("GPSTime:%d:%d:%d:%d\r\n\r\n",gpsx.utc.hour+8,gpsx.utc.min,gpsx.utc.sec,gpsx.utc.ms);//Add By Yao 用于检测GPS秒脉冲到来是否正确
		}
//		if(count==499||count==500||count==501)
//		{
//			printf("count:%d\r\n",count);
//		}

		count = 0;
		excount = 0;
		SYNC_0();
		//if (TIM_GetCounter(TIM5) > 2) printf("diff:%d\r\n",TIM_GetCounter(TIM5));	//TIM_GetCounter(TIM5)
		if (gpsx.utc.month <= 12 && gpsx.utc.year >= 2018 && gpsx.utc.date <= 31 && gpsx.utc.hour <= 24)
		{
			convertUTCtoBJ();
			OS_TaskResume((OS_TCB*)&GpsTaskTCB,&err);
		}
		delay_us(400);
		SYNC_1();
	}
	EXTI_ClearITPendingBit(EXTI_Line0);
}

/*******************************************************************************
* 函 数 名         : EXTI2_IRQHandler
* 函数功能		   : 外部中断2函数
* 输    入         : 无
* 输    出         : 无
******************************************************************************
void EXTI2_IRQHandler(void)
{
	OS_ERR err;
	if(EXTI_GetITStatus(EXTI_Line2)==1)
	{
		OS_TaskResume((OS_TCB*)&ReadTaskTCB,&err);
		//OSSemPost(&read,OS_OPT_POST_FIFO,&err); //释放full信号量
	}
	EXTI_ClearITPendingBit(EXTI_Line2);
}

*/
