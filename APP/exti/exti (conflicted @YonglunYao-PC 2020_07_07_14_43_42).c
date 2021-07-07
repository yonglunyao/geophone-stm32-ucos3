#include "exti.h"
#include "SysTick.h"
#include "gps.h"
#include "stdio.h"
#include "time.h"
#include "usart.h"
#include "bsp_ads1256.h"

u8 unsync = 1;
/*******************************************************************************
* �� �� ��         : My_EXTI_Init
* ��������		   : �ⲿ�жϳ�ʼ��
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void My_EXTI_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	
	
	
	//EXTI0 NVIC ����
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//EXTI0�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	/*
	//EXTI2 NVIC ����
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource2);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;//EXTI2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	EXTI_InitStructure.EXTI_Line=EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	*/
}

/*******************************************************************************
* �� �� ��         : EXTI0_IRQHandler
* ��������		   : �ⲿ�ж�0������GPS����������ж�
* ��    ��         : ��
* ��    ��         : ��
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
			printf("���ݲɼ�����\r\n");//Add By Yao
			printf("GPSTime:%d:%d:%d:%d\r\n\r\n",gpsx.utc.hour+8,gpsx.utc.min,gpsx.utc.sec,gpsx.utc.ms);//Add By Yao ���ڼ��GPS�����嵽���Ƿ���ȷ
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
* �� �� ��         : EXTI2_IRQHandler
* ��������		   : �ⲿ�ж�2����
* ��    ��         : ��
* ��    ��         : ��
******************************************************************************
void EXTI2_IRQHandler(void)
{
	OS_ERR err;
	if(EXTI_GetITStatus(EXTI_Line2)==1)
	{
		OS_TaskResume((OS_TCB*)&ReadTaskTCB,&err);
		//OSSemPost(&read,OS_OPT_POST_FIFO,&err); //�ͷ�full�ź���
	}
	EXTI_ClearITPendingBit(EXTI_Line2);
}

*/
