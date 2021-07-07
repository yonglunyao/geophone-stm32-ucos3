#include "exti.h"
#include "SysTick.h"
#include "gps.h"
#include "stdio.h"
#include "time.h"
#include "usart.h"
#include "bsp_ads1256.h"
#include "oled.h"
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

}

/*******************************************************************************
* �� �� ��         : EXTI0_IRQHandler
* ��������		   : �ⲿ�ж�0������GPS����������жϣ��жϷ����������ᵼ�²ɼ�����
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
extern u8 SwitchFtptoFlash;
void EXTI0_IRQHandler(void)
{
	OS_ERR err;
//	u8 oled_buf[20];
	static u8 FirstTimeFlag=1;	//���ڽ�ִ��һ�λָ�����
	static u8 StartTimes=10;		//��������ϵͳ����ʱ���ݲɼ�����֪ͨ
	extern u16 count;
	extern u16 excount;
	extern OS_TCB ReadTaskTCB;
	extern OS_TCB FileTaskTCB;
	extern OS_TCB FtpTaskTCB;
	extern OS_TCB UsbTaskTCB;
	if(EXTI_GetITStatus(EXTI_Line0)==1)
	{
		GPS_Analysis(&gpsx,(u8*)USART3_RX_BUF);//ADD By Yao
		/*GPS��λ�ɹ���ָ��������񣬸ó����ִ��һ��*/
		if(FirstTimeFlag)
		{
			FirstTimeFlag=0;
			printf("GPS��λ�ɹ���\r\n\r\n");
#if OLED_ENABLE
			OLED_ShowString(0,3,(u8*)"    GPS OK      ");
#endif
			OS_TaskResume((OS_TCB*)&ReadTaskTCB,&err);	//ADD By Yao
			OS_TaskResume((OS_TCB*)&FileTaskTCB,&err);	//ADD By Yao
			if(!SwitchFtptoFlash)OS_TaskResume((OS_TCB*)&FtpTaskTCB,&err);		//ADD By Yao		
			if(SwitchFtptoFlash)OS_TaskResume((OS_TCB*)&UsbTaskTCB,&err);		//ADD By Yao				
		}		
		
		if(StartTimes) StartTimes--;	//Edit By Yao

		if ((!StartTimes) && count != 499 && count != 500) 
		{
			printf("count: %d excount:%d\r\n", count, excount);
			printf("���ݲɼ�����\r\n");//Add By Yao
			printf("GPSTime:%d:%d:%d:%d\r\n\r\n",gpsx.utc.hour+8,gpsx.utc.min,gpsx.utc.sec,gpsx.utc.ms);//Add By Yao ���ڼ��GPS�����嵽���Ƿ���ȷ
		}

		count = 0;
		excount = 0;
		SYNC_0(); //����ͬ��
		//if (TIM_GetCounter(TIM5) > 2) printf("diff:%d\r\n",TIM_GetCounter(TIM5));	//TIM_GetCounter(TIM5)
		TIM_SetCounter(TIM5,0);
		if (gpsx.utc.month <= 12 && gpsx.utc.year >= 2018 && gpsx.utc.date <= 31 && gpsx.utc.hour <= 24)
		{
			TIM_Cmd(TIM5,ENABLE);
			convertUTCtoBJ();
//			sprintf((char*)oled_buf,"%02d-%02d   %02d:%02d:%02d",month,day,hour,min,sec); //OLED��ʾʱ��������
//			OLED_ShowString(0,0,(u8*)oled_buf);
			OS_TaskResume((OS_TCB*)&GpsTaskTCB,&err);
		}
//		delay_us(400); //Edit By Yao
		SYNC_1();
	}
	EXTI_ClearITPendingBit(EXTI_Line0);
}

/*******************************************************************************
* �� �� ��         : Copy_EXTI_Init
* ��������		   : �����жϳ�ʼ��
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void Mode_EXTI_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	
	/* ��GPIOʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE , ENABLE);
	/* ���ü����������IO */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	/* ��Ϊ���� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	/* IO������ٶ� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	//EXTI0 NVIC ����
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource1);
	
	EXTI_InitStructure.EXTI_Line=EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;//EXTI1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

}

/*******************************************************************************
* �� �� ��         : EXTI1_IRQHandler
* ��������		   : �ⲿ�ж�1����
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void EXTI1_IRQHandler(void)
{
	OS_ERR err;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	extern u8 SwitchFtptoFlash;
	delay_ms(20);//ȥ����
	if(EXTI_GetITStatus(EXTI_Line1)==1)
	{
		//�л�ΪUģʽ
		SwitchFtptoFlash=!SwitchFtptoFlash;
		//���ж�
		EXTI_InitStructure.EXTI_Line=EXTI_Line1;
		EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
		EXTI_InitStructure.EXTI_LineCmd=DISABLE;
		EXTI_Init(&EXTI_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;//EXTI1�ж�ͨ��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;//��ռ���ȼ�
		NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;			//IRQͨ��ʹ��
		NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

	}
	EXTI_ClearITPendingBit(EXTI_Line1);
}


