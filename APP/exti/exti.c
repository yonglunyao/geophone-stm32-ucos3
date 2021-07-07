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

}

/*******************************************************************************
* 函 数 名         : EXTI0_IRQHandler
* 函数功能		   : 外部中断0函数，GPS秒脉冲产生中断（中断服务函数过长会导致采集错误）
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
extern u8 SwitchFtptoFlash;
void EXTI0_IRQHandler(void)
{
	OS_ERR err;
//	u8 oled_buf[20];
	static u8 FirstTimeFlag=1;	//用于仅执行一次恢复任务
	static u8 StartTimes=10;		//用于屏蔽系统启动时数据采集错误通知
	extern u16 count;
	extern u16 excount;
	extern OS_TCB ReadTaskTCB;
	extern OS_TCB FileTaskTCB;
	extern OS_TCB FtpTaskTCB;
	extern OS_TCB UsbTaskTCB;
	if(EXTI_GetITStatus(EXTI_Line0)==1)
	{
		GPS_Analysis(&gpsx,(u8*)USART3_RX_BUF);//ADD By Yao
		/*GPS定位成功后恢复三个任务，该程序仅执行一次*/
		if(FirstTimeFlag)
		{
			FirstTimeFlag=0;
			printf("GPS定位成功！\r\n\r\n");
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
			printf("数据采集出错！\r\n");//Add By Yao
			printf("GPSTime:%d:%d:%d:%d\r\n\r\n",gpsx.utc.hour+8,gpsx.utc.min,gpsx.utc.sec,gpsx.utc.ms);//Add By Yao 用于检测GPS秒脉冲到来是否正确
		}

		count = 0;
		excount = 0;
		SYNC_0(); //掉电同步
		//if (TIM_GetCounter(TIM5) > 2) printf("diff:%d\r\n",TIM_GetCounter(TIM5));	//TIM_GetCounter(TIM5)
		TIM_SetCounter(TIM5,0);
		if (gpsx.utc.month <= 12 && gpsx.utc.year >= 2018 && gpsx.utc.date <= 31 && gpsx.utc.hour <= 24)
		{
			TIM_Cmd(TIM5,ENABLE);
			convertUTCtoBJ();
//			sprintf((char*)oled_buf,"%02d-%02d   %02d:%02d:%02d",month,day,hour,min,sec); //OLED显示时间与日期
//			OLED_ShowString(0,0,(u8*)oled_buf);
			OS_TaskResume((OS_TCB*)&GpsTaskTCB,&err);
		}
//		delay_us(400); //Edit By Yao
		SYNC_1();
	}
	EXTI_ClearITPendingBit(EXTI_Line0);
}

/*******************************************************************************
* 函 数 名         : Copy_EXTI_Init
* 函数功能		   : 按键中断初始化
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void Mode_EXTI_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	
	/* 打开GPIO时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE , ENABLE);
	/* 配置几个推完输出IO */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	/* 设为输入 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	/* IO口最大速度 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	//EXTI0 NVIC 配置
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource1);
	
	EXTI_InitStructure.EXTI_Line=EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;//EXTI1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

}

/*******************************************************************************
* 函 数 名         : EXTI1_IRQHandler
* 函数功能		   : 外部中断1函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void EXTI1_IRQHandler(void)
{
	OS_ERR err;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	extern u8 SwitchFtptoFlash;
	delay_ms(20);//去抖动
	if(EXTI_GetITStatus(EXTI_Line1)==1)
	{
		//切换为U模式
		SwitchFtptoFlash=!SwitchFtptoFlash;
		//关中断
		EXTI_InitStructure.EXTI_Line=EXTI_Line1;
		EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
		EXTI_InitStructure.EXTI_LineCmd=DISABLE;
		EXTI_Init(&EXTI_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;//EXTI1中断通道
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;//抢占优先级
		NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;			//IRQ通道使能
		NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

	}
	EXTI_ClearITPendingBit(EXTI_Line1);
}


