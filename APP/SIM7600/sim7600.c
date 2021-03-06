#include "includes.h"
#include "sim7600.h"
#include "includes.h"
#include "usart.h"		
#include "SysTick.h"	 
#include "string.h" 
#include "math.h"
#include "stdio.h"
#include "oled.h"
#include "system.h"
#include "malloc.h"

//********************************************************************************
//无
//////////////////////////////////////////////////////////////////////////////////	
u8 SIM900_CSQ[3];
u8 dtbuf[50];
u8 Flag_Rec_Message=0;
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
	//ADD By Yao
void SIM7600_Init(void)
{
	//配置SIM7600硬件复位引脚
	GPIO_InitTypeDef GPIO_InitStructure;
	/* 打开GPIO时钟 */
	RCC_AHB1PeriphClockCmd(RCC_SIM_RST , ENABLE);
	/* 配置几个推完输出IO */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	/* 设为推挽输出口 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	/* IO口最大速度 */
	GPIO_InitStructure.GPIO_Pin = PIN_SIM_RST;
	GPIO_Init(PORT_SIM_RST, &GPIO_InitStructure);
	
	SIM_RST_1();
	delay_ms(100);
	SIM_RST_0();
}

	//ADD By Yao
void SIM7600_Reset(void)
{
	OS_ERR err; 
	u8 rres = 1;
	//复位
	SIM_RST_1();
	delay_ms(1000);
	SIM_RST_0();
	//GSM检测
	while(rres)
	{
		rres=GSM_Dect();
		if (rres != SIM_OK && rres != SIM_CREG_FAIL)
		{
			printf("res:%d RX: %s\r\n", rres, USART2_RX_BUF);
			continue;
		}
		rres=SIM7600_CONNECT_SERVER();
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
		printf("res:%d RX: %s\r\n", rres, USART2_RX_BUF);
	}
}

//usmart支持部分 
//将收到的AT指令应答数据返回给电脑串口
//mode:0,不清零USART2_RX_STA;
//     1,清零USART2_RX_STA;
void sim_at_response(u8 mode)
{
	if(USART2_RX_STA&0X8000)		//接收到一次数据了
	{ 
		USART2_RX_BUF[USART2_RX_STA&0X7FFF]=0;//添加结束符
		printf("%s",USART2_RX_BUF);	//发送到串口
		if(mode)USART2_RX_STA=0;		
	} 
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//ATK-SIM7600 各项测试(拨号测试、短信测试、GPRS测试)共用代码
//sim7600发送命令后,检测接收到的应答
//str:期待的应答结果
//返回值:0,没有得到期待的应答结果
//    其他,期待应答结果的位置(str的位置)
u8* sim7600_check_cmd(u8 *str)
{
	char *strx=0;
	if(USART2_RX_STA&0X8000)		//接收到一次数据了
	{ 
		USART2_RX_BUF[USART2_RX_STA&0X7FFF]=0;//添加结束符
		strx=strstr((const char*)USART2_RX_BUF,(const char*)str);
	} 
	return (u8*)strx;
}
//向sim7600发送命令
//cmd:发送的命令字符串(不需要添加回车了),当cmd<0XFF的时候,发送数字(比如发送0X1A),大于的时候发送字符串.
//ack:期待的应答结果,如果为空,则表示不需要等待应答
//waittime:等待时间(单位:10ms)
//返回值:0,发送成功(得到了期待的应答结果)
//       1,发送失败
u8 sim7600_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
{
	OS_ERR err; 
	extern OS_MUTEX SIM_Busy_mutex;
	u8 res=0; 
	USART2_RX_STA=0;
	USART2_RX_REC_ATCOMMAD=1;
	OSMutexPend(&SIM_Busy_mutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
	if((u32)cmd<=0XFF)
	{
//		while ( DMA_GetFlagStatus(DMA1_Stream6, DMA_FLAG_TCIF6) == RESET);	//等待通道7传输完成   
		while ( USART_GetITStatus(USART2, USART_IT_RXNE) != RESET);	
		USART2->DR=(u32)cmd;
	}
	else 
		u2_printf("%s\r\n",cmd);//发送命令
	if(ack&&waittime)		//需要等待应答
	{
		while(--waittime)	//等待倒计时
		{
			OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err);//延时
//			delay_ms(20);
			if(USART2_RX_STA&0X8000)//接收到期待的应答结果
			{
				if(sim7600_check_cmd(ack))break;//得到有效数据 
				USART2_RX_STA=0;
			} 
		}
		if(waittime==0)res=1; 
	}
	USART2_RX_STA=0;
	USART2_RX_REC_ATCOMMAD=0;
	OSMutexPost(&SIM_Busy_mutex,OS_OPT_POST_FIFO,&err); //等待sdmutex信号量
	return res;
} 

u8 sim7600_work_test(void)
{
	
	if(sim7600_send_cmd((u8 *)"AT",(u8 *)"OK",100))
	{
		if(sim7600_send_cmd((u8 *)"AT",(u8 *)"OK",100))
			return SIM_COMMUNTION_ERR;	//通信不上
	}		
	
	if(sim7600_send_cmd((u8 *)"AT+CPIN?",(u8 *)"READY",400))
		return SIM_CPIN_ERR;	//没有SIM卡
	

	if(sim7600_send_cmd((u8 *)"AT+CREG?",(u8 *)"0,1",400))
	{
		if(strstr((const char*)USART2_RX_BUF,"0,5")==NULL)
		{
			 if(!sim7600_send_cmd((u8 *)"AT+CSQ",(u8 *)"OK",200))	
			 {
					memcpy(SIM900_CSQ,USART2_RX_BUF+15,2);
			 }
			 return SIM_CREG_FAIL;	//等待附着到网络
		}
	}	
	
	return SIM_OK;
}

u8 GSM_Dec_Times=0;
u8 GSM_Dect(void)
{
	u8 res;
	res=sim7600_work_test();	
	switch(res)
	{
		case SIM_OK:
			printf("GSM模块自检成功\r\n");
#if OLED_ENABLE
			OLED_ShowString(0,3,(u8*)"     4G OK      ");
#endif
//检测超过次数单片机复位 Add By Yao
			GSM_Dec_Times++;
			if(GSM_Dec_Times>10)
			{
				printf("4G连接失败，准备重启...\r\n\r\n");
#if OLED_ENABLE
				OLED_ShowString(0,3,(u8*)" 4G failure,RST ");
#endif
				delay_ms(2000);
				soft_reset();
			}
			break;
		case SIM_COMMUNTION_ERR:
			printf("与GSM模块未通讯成功，请等待\r\n");
#if OLED_ENABLE
			OLED_ShowString(0,3,(u8*)"Connecting to 4G");
#endif
			break;
		case SIM_CPIN_ERR:
			printf("没检测到SIM卡\r\n");
#if OLED_ENABLE
			OLED_ShowString(0,3,(u8*)"  No SIM card   ");
#endif
			break;
		case SIM_CREG_FAIL:
			printf("注册网络中...\r\n");
			printf("当前信号值：%s\r\n",SIM900_CSQ);
#if OLED_ENABLE
			OLED_ShowString(0,3,(u8*)"  Networking... ");
#endif
		default:
			break;
	}
	return res;
}
//连接到FTP服务器
u8 SIM7600_CONNECT_SERVER()
{		
	extern u8 GSM_Dec_Times;
	if(sim7600_send_cmd((u8 *)"AT+NETOPEN?",(u8 *)"+NETOPEN: 1",200))//检查网络是否打开
	{
		if(sim7600_send_cmd((u8 *)"AT+CGSOCKCONT=1,\"IP\",\"CTNET\"",(u8 *)"OK",200))	return 1;
		if(sim7600_send_cmd((u8 *)"AT+CSOCKSETPN=1",(u8 *)"OK",600))	return 2;
		//if(sim7600_send_cmd((u8 *)"AT+CIPMODE=0",(u8 *)"OK",1000))	return 3;
		if(sim7600_send_cmd((u8 *)"AT+NETOPEN",(u8 *)"+NETOPEN: 0",1000))	   
		{
			sim7600_send_cmd((u8 *)"AT+NETCLOSE",(u8 *)"+NETCLOSE: 0",1000);  
			return 4;
		}
	}
	if(sim7600_send_cmd((u8 *)"AT+IPADDR",(u8 *)"OK",500))	return 5;
	if(sim7600_send_cmd((u8 *)"AT+CFTPSERV=\"111.231.145.217\"",(u8 *)"OK",500))	return 6;	  
	if(sim7600_send_cmd((u8 *)"AT+CFTPPORT=21",(u8 *)"OK",500))	return 7;	 
	if(sim7600_send_cmd((u8 *)"AT+CFTPMODE=1",(u8 *)"OK",500))	return 8;	 
	if(sim7600_send_cmd((u8 *)"AT+CFTPTYPE=I",(u8 *)"OK",500))	return 9;	 
	if(sim7600_send_cmd((u8 *)"AT+CFTPUN=\"ftpu\"",(u8 *)"OK",500))	return 10;	 
	if(sim7600_send_cmd((u8 *)"AT+CFTPPW=\"wsn123\"",(u8 *)"OK",500))	return 11;	
	GSM_Dec_Times=0;	//成功连接服务器，清除GSM检测次数，防止超出复位
	return 0;
}	
