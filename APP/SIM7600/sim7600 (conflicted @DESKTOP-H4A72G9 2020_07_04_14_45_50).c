
#include "sim7600.h"
#include "usart.h"		
#include "SysTick.h"	 
#include "string.h" 
#include "math.h"
#include "stdio.h"

//********************************************************************************
//��
//////////////////////////////////////////////////////////////////////////////////	
u8 SIM900_CSQ[3];
u8 dtbuf[50];
u8 Flag_Rec_Message=0;

//��ʼ����������ʼ����λ����
void sim7600_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��GPIOʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_SIM_RST,ENABLE);

	/* ���ü����������IO */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	/* ��Ϊ��������� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	/* IO������ٶ� */
	GPIO_InitStructure.GPIO_Pin = PIN_SIM_RST;
//	GPIO_InitStructure.GPIO_Pin = PIN_CS0 | PIN_CS1 | PIN_CS2  | PIN_RST;
	GPIO_Init(PORT_SIM_RST, &GPIO_InitStructure);
	//SIM7600��λ
	SIM_RST_1();
	delay_ms(100);
	SIM_RST_0();
}

void SIM7600_Reset(void)
{
	SIM_RST_1();
	delay_ms(1000);
	SIM_RST_0();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//usmart֧�ֲ��� 
//���յ���ATָ��Ӧ�����ݷ��ظ����Դ���
//mode:0,������USART2_RX_STA;
//     1,����USART2_RX_STA;
void sim_at_response(u8 mode)
{
	if(USART2_RX_STA&0X8000)		//���յ�һ��������
	{ 
		USART2_RX_BUF[USART2_RX_STA&0X7FFF]=0;//��ӽ�����
		printf("%s",USART2_RX_BUF);	//���͵�����
		if(mode)USART2_RX_STA=0;		
	} 
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//ATK-SIM7600 �������(���Ų��ԡ����Ų��ԡ�GPRS����)���ô���
//sim7600���������,�����յ���Ӧ��
//str:�ڴ���Ӧ����
//����ֵ:0,û�еõ��ڴ���Ӧ����
//    ����,�ڴ�Ӧ������λ��(str��λ��)
u8* sim7600_check_cmd(u8 *str)
{
	char *strx=0;
	if(USART2_RX_STA&0X8000)		//���յ�һ��������
	{ 
		USART2_RX_BUF[USART2_RX_STA&0X7FFF]=0;//��ӽ�����
		strx=strstr((const char*)USART2_RX_BUF,(const char*)str);
	} 
	return (u8*)strx;
}
//��sim7600��������
//cmd:���͵������ַ���(����Ҫ��ӻس���),��cmd<0XFF��ʱ��,��������(���緢��0X1A),���ڵ�ʱ�����ַ���.
//ack:�ڴ���Ӧ����,���Ϊ��,���ʾ����Ҫ�ȴ�Ӧ��
//waittime:�ȴ�ʱ��(��λ:10ms)
//����ֵ:0,���ͳɹ�(�õ����ڴ���Ӧ����)
//       1,����ʧ��
u8 sim7600_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
{
	u8 res=0; 
	USART2_RX_STA=0;
	USART2_RX_REC_ATCOMMAD=1;
	if((u32)cmd<=0XFF)
	{
		while ( DMA_GetFlagStatus(DMA1_Stream6, DMA_FLAG_TCIF6) == RESET);	//�ȴ�ͨ��7�������   
		USART2->DR=(u32)cmd;
	}
	else 
		u2_printf("%s\r\n",cmd);//��������
	if(ack&&waittime)		//��Ҫ�ȴ�Ӧ��
	{
		while(--waittime)	//�ȴ�����ʱ
		{
			delay_ms(10);
			if(USART2_RX_STA&0X8000)//���յ��ڴ���Ӧ����
			{
				if(sim7600_check_cmd(ack))break;//�õ���Ч���� 
				USART2_RX_STA=0;
			} 
		}
		if(waittime==0)res=1; 
	}
	USART2_RX_STA=0;
	USART2_RX_REC_ATCOMMAD=0;
	return res;
} 

u8 sim7600_work_test(void)
{
	
	if(sim7600_send_cmd((u8 *)"AT",(u8 *)"OK",100))
	{
		if(sim7600_send_cmd((u8 *)"AT",(u8 *)"OK",100))
			return SIM_COMMUNTION_ERR;	//ͨ�Ų���
	}		
	
	if(sim7600_send_cmd((u8 *)"AT+CPIN?",(u8 *)"READY",400))
		return SIM_CPIN_ERR;	//û��SIM��
	

	if(sim7600_send_cmd((u8 *)"AT+CREG?",(u8 *)"0,1",400))
	{
		if(strstr((const char*)USART2_RX_BUF,"0,5")==NULL)
		{
			 if(!sim7600_send_cmd((u8 *)"AT+CSQ",(u8 *)"OK",200))	
			 {
					memcpy(SIM900_CSQ,USART2_RX_BUF+15,2);
			 }
			 return SIM_CREG_FAIL;	//�ȴ����ŵ�����
		}
	}	
	
	return SIM_OK;
}

u8 GSM_Dect(void)
{
	u8 res;
	res=sim7600_work_test();	
	switch(res)
	{
		case SIM_OK:
			printf("GSMģ���Լ�ɹ�\r\n");
			break;
		case SIM_COMMUNTION_ERR:
			printf("��GSMģ��δͨѶ�ɹ�����ȴ�\r\n");
			break;
		case SIM_CPIN_ERR:
			printf("û��⵽SIM��\r\n");
			break;
		case SIM_CREG_FAIL:
			printf("ע�������С�����\r\n");
			printf("��ǰ�ź�ֵ��%s\r\n",SIM900_CSQ);
			break;
		default:
			break;
	}
	return res;
}
u8 SIM7600_CONNECT_SERVER()
{		
	if(sim7600_send_cmd((u8 *)"AT+CGSOCKCONT=1,\"IP\",\"3GNET\"",(u8 *)"OK",200))	return 1;//��ͨ�������CTNETΪ3GNET
//	if(sim7600_send_cmd((u8 *)"AT+CGSOCKCONT=1,\"IP\",,,0,0",(u8 *)"OK",200))	return 1;//��ͨ�������CTNETΪ3GNET
	
	if(sim7600_send_cmd((u8 *)"AT+CSOCKSETPN=1",(u8 *)"OK",600))	return 2;
	//if(sim7600_send_cmd((u8 *)"AT+CIPMODE=0",(u8 *)"OK",1000))	return 3;
	if(sim7600_send_cmd((u8 *)"AT+NETOPEN",(u8 *)"+NETOPEN: 0",1000))	   
	{
		sim7600_send_cmd((u8 *)"AT+NETCLOSE",(u8 *)"+NETCLOSE: 0",1000);  
		return 4;
	}
	if(sim7600_send_cmd((u8 *)"AT+IPADDR",(u8 *)"OK",500))	return 5;
	if(sim7600_send_cmd((u8 *)"AT+CFTPSERV=\"111.231.145.217\"",(u8 *)"OK",500))	return 6;	  
	if(sim7600_send_cmd((u8 *)"AT+CFTPPORT=21",(u8 *)"OK",500))	return 7;	 
	if(sim7600_send_cmd((u8 *)"AT+CFTPMODE=1",(u8 *)"OK",500))	return 8;	 
	if(sim7600_send_cmd((u8 *)"AT+CFTPTYPE=I",(u8 *)"OK",500))	return 9;	 
	if(sim7600_send_cmd((u8 *)"AT+CFTPUN=\"ftpu\"",(u8 *)"OK",500))	return 10;	 
	if(sim7600_send_cmd((u8 *)"AT+CFTPPW=\"wsn123\"",(u8 *)"OK",500))	return 11;	 
	return 0;
}	
