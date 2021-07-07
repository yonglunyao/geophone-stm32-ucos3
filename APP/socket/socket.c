#include "socket.h"
#include "sim7600.h"
#include "malloc.h"
#include "system.h"
#include "usart.h"
#include "delay.h"
/*
	���ܣ��ṹ��������ݴ��л�
	buf:���л����������
	data:��Ҫ���л��Ľṹ�壬����
	buflen:���buf����
*/
u8 Serialize_Report(u8* buf,Packet_connectData data,u8* buflen)
{
	u8 i;
	for(i=0;i<10;i++) buf[i]=data.Station_ID[i];
	for(i=10;i<20;i++) buf[i]=data.Authentication_Code[i-10];
	buf[21]=data.IsReset;
	buf[22]=data.WorkStatus;
	buf[23]=data.BatteryPower;
	for(i=24;i<28;i++) buf[i]=(data.Sent_Count)>>(8*(i-24))&0xFF;
	for(i=28;i<32;i++) buf[i]=(data.Wait_Count)>>(8*(i-28))&0xFF;
	buf[32]=data.time.year;
	buf[33]=data.time.month;
	buf[34]=data.time.day;
	buf[35]=data.time.hour;
	buf[36]=data.time.minute;
	buf[37]=data.time.second;
//	buf[37]=0x1A;
	*buflen=38;
	return 0;
}

/*
	���ܣ��ṹ��������ݷ����л�
	buf:�����л����뻺����
	data:�����л�����ṹ��
*/
u8 Deserialize_Receive(u8* buf,Packet_connectData* data)
{
	data->IsReset=buf[21];//ֻ�иó�Ա������ִ����䣬���಻��Ҫ���л�
	return 0;
}

/*
	���ܣ������ϱ��������л�������ݷ��͵�������
	buf:�����ͻ�����������
	buflen:buf���ȣ�����
*/
u8 transport_sendPacketBuffer(u8* buf, u8 buflen)
{
	RES_REPORT res;
	u16 j;
	if(sim7600_send_cmd((u8*)"AT+CIPSEND=0,",(u8*)">",500)==0)		//��������
	{ 
		for(j=0;j<buflen;j++)	USART2_TX_BUF[j]=buf[j];  //�����ݷ���USART2�ķ��ͻ�����׼������
		SendUsart2TXBUF(buflen);
//		delay_ms(5);
		if(sim7600_send_cmd((u8*)0X1A,(u8*)"+CIPSEND:",1000)==0) 
		{	
			res=REPORT_OK;
			printf("���ݷ��ͳɹ�!");
		}//CTRL+Z,�������ݷ���,����һ�δ���	
		else 
		{
			res=REPORT_FAILD;
			printf("���ݷ���ʧ��!");
		}
	}
	else 
	{
		sim7600_send_cmd((u8*)0X1B,0,0);	//ESC,ȡ������ 
		res=REPORT_ABORT;
	}
	return res;
}

//���ӵ�SOCKET������
void sim7600_LinkToSever(u8* ipaddr,u8* port)
{
	u8 *p;
	
	if(sim7600_send_cmd((u8 *)"AT+NETOPEN?",(u8 *)"+NETOPEN: 1",200)==0)//��������Ƿ��
		return;//�����Ѿ��򿪣�ֱ�ӷ���
	
	p=mymalloc(SRAMIN,50);//����50���ֽڵ��ڴ�
	
	sim7600_send_cmd((u8 *)"AT+CGSOCKCONT=1,\"IP\",\"CTNET\"",(u8 *)"OK",200);
	sim7600_send_cmd((u8 *)"AT+CSOCKSETPN=1",(u8 *)"OK",600);
	sim7600_send_cmd((u8 *)"AT+CIPMODE=0",(u8 *)"OK",1000);

	start:
	if(sim7600_send_cmd((u8 *)"AT+NETOPEN",(u8 *)"+NETOPEN: 0",200)==0)	
		printf("NETOPEN OK\r\n");
	else
	{
		printf("NETOPEN Faild\r\n");
		if(sim7600_send_cmd((u8 *)"AT+NETOPEN",(u8 *)"Network is already opened",1000)==0)	
			return;
		goto start;
	}
	
	
		myfree(SRAMIN,p); 
}
//�Ͽ�����������
void sim7600_close_sever(void)
{
  while(sim7600_send_cmd((u8*)"AT+CIPCLOSE=0",(u8*)"+CIPCLOSE: 0,0",500))
	{
		printf("�ȴ��Ͽ�������\r\n");	
		delay_ms(500);		
	}
		printf("�������Ͽ��ɹ�\r\n");  
}
