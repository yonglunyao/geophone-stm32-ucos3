#include "http.h"
#include "system.h"
#include "includes.h"
#include "malloc.h"
#include "socket.h"
#include "string.h"
#include "usart.h"
#include "sim7600.h"
#include "delay.h"

/*
	���ܣ��ṹ��������ݴ��л�
	buf:���л����������
	data:��Ҫ���л��Ľṹ�壬����
	buflen:���buf����
*/
void Serialize_HttpReport(u8* buf,Packet_connectData data,u8* buflen)
{
	u8 i;
	u8* tmp[32];
	sprintf((char*)buf,"%s ",data.Station_ID);
//	switch((u8)data.WorkStatus)
//	{
//		case 0:
//			strcat((char*)buf,"WorkStatus:OK              ");
//			break;
//		case 1:
//			strcat((char*)buf,"WorkStatus:GPS Unlocated   ");
//			break;
//		case 2:
//			strcat((char*)buf,"WorkStatus:SD Card Error   ");
//			break;
//		case 3:case 4:case 5:
//			strcat((char*)buf,"WorkStatus:ADS1256 Error   ");
//			break;
//		case 6:
//			strcat((char*)buf,"WorkStatus:USB Error       ");
//			break;
//				
//	}
	sprintf((char*)tmp,"%03d ",data.BatteryPower);
	strcat((char*)buf,(char*)tmp);
	sprintf((char*)tmp,"%06d ",data.Sent_Count);
	strcat((char*)buf,(char*)tmp);
	sprintf((char*)tmp,"%06d ",data.Wait_Count);
	strcat((char*)buf,(char*)tmp);
	sprintf((char*)tmp,"20%02d-%02d-%02d %02d:%02d:%02d ",
					data.time.year,data.time.month,data.time.day,data.time.hour,data.time.minute,data.time.second);
	strcat((char*)buf,(char*)tmp);
	sprintf((char*)tmp,"%s",data.Authentication_Code);
	strcat((char*)buf,(char*)tmp);
	*buflen=strlen((char*)buf);
}

u8 transport_HttpSendPacketBuffer(u8* server_addr,u8* port,u8* buf,u8 buflen)
{
	OS_ERR err; 
	u8 res;
	static u8 times=0;//��¼HTTP��������
	extern OS_MUTEX SIM_Busy_mutex;
	u8* request=mymalloc(SRAMIN,64);
//	u8* sendbuf=mymalloc(SRAMIN,512);
	u8 sendbuf[512];
	u8* tmpbuf=mymalloc(SRAMIN,50);
	u16 sendbuflen=0;
	sprintf((char*)request,"AT+CHTTPACT=\"%s\",%s\r\n",server_addr,port);//REQUEST
	while(sim7600_send_cmd(request,(u8*)"REQUEST",500))
	{
		times++;
		printf("����HTTP������ʧ��\r\n"); 
		OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err);//��ʱ
		if(times>0) //ֻ���һ��
		{
			times=0;
			return 2;
		}
	}
	times=0;
//	printf("����HTTP�������ɹ�\r\n"); 
	memset(request,0,64);
	memset(sendbuf,0,512);
	memset(tmpbuf,0,50);
	sprintf((char*)sendbuf,"POST / HTTP/1.1\r\n");
	sprintf((char*)tmpbuf,"Host: %s\r\n",server_addr);
	printf("%s",tmpbuf);
	strcat((char*)sendbuf,(char*)tmpbuf);
	strcat((char*)sendbuf,(char*)"User-Agent: MY WEB AGENT\r\n");
	strcat((char*)sendbuf,(char*)"Accept:\r\n");
	strcat((char*)sendbuf,(char*)"Content-Type: text/html;Charset: utf-8\r\n");
	strcat((char*)sendbuf,(char*)"Cache-Control: no-cache\r\n");
	strcat((char*)sendbuf,(char*)"Accept-Charset: utf-8, us-ascii\r\n");
	strcat((char*)sendbuf,(char*)"Pragma: no-cache\r\n");
	strcat((char*)sendbuf,(char*)"Content-Length: ");
	sprintf((char*)tmpbuf,"%d\r\n\r\n",buflen);
	strcat((char*)sendbuf,(char*)tmpbuf);
	strcat((char*)sendbuf,(char*)buf);
	sendbuflen=strlen((const char*)sendbuf)+1;
//	printf("%s\r\n",sendbuf);
	OSMutexPend(&SIM_Busy_mutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
//	OSSchedLock(&err);
	for(int j=0;j<sendbuflen;j++)	USART2_TX_BUF[j]=sendbuf[j];  //�����ݷ���USART2�ķ��ͻ�����׼������
	SendUsart2TXBUF(sendbuflen);
	if(sim7600_send_cmd((u8*)0X1A,(u8*)"+CHTTPACT: 0",1000)==0) 
	{	
//		printf("���ݷ��ͳɹ�!\r\n");
		res=0;
	}//CTRL+Z,�������ݷ���,����һ�δ���	
	else 
	{
//	printf("���ݷ���ʧ��!\r\n");
		res=1;
	}
///	printf("RX2:%s\r\n",USART2_RX_BUF);
	OSMutexPost(&SIM_Busy_mutex,OS_OPT_POST_FIFO,&err); //�ȴ�sdmutex�ź���
//	OSSchedUnlock(&err);
	myfree(SRAMIN,request);
//	myfree(SRAMIN,sendbuf);
	myfree(SRAMIN,tmpbuf);
	return res;
}
