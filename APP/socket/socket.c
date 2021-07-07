#include "socket.h"
#include "sim7600.h"
#include "malloc.h"
#include "system.h"
#include "usart.h"
#include "delay.h"
/*
	功能：结构体变量数据串行化
	buf:串行化输出缓冲区
	data:需要串行化的结构体，输入
	buflen:输出buf长度
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
	功能：结构体变量数据反串行化
	buf:反串行化输入缓冲区
	data:反串行化输出结构体
*/
u8 Deserialize_Receive(u8* buf,Packet_connectData* data)
{
	data->IsReset=buf[21];//只有该成员变量是执行语句，其余不需要串行化
	return 0;
}

/*
	功能：数据上报：将串行化后的数据发送到服务器
	buf:待发送缓冲区，输入
	buflen:buf长度，输入
*/
u8 transport_sendPacketBuffer(u8* buf, u8 buflen)
{
	RES_REPORT res;
	u16 j;
	if(sim7600_send_cmd((u8*)"AT+CIPSEND=0,",(u8*)">",500)==0)		//发送数据
	{ 
		for(j=0;j<buflen;j++)	USART2_TX_BUF[j]=buf[j];  //将数据放在USART2的发送缓冲区准备发送
		SendUsart2TXBUF(buflen);
//		delay_ms(5);
		if(sim7600_send_cmd((u8*)0X1A,(u8*)"+CIPSEND:",1000)==0) 
		{	
			res=REPORT_OK;
			printf("数据发送成功!");
		}//CTRL+Z,结束数据发送,启动一次传输	
		else 
		{
			res=REPORT_FAILD;
			printf("数据发送失败!");
		}
	}
	else 
	{
		sim7600_send_cmd((u8*)0X1B,0,0);	//ESC,取消发送 
		res=REPORT_ABORT;
	}
	return res;
}

//连接到SOCKET服务器
void sim7600_LinkToSever(u8* ipaddr,u8* port)
{
	u8 *p;
	
	if(sim7600_send_cmd((u8 *)"AT+NETOPEN?",(u8 *)"+NETOPEN: 1",200)==0)//检查网络是否打开
		return;//网络已经打开，直接返回
	
	p=mymalloc(SRAMIN,50);//申请50个字节的内存
	
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
//断开服务器连接
void sim7600_close_sever(void)
{
  while(sim7600_send_cmd((u8*)"AT+CIPCLOSE=0",(u8*)"+CIPCLOSE: 0,0",500))
	{
		printf("等待断开服务器\r\n");	
		delay_ms(500);		
	}
		printf("服务器断开成功\r\n");  
}
