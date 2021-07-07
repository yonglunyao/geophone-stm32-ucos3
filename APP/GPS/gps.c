#include "gps.h" 
#include "systick.h" 							   
#include "usart.h" 								   
#include "stdio.h"	 
#include "stdarg.h"	 
#include "string.h"	 
#include "math.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F103������
//ATK-S1216F8 GPSģ����������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2015/04/11
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved							  
////////////////////////////////////////////////////////////////////////////////// 	   
//
nmea_msg gpsx; //gps��ȡ����
u16 year=0;
u8 month=0;
u8 day=0;
u8 hour=0;
u8 min=0;
u8 sec=0;
u16 ms=0;

const u32 BAUD_id[9]={4800,9600,19200,38400,57600,115200,230400,460800,921600};//ģ��֧�ֲ���������

#define MINUTE 60
#define HOUR (60*MINUTE)
#define DAY (24*HOUR)
#define YEAR (365*DAY)

#define UTC_BASE_YEAR 1970
#define MONTH_PER_YEAR 12 
#define DAY_PER_YEAR 365 

static u32 mon[12] = {
	0,
	DAY*(31),
	DAY*(31 + 29),
	DAY*(31 + 29 + 31),
	DAY*(31 + 29 + 31 + 30),
	DAY*(31 + 29 + 31 + 30 + 31),
	DAY*(31 + 29 + 31 + 30 + 31 + 30),
	DAY*(31 + 29 + 31 + 30 + 31 + 30 + 31),
	DAY*(31 + 29 + 31 + 30 + 31 + 30 + 31 + 31),
	DAY*(31 + 29 + 31 + 30 + 31 + 30 + 31 + 31 + 30),
	DAY*(31 + 29 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31),
	DAY*(31 + 29 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30)
};

const u8 g_day_per_mon[MONTH_PER_YEAR] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

u32 mktime(u16 year, u8 month, u8 day, u8 hour, u8 min, u8 sec)
{
	u32 res;
	u16 m_year;

	m_year = year - UTC_BASE_YEAR;
	res = YEAR*m_year + DAY*((m_year + 1) / 4);
	res += mon[month - 1];
	if (month > 2 && ((m_year + 2) % 4))
		res -= DAY;
	res += DAY*(day - 1);
	res += HOUR*hour;
	res += MINUTE*min;
	res += sec;
	return res;
}

u8 applib_dt_is_leap_year(u16 m_year)
{
	if ((m_year % 400) == 0)
		return 1;
	else if ((m_year % 100) == 0)
		return 0;
	else if ((m_year % 4) == 0)
		return 1;
	else 
		return 0;
}

u8 applib_dt_last_day_of_mon(u8 m_month, u16 m_year)
{
	if ((m_month == 0) || (m_month > 12))
		return g_day_per_mon[1] + applib_dt_is_leap_year(m_year);
	if (m_month != 2)
		return g_day_per_mon[m_month - 1];
	else 
		return g_day_per_mon[1] + applib_dt_is_leap_year(m_year);
}

void utc_sec_2_mytime(u32 utc_sec)
{
	u32 m_sec, m_day;
	u16 y;
	u8 m;
	u16 d;
	m_sec = utc_sec % DAY;
	hour = m_sec / HOUR;
	m_sec %= HOUR;
	min = m_sec / MINUTE;
	sec = m_sec % MINUTE;
	m_day = utc_sec / DAY;
	for (y = UTC_BASE_YEAR; m_day > 0; y++)
	{
		d = (DAY_PER_YEAR + applib_dt_is_leap_year(y));
		if (m_day >= d)
			m_day -= d;
		else
			break;
	} 
	year = y;
	for (m = 1; m < MONTH_PER_YEAR; m++)
	{
		d = applib_dt_last_day_of_mon(m, y);
		if (m_day >= d)
			m_day -= d;
		else
			break;
	}
	month = m;
	day = (u8)(m_day + 1);
}

void convertUTCtoBJ()
{
	if (gpsx.utc.date == 0 || gpsx.utc.date > 31)
		return;
	utc_sec_2_mytime(mktime(gpsx.utc.year, gpsx.utc.month, gpsx.utc.date, gpsx.utc.hour, gpsx.utc.min, gpsx.utc.sec) + 28801);
}
//��buf����õ���cx���������ڵ�λ��
//����ֵ:0~0XFE,����������λ�õ�ƫ��.
//       0XFF,�������ڵ�cx������							  
u8 NMEA_Comma_Pos(u8 *buf,u8 cx)
{	 		    
	u8 *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//����'*'���߷Ƿ��ַ�,�򲻴��ڵ�cx������
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}
//m^n����
//����ֵ:m^n�η�.
u32 NMEA_Pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}
//strת��Ϊ����,��','����'*'����
//buf:���ִ洢��
//dx:С����λ��,���ظ����ú���
//����ֵ:ת�������ֵ
u32 NMEA_Str2num(u8 *buf,u8*dx)
{
	u8 *p=buf;
	u32 ires=0,fres=0;
	u8 ilen=0,flen=0,i;
	u8 mask=0;
	u32 res;
	while(1) //�õ�������С���ĳ���
	{
		if(*p=='-'){mask|=0X02;p++;}//�Ǹ���
		if(*p==','||(*p=='*'))break;//����������
		if(*p=='.'){mask|=0X01;p++;}//����С������
		else if(*p>'9'||(*p<'0'))	//�зǷ��ַ�
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//ȥ������
	for(i=0;i<ilen;i++)	//�õ�������������
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//���ȡ5λС��
	*dx=flen;	 		//С����λ��
	for(i=0;i<flen;i++)	//�õ�С����������
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}	  							 


//����GNGGA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GNGGA_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;    
	p1=(u8*)strstr((const char *)buf,"$GNGGA");
	posx=NMEA_Comma_Pos(p1,9);								//�õ����θ߶�
	if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx);  
}

//����GNRMC��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GNRMC_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;     
	u32 temp;
	u32 temp2;
	float rs;  
	p1=(u8*)strstr((const char *)buf,"$GNRMC");//"$GNRMC",������&��GNRMC�ֿ������,��ֻ�ж�GPRMC.
	posx=NMEA_Comma_Pos(p1,1);								//�õ�UTCʱ��
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//�õ�UTCʱ��,ȥ��ms
		temp2=NMEA_Str2num(p1+posx,&dx)%NMEA_Pow(10,dx);	 	//�õ�UTCʱ��,ȥ��ms
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;
		gpsx->utc.ms=temp2;
	}	
	posx=NMEA_Comma_Pos(p1,3);								//�õ�γ��
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//�õ���
		rs=temp%NMEA_Pow(10,dx+2);				//�õ�'		 
		gpsx->latitude=gpsx->latitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//ת��Ϊ�� 
	}
	posx=NMEA_Comma_Pos(p1,4);								//��γ���Ǳ�γ 
	if(posx!=0XFF)gpsx->nshemi=*(p1+posx);					 
 	posx=NMEA_Comma_Pos(p1,5);								//�õ�����
	if(posx!=0XFF)
	{												  
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//�õ���
		rs=temp%NMEA_Pow(10,dx+2);				//�õ�'		 
		gpsx->longitude=gpsx->longitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//ת��Ϊ�� 
	}
	posx=NMEA_Comma_Pos(p1,6);								//������������
	if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);		 
	posx=NMEA_Comma_Pos(p1,9);								//�õ�UTC����
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 				//�õ�UTC����
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;	 	 
	} 
}

//��ȡNMEA-0183��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void GPS_Analysis(nmea_msg *gpsx,u8 *buf)
{
	NMEA_GNGGA_Analysis(gpsx,buf);	//GNGGA���� 	
	NMEA_GNRMC_Analysis(gpsx,buf);	//GPNMC����
	//LBlocate(gpsx);
}
/*
///////////////////////////////////////////UBLOX ���ô���/////////////////////////////////////
////���CFG����ִ�����
////����ֵ:0,ACK�ɹ�
////       1,���ճ�ʱ����
////       2,û���ҵ�ͬ���ַ�
////       3,���յ�NACKӦ��
u8 SkyTra_Cfg_Ack_Check(void)
{			 
	u16 len=0,i;
	u8 rval=0;
	while((USART3_RX_STA&0X8000)==0 && len<100)//�ȴ����յ�Ӧ��   
	{
		len++;
		delay_ms(5);
	}		 
	if(len<100)   	//��ʱ����.
	{
		len=USART3_RX_STA&0X7FFF;	//�˴ν��յ������ݳ��� 
		for(i=0;i<len;i++)
		{
			if(USART3_RX_BUF[i]==0X83)break;
			else if(USART3_RX_BUF[i]==0X84)
			{
				rval=3;
				break;
			}
		}
		if(i==len)rval=2;						//û���ҵ�ͬ���ַ�
	}else rval=1;								//���ճ�ʱ����
    USART3_RX_STA=0;							//�������
	return rval;  
}





//����SkyTra_GPS/����ģ�鲨����
//baud_id:0~8����Ӧ������,4800/9600/19200/38400/57600/115200/230400/460800/921600	  
//����ֵ:0,ִ�гɹ�;����,ִ��ʧ��(���ﲻ�᷵��0��)
u8 SkyTra_Cfg_Prt(u32 baud_id)
{
	SkyTra_baudrate *cfg_prt=(SkyTra_baudrate *)USART3_TX_BUF;
	cfg_prt->sos=0XA1A0;		//��������(С��ģʽ)
	cfg_prt->PL=0X0400;			//��Ч���ݳ���(С��ģʽ)
	cfg_prt->id=0X05;		    //���ò����ʵ�ID 
	cfg_prt->com_port=0X00;			//��������1
	cfg_prt->Baud_id=baud_id;	 	////�����ʶ�Ӧ���
	cfg_prt->Attributes=1; 		  //���浽SRAM&FLASH
	cfg_prt->CS=cfg_prt->id^cfg_prt->com_port^cfg_prt->Baud_id^cfg_prt->Attributes;
	cfg_prt->end=0X0A0D;        //���ͽ�����(С��ģʽ)
	SkyTra_Send_Date((u8*)cfg_prt,sizeof(SkyTra_baudrate));//�������ݸ�SkyTra   
	delay_ms(200);				//�ȴ�������� 
	USART3_Init(BAUD_id[baud_id]);	//���³�ʼ������3  
	return SkyTra_Cfg_Ack_Check();//���ﲻ�ᷴ��0,��ΪUBLOX��������Ӧ���ڴ������³�ʼ����ʱ���Ѿ���������.
} 
//����SkyTra_GPS/����ģ���ʱ��������
//width:������1~100000(us)
//����ֵ:0,���ͳɹ�;����,����ʧ��.
u8 SkyTra_Cfg_Tp(u32 width)
{
	u32 temp=width;
	SkyTra_pps_width *cfg_tp=(SkyTra_pps_width *)USART3_TX_BUF;
	temp=(width>>24)|((width>>8)&0X0000FF00)|((width<<8)&0X00FF0000)|((width<<24)&0XFF000000);//С��ģʽ
	cfg_tp->sos=0XA1A0;		    //cfg header(С��ģʽ)
	cfg_tp->PL=0X0700;        //��Ч���ݳ���(С��ģʽ)
	cfg_tp->id=0X65	;			    //cfg tp id
	cfg_tp->Sub_ID=0X01;			//����������Ϊ20���ֽ�.
	cfg_tp->width=temp;		  //������,us
	cfg_tp->Attributes=0X01;  //���浽SRAM&FLASH	
	cfg_tp->CS=cfg_tp->id^cfg_tp->Sub_ID^(cfg_tp->width>>24)^(cfg_tp->width>>16)&0XFF^(cfg_tp->width>>8)&0XFF^cfg_tp->width&0XFF^cfg_tp->Attributes;    	//�û���ʱΪ0ns
	cfg_tp->end=0X0A0D;       //���ͽ�����(С��ģʽ)
	SkyTra_Send_Date((u8*)cfg_tp,sizeof(SkyTra_pps_width));//�������ݸ�NEO-6M  
	return SkyTra_Cfg_Ack_Check();
}
//����SkyTraF8-BD�ĸ�������    
//Frep:��ȡֵ��Χ:1,2,4,5,8,10,20,25,40,50������ʱ��������λΪHz������ܴ���50Hz
//����ֵ:0,���ͳɹ�;����,����ʧ��.
u8 SkyTra_Cfg_Rate(u8 Frep)
{
	SkyTra_PosRate *cfg_rate=(SkyTra_PosRate *)USART3_TX_BUF;
 	cfg_rate->sos=0XA1A0;	    //cfg header(С��ģʽ)
	cfg_rate->PL=0X0300;			//��Ч���ݳ���(С��ģʽ)
	cfg_rate->id=0X0E;	      //cfg rate id
	cfg_rate->rate=Frep;	 	  //��������
	cfg_rate->Attributes=0X01;	   	//���浽SRAM&FLASH	.
	cfg_rate->CS=cfg_rate->id^cfg_rate->rate^cfg_rate->Attributes;//������,us
	cfg_rate->end=0X0A0D;       //���ͽ�����(С��ģʽ)
	SkyTra_Send_Date((u8*)cfg_rate,sizeof(SkyTra_PosRate));//�������ݸ�NEO-6M 
	return SkyTra_Cfg_Ack_Check();
}
//����һ�����ݸ�Ublox NEO-6M������ͨ������3����
//dbuf�����ݻ����׵�ַ
//len��Ҫ���͵��ֽ���
void SkyTra_Send_Date(u8* dbuf,u16 len)
{
	u16 j;
	for(j=0;j<len;j++)//ѭ����������
	{
		while((USART3->SR&0X40)==0);//ѭ������,ֱ���������   
		USART3->DR=dbuf[j];  
	}	
}
*/

//wgs84 to bj54
void LBlocate(nmea_msg *gpsx)
{
	double LL = gpsx->longitude / 100000.0;
	double BB = gpsx->latitude / 100000.0;
	double a=6378245.0; 
	double b=6356863.0187730473;  
	double mid = 117;//���뾭��
	double x,y;
	double e1=sqrt(pow(a,2.0)-pow(b,2.0))/a;
	double e2=sqrt(pow(a,2.0)-pow(b,2.0))/b;
	double l=LL-mid;  //lΪ����
	double B=BB*3.141592683/180.0;
	double L=LL*3.141592683/180.0;
	l=l*3.141592683/180.0;
	 //X0Ϊ��l=0ʱ���ӳ������������߻��� 
    //���������߻���X��ϵ��
	double A0=1.0+3.0/4*pow(e1,2.0)+45.0/64*pow(e1,4.0)+350.0/512*pow(e1,6.0)+11025.0/16384*pow(e1,8.0);
	double A2=-(3.0/4*pow(e1,2.0)+60.0/64*pow(e1,4.0)+525.0/512*pow(e1,6.0)+17640.0/16384*pow(e1,8.0))/2.0;
	double A4=(15.0/64*pow(e1,4.0)+210.0/512*pow(e1,6.0)+8820.0/16384*pow(e1,8.0))/4;
	double A6=-(35.0/512*pow(e1,6.0)+2520.0/16384*pow(e1,8.0))/6;
	double A8=(315.0/16384.0*pow(e1,8.0))/8;
    //���������߻���X
	double X0=a*(1.0-pow(e1,2.0))*(A0*B+A2*sin(2*B)+A4*sin(4*B)+A6*sin(6*B)+A8*sin(8*B));
	double t=tan(B);
	double anke=e2*cos(B);
	double N=a/sqrt(1.0-pow(e1,2.0)*pow(sin(B),2.0));  //NΪͶӰ���î��Ȧ���ʰ뾶
	//�������
	x=X0+1.0/2*N*t*pow(cos(B),2.0)*pow(l,2.0)+1.0/24*N*t*(5.0-pow(t,2.0)+9.0*pow(anke,2.0)+4.0*pow(anke,4.0))*pow(cos(B),4.0)
		*pow(l,4.0)+1.0/720*N*t*(61.0-58.0*pow(t,2.0)+pow(t,4.0)+270*pow(anke,2.0)-330.0*pow(anke,2.0)*pow(t,2.0))*pow(cos(B),6.0)*pow(l,6.0);
	y=N*cos(B)*l+1.0/6*N*(1-pow(t,2.0)+pow(anke,2.0))*pow(cos(B),3.0)*pow(l,3.0)
        +1.0/120*N*(5.0-18.0*pow(t,2.0)+pow(t,4.0)+14.0*pow(anke,2.0)-58*pow(anke,2)
		*pow(t,2))*pow(cos(B),5.0)*pow(l,5.0);
	y+=500000.0;  //add 500km
	gpsx->x = x * 100000;
	gpsx->y = y * 100000;
}




