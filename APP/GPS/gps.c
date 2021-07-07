#include "gps.h" 
#include "systick.h" 							   
#include "usart.h" 								   
#include "stdio.h"	 
#include "stdarg.h"	 
#include "string.h"	 
#include "math.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F103开发板
//ATK-S1216F8 GPS模块驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2015/04/11
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved							  
////////////////////////////////////////////////////////////////////////////////// 	   
//
nmea_msg gpsx; //gps读取数据
u16 year=0;
u8 month=0;
u8 day=0;
u8 hour=0;
u8 min=0;
u8 sec=0;
u16 ms=0;

const u32 BAUD_id[9]={4800,9600,19200,38400,57600,115200,230400,460800,921600};//模块支持波特率数组

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
//从buf里面得到第cx个逗号所在的位置
//返回值:0~0XFE,代表逗号所在位置的偏移.
//       0XFF,代表不存在第cx个逗号							  
u8 NMEA_Comma_Pos(u8 *buf,u8 cx)
{	 		    
	u8 *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}
//m^n函数
//返回值:m^n次方.
u32 NMEA_Pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}
//str转换为数字,以','或者'*'结束
//buf:数字存储区
//dx:小数点位数,返回给调用函数
//返回值:转换后的数值
u32 NMEA_Str2num(u8 *buf,u8*dx)
{
	u8 *p=buf;
	u32 ires=0,fres=0;
	u8 ilen=0,flen=0,i;
	u8 mask=0;
	u32 res;
	while(1) //得到整数和小数的长度
	{
		if(*p=='-'){mask|=0X02;p++;}//是负数
		if(*p==','||(*p=='*'))break;//遇到结束了
		if(*p=='.'){mask|=0X01;p++;}//遇到小数点了
		else if(*p>'9'||(*p<'0'))	//有非法字符
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//去掉负号
	for(i=0;i<ilen;i++)	//得到整数部分数据
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//最多取5位小数
	*dx=flen;	 		//小数点位数
	for(i=0;i<flen;i++)	//得到小数部分数据
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}	  							 


//分析GNGGA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GNGGA_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;    
	p1=(u8*)strstr((const char *)buf,"$GNGGA");
	posx=NMEA_Comma_Pos(p1,9);								//得到海拔高度
	if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx);  
}

//分析GNRMC信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GNRMC_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;     
	u32 temp;
	u32 temp2;
	float rs;  
	p1=(u8*)strstr((const char *)buf,"$GNRMC");//"$GNRMC",经常有&和GNRMC分开的情况,故只判断GPRMC.
	posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//得到UTC时间,去掉ms
		temp2=NMEA_Str2num(p1+posx,&dx)%NMEA_Pow(10,dx);	 	//得到UTC时间,去掉ms
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;
		gpsx->utc.ms=temp2;
	}	
	posx=NMEA_Comma_Pos(p1,3);								//得到纬度
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
		gpsx->latitude=gpsx->latitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 
	}
	posx=NMEA_Comma_Pos(p1,4);								//南纬还是北纬 
	if(posx!=0XFF)gpsx->nshemi=*(p1+posx);					 
 	posx=NMEA_Comma_Pos(p1,5);								//得到经度
	if(posx!=0XFF)
	{												  
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
		gpsx->longitude=gpsx->longitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 
	}
	posx=NMEA_Comma_Pos(p1,6);								//东经还是西经
	if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);		 
	posx=NMEA_Comma_Pos(p1,9);								//得到UTC日期
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 				//得到UTC日期
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;	 	 
	} 
}

//提取NMEA-0183信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void GPS_Analysis(nmea_msg *gpsx,u8 *buf)
{
	NMEA_GNGGA_Analysis(gpsx,buf);	//GNGGA解析 	
	NMEA_GNRMC_Analysis(gpsx,buf);	//GPNMC解析
	//LBlocate(gpsx);
}
/*
///////////////////////////////////////////UBLOX 配置代码/////////////////////////////////////
////检查CFG配置执行情况
////返回值:0,ACK成功
////       1,接收超时错误
////       2,没有找到同步字符
////       3,接收到NACK应答
u8 SkyTra_Cfg_Ack_Check(void)
{			 
	u16 len=0,i;
	u8 rval=0;
	while((USART3_RX_STA&0X8000)==0 && len<100)//等待接收到应答   
	{
		len++;
		delay_ms(5);
	}		 
	if(len<100)   	//超时错误.
	{
		len=USART3_RX_STA&0X7FFF;	//此次接收到的数据长度 
		for(i=0;i<len;i++)
		{
			if(USART3_RX_BUF[i]==0X83)break;
			else if(USART3_RX_BUF[i]==0X84)
			{
				rval=3;
				break;
			}
		}
		if(i==len)rval=2;						//没有找到同步字符
	}else rval=1;								//接收超时错误
    USART3_RX_STA=0;							//清除接收
	return rval;  
}





//配置SkyTra_GPS/北斗模块波特率
//baud_id:0~8，对应波特率,4800/9600/19200/38400/57600/115200/230400/460800/921600	  
//返回值:0,执行成功;其他,执行失败(这里不会返回0了)
u8 SkyTra_Cfg_Prt(u32 baud_id)
{
	SkyTra_baudrate *cfg_prt=(SkyTra_baudrate *)USART3_TX_BUF;
	cfg_prt->sos=0XA1A0;		//引导序列(小端模式)
	cfg_prt->PL=0X0400;			//有效数据长度(小端模式)
	cfg_prt->id=0X05;		    //配置波特率的ID 
	cfg_prt->com_port=0X00;			//操作串口1
	cfg_prt->Baud_id=baud_id;	 	////波特率对应编号
	cfg_prt->Attributes=1; 		  //保存到SRAM&FLASH
	cfg_prt->CS=cfg_prt->id^cfg_prt->com_port^cfg_prt->Baud_id^cfg_prt->Attributes;
	cfg_prt->end=0X0A0D;        //发送结束符(小端模式)
	SkyTra_Send_Date((u8*)cfg_prt,sizeof(SkyTra_baudrate));//发送数据给SkyTra   
	delay_ms(200);				//等待发送完成 
	USART3_Init(BAUD_id[baud_id]);	//重新初始化串口3  
	return SkyTra_Cfg_Ack_Check();//这里不会反回0,因为UBLOX发回来的应答在串口重新初始化的时候已经被丢弃了.
} 
//配置SkyTra_GPS/北斗模块的时钟脉冲宽度
//width:脉冲宽度1~100000(us)
//返回值:0,发送成功;其他,发送失败.
u8 SkyTra_Cfg_Tp(u32 width)
{
	u32 temp=width;
	SkyTra_pps_width *cfg_tp=(SkyTra_pps_width *)USART3_TX_BUF;
	temp=(width>>24)|((width>>8)&0X0000FF00)|((width<<8)&0X00FF0000)|((width<<24)&0XFF000000);//小端模式
	cfg_tp->sos=0XA1A0;		    //cfg header(小端模式)
	cfg_tp->PL=0X0700;        //有效数据长度(小端模式)
	cfg_tp->id=0X65	;			    //cfg tp id
	cfg_tp->Sub_ID=0X01;			//数据区长度为20个字节.
	cfg_tp->width=temp;		  //脉冲宽度,us
	cfg_tp->Attributes=0X01;  //保存到SRAM&FLASH	
	cfg_tp->CS=cfg_tp->id^cfg_tp->Sub_ID^(cfg_tp->width>>24)^(cfg_tp->width>>16)&0XFF^(cfg_tp->width>>8)&0XFF^cfg_tp->width&0XFF^cfg_tp->Attributes;    	//用户延时为0ns
	cfg_tp->end=0X0A0D;       //发送结束符(小端模式)
	SkyTra_Send_Date((u8*)cfg_tp,sizeof(SkyTra_pps_width));//发送数据给NEO-6M  
	return SkyTra_Cfg_Ack_Check();
}
//配置SkyTraF8-BD的更新速率    
//Frep:（取值范围:1,2,4,5,8,10,20,25,40,50）测量时间间隔，单位为Hz，最大不能大于50Hz
//返回值:0,发送成功;其他,发送失败.
u8 SkyTra_Cfg_Rate(u8 Frep)
{
	SkyTra_PosRate *cfg_rate=(SkyTra_PosRate *)USART3_TX_BUF;
 	cfg_rate->sos=0XA1A0;	    //cfg header(小端模式)
	cfg_rate->PL=0X0300;			//有效数据长度(小端模式)
	cfg_rate->id=0X0E;	      //cfg rate id
	cfg_rate->rate=Frep;	 	  //更新速率
	cfg_rate->Attributes=0X01;	   	//保存到SRAM&FLASH	.
	cfg_rate->CS=cfg_rate->id^cfg_rate->rate^cfg_rate->Attributes;//脉冲间隔,us
	cfg_rate->end=0X0A0D;       //发送结束符(小端模式)
	SkyTra_Send_Date((u8*)cfg_rate,sizeof(SkyTra_PosRate));//发送数据给NEO-6M 
	return SkyTra_Cfg_Ack_Check();
}
//发送一批数据给Ublox NEO-6M，这里通过串口3发送
//dbuf：数据缓存首地址
//len：要发送的字节数
void SkyTra_Send_Date(u8* dbuf,u16 len)
{
	u16 j;
	for(j=0;j<len;j++)//循环发送数据
	{
		while((USART3->SR&0X40)==0);//循环发送,直到发送完毕   
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
	double mid = 117;//中央经度
	double x,y;
	double e1=sqrt(pow(a,2.0)-pow(b,2.0))/a;
	double e2=sqrt(pow(a,2.0)-pow(b,2.0))/b;
	double l=LL-mid;  //l为经差
	double B=BB*3.141592683/180.0;
	double L=LL*3.141592683/180.0;
	l=l*3.141592683/180.0;
	 //X0为当l=0时，从赤道起算的子午线弧长 
    //计算子午线弧长X的系数
	double A0=1.0+3.0/4*pow(e1,2.0)+45.0/64*pow(e1,4.0)+350.0/512*pow(e1,6.0)+11025.0/16384*pow(e1,8.0);
	double A2=-(3.0/4*pow(e1,2.0)+60.0/64*pow(e1,4.0)+525.0/512*pow(e1,6.0)+17640.0/16384*pow(e1,8.0))/2.0;
	double A4=(15.0/64*pow(e1,4.0)+210.0/512*pow(e1,6.0)+8820.0/16384*pow(e1,8.0))/4;
	double A6=-(35.0/512*pow(e1,6.0)+2520.0/16384*pow(e1,8.0))/6;
	double A8=(315.0/16384.0*pow(e1,8.0))/8;
    //计算子午线弧长X
	double X0=a*(1.0-pow(e1,2.0))*(A0*B+A2*sin(2*B)+A4*sin(4*B)+A6*sin(6*B)+A8*sin(8*B));
	double t=tan(B);
	double anke=e2*cos(B);
	double N=a/sqrt(1.0-pow(e1,2.0)*pow(sin(B),2.0));  //N为投影点的卯酉圈曲率半径
	//坐标计算
	x=X0+1.0/2*N*t*pow(cos(B),2.0)*pow(l,2.0)+1.0/24*N*t*(5.0-pow(t,2.0)+9.0*pow(anke,2.0)+4.0*pow(anke,4.0))*pow(cos(B),4.0)
		*pow(l,4.0)+1.0/720*N*t*(61.0-58.0*pow(t,2.0)+pow(t,4.0)+270*pow(anke,2.0)-330.0*pow(anke,2.0)*pow(t,2.0))*pow(cos(B),6.0)*pow(l,6.0);
	y=N*cos(B)*l+1.0/6*N*(1-pow(t,2.0)+pow(anke,2.0))*pow(cos(B),3.0)*pow(l,3.0)
        +1.0/120*N*(5.0-18.0*pow(t,2.0)+pow(t,4.0)+14.0*pow(anke,2.0)-58*pow(anke,2)
		*pow(t,2))*pow(cos(B),5.0)*pow(l,5.0);
	y+=500000.0;  //add 500km
	gpsx->x = x * 100000;
	gpsx->y = y * 100000;
}




