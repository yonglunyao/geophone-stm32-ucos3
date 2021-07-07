#include "system.h"
#ifndef _SOCKET_H_
#define _SOCKET_H_
//标识Packet_connectData.Work_Status状态
typedef enum {
	STATUS_OK=0,						//0x00
	STATUS_GPS_UNLOCATED,		//0x01
	STATUS_SD_ERROR,				//0x02
	STATUS_AD0_ERROR,				//0x03
	STATUS_AD1_ERROR,				//0x04
	STATUS_AD2_ERROR,				//0x05
	STATUS_USB_ERROR				//0x06
}Work_Status;

//标识Packet_connectData.IsReset状态
typedef enum {
	RESET_NO=0,
	RESET_YES
}CMD_Reset;

//标识数据上报返回值
typedef enum {
	REPORT_OK=0,
	REPORT_ABORT,
	REPORT_FAILD
}RES_REPORT;

typedef struct{
	u8 year;
	u8 month;
	u8 day;
	u8 hour;
	u8 minute;
	u8 second;
}Time;

typedef struct{
	u8 Station_ID[10];	//设备ID
	u8 Authentication_Code[10];	//认证码
	CMD_Reset IsReset;	//数据下行时标识是否复位
	Work_Status WorkStatus;	//工作状态
	u32	Sent_Count;	//发送完成个数
	u32 Wait_Count;	//等待发送个数
	u8 BatteryPower;	//电池电量
	Time time;
}Packet_connectData;



u8 Serialize_Report(u8* buf,Packet_connectData data,u8* buflen);
u8 Deserialize_Receive(u8* buf,Packet_connectData* data);
u8 transport_sendPacketBuffer(u8* buf, u8 buflen);
void sim7600_LinkToSever(u8* ipaddr,u8* port);
void sim7600_close_sever(void);

#endif
