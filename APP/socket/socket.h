#include "system.h"
#ifndef _SOCKET_H_
#define _SOCKET_H_
//��ʶPacket_connectData.Work_Status״̬
typedef enum {
	STATUS_OK=0,						//0x00
	STATUS_GPS_UNLOCATED,		//0x01
	STATUS_SD_ERROR,				//0x02
	STATUS_AD0_ERROR,				//0x03
	STATUS_AD1_ERROR,				//0x04
	STATUS_AD2_ERROR,				//0x05
	STATUS_USB_ERROR				//0x06
}Work_Status;

//��ʶPacket_connectData.IsReset״̬
typedef enum {
	RESET_NO=0,
	RESET_YES
}CMD_Reset;

//��ʶ�����ϱ�����ֵ
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
	u8 Station_ID[10];	//�豸ID
	u8 Authentication_Code[10];	//��֤��
	CMD_Reset IsReset;	//��������ʱ��ʶ�Ƿ�λ
	Work_Status WorkStatus;	//����״̬
	u32	Sent_Count;	//������ɸ���
	u32 Wait_Count;	//�ȴ����͸���
	u8 BatteryPower;	//��ص���
	Time time;
}Packet_connectData;



u8 Serialize_Report(u8* buf,Packet_connectData data,u8* buflen);
u8 Deserialize_Receive(u8* buf,Packet_connectData* data);
u8 transport_sendPacketBuffer(u8* buf, u8 buflen);
void sim7600_LinkToSever(u8* ipaddr,u8* port);
void sim7600_close_sever(void);

#endif
