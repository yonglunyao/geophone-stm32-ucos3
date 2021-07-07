#ifndef __SIM7600_H__
#define __SIM7600_H__	 
#include "system.h"
//////////////////////////////////////////////////////////////////////////////////	 

#define RCC_SIM_RST 	RCC_AHB1Periph_GPIOF	//ADD By Yao
#define PORT_SIM_RST	GPIOF	//ADD By Yao
#define PIN_SIM_RST		GPIO_Pin_4	//ADD By Yao

#define SIM_OK 0
#define SIM_COMMUNTION_ERR 0xff
#define SIM_CPIN_ERR 0xfe
#define SIM_CREG_FAIL 0xfd
#define SIM_MAKE_CALL_ERR 0Xfc
#define SIM_ATA_ERR       0xfb

#define SIM_CMGF_ERR 0xfa
#define SIM_CSCS_ERR 0xf9
#define SIM_CSCA_ERR 0xf8
#define SIM_CSMP_ERR 0Xf7
#define SIM_CMGS_ERR       0xf6
#define SIM_CMGS_SEND_FAIL       0xf5

#define SIM_CNMI_ERR 0xf4

#define SIM_RST_0()		GPIO_ResetBits(PORT_SIM_RST, PIN_SIM_RST)
#define SIM_RST_1()		GPIO_SetBits(PORT_SIM_RST, PIN_SIM_RST)

u8 sim7600_send_cmd(u8 *cmd,u8 *ack,u16 waittime);
u8* sim7600_check_cmd(u8 *str);
extern u8 Flag_Rec_Message;	//收到短信标示


extern u8 SIM7600_CSQ[3];
extern u8 GSM_Dect(void);
extern u8 SIM7600_CONNECT_SERVER_SEND_INFOR(u8 *IP_ADD,u8 *COM);
extern u8 SIM7600_GPRS_SEND_DATA(u8 *temp_data);
u8 SIM7600_CONNECT_SERVER(void);
void SIM7600_Init(void);	//ADD By Yao
void SIM7600_Reset(void);	//ADD By Yao
#endif





