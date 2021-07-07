#ifndef _ch376_H
#define _ch376_H

#include "CH376INC.H"
#include "system.h"

//#define	CH376_INT_WIRE    PAin(13)      // PA13  上拉输入  INT   
//#define CH376_RST         PAout(14)     // PA14  推挽输出  RST   


#define xEndCH376Cmd() 
/* 附加的USB操作状态定义 */
#define		ERR_USB_UNKNOWN		0xFA	        // 未知错误,不应该发生的情况,需检查硬件或者程序错误

extern u8 *RxBuf;
extern u8 *TxBuf;


void CH376S_Init(void);
void CH376S_Reset();                      // CH376S复位

void mDelayuS(u8 us);
void mDelaymS(u8 ms);

void xWriteCH376Cmd(u8 cmd);  	          // 向CH376写命令
void xWriteCH376Data(u8 data);  	        // 向CH376写数据
u8 	xReadCH376Data( void );			          // 从CH376读数据

//u8 	Query376Interrupt( void );		        // 查询CH376中断(INT#引脚为低电平)

void 	CH376_PORT_INIT( void );  		      // CH376通讯接口初始化
u8 	mInitCH376Host( void );			          // 初始化CH376




#endif




