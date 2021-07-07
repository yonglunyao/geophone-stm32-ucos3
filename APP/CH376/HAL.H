/* CH376芯片 硬件抽象层 V1.0 */
/* 提供I/O接口子程序 */

//#include	<reg52.h>
#include	"CH376INC.H"

#ifndef	__CH376_HAL_H__
#define __CH376_HAL_H__

#ifdef __CX51__
#ifndef __C51__
#define __C51__		1
#endif
#endif

#ifdef __C51__
#define	BIG_ENDIAN		1
#endif
#ifdef __C51__
#pragma NOAREGS
#endif


/* 附加的USB操作状态定义 */
#define		ERR_USB_UNKNOWN		0xFA	/* 未知错误,不应该发生的情况,需检查硬件或者程序错误 */

/* 延时指定微秒时间,根据单片机主频调整,不精确 */
void	mDelayuS( UINT8 us );

/* 延时指定毫秒时间,根据单片机主频调整,不精确 */
void	mDelaymS( UINT8 ms );

void	CH376_PORT_INIT( void );  		/* CH376通讯接口初始化 */

void	xEndCH376Cmd( void );			/* 结束CH376命令,仅用于SPI接口方式 */

void	xWriteCH376Cmd( UINT8 mCmd );	/* 向CH376写命令 */

void	xWriteCH376Data( UINT8 mData );	/* 向CH376写数据 */

UINT8	xReadCH376Data( void );			/* 从CH376读数据 */

UINT8	Query376Interrupt( void );		/* 查询CH376中断(INT#引脚为低电平) */

UINT8	mInitCH376Host( void );			/* 初始化CH376 */

#endif
