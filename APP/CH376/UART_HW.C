/* CH376芯片 硬件标准异步串口连接的硬件抽象层 V1.0 */
/* 提供I/O接口子程序 */

#include	"HAL.H"
#include "usart.h"

/* 本例中的硬件连接方式如下(实际应用电路可以参照修改下述定义及子程序) */
/* 单片机的引脚    CH376芯片的引脚
      TXD                  RXD
      RXD                  TXD       */


u8 *RxBuf = USART6_RX_BUF;
u8 *TxBuf = USART6_TX_BUF;
#define CH376_INT_WIRE			INT0	/* 假定CH376的INT#引脚,如果未连接那么也可以通过查询串口中断状态码实现 */

#define	UART_INIT_BAUDRATE	2000000	/* 默认通讯波特率9600bps,建议通过硬件引脚设定直接选择更高的CH376的默认通讯波特率 */
#define	UART_WORK_BAUDRATE	2000000	/* 正式通讯波特率57600bps */

void	CH376_PORT_INIT( void )  /* 由于使用异步串口读写时序,所以进行初始化 */
{
	USART6_Init(UART_INIT_BAUDRATE);
}

#ifdef	UART_WORK_BAUDRATE
void	SET_WORK_BAUDRATE( void )  /* 将单片机切换到正式通讯波特率 */
{
	USART6_Init(UART_WORK_BAUDRATE);
}
#endif

//#define	xEndCH376Cmd()  /* 结束CH376命令,仅用于SPI接口方式 */

void	xWriteCH376Cmd( UINT8 cmd )  /* 向CH376写命令 */
{
	TxBuf[0] = 0x57;
	TxBuf[1] = 0xab;
	TxBuf[2] = cmd;
	SendUsart6TXBUF(3);
}

void	xWriteCH376Data( UINT8 data )  /* 向CH376写数据 */
{
	TxBuf[0] = data;
	SendUsart6TXBUF(1);
}

UINT8	xReadCH376Data( void )  /* 从CH376读数据 */
{
//	if(USART6_RX_STA|0x1000)
//	{
//		return USART6_RX_BUF[0];
//	}
	u8 res;
	u32 t;
	while (USART_GetFlagStatus(USART6, USART_FLAG_RXNE) == RESET)//U盘出现故障防止卡死，Edit By Yao
	{
		t++;
		if(t>1000000)return 0;
	}
	res = USART_ReceiveData(USART6);
	USART_ClearFlag(USART6, USART_FLAG_RXNE);
	return res;
}

/* 查询CH376中断(INT#低电平) */
UINT8	Query376Interrupt( void )
{
//#ifdef	CH376_INT_WIRE
//	return( CH376_INT_WIRE ? FALSE : TRUE );  /* 如果连接了CH376的中断引脚则直接查询中断引脚 */
//#else
//	if ( RI ) {  /* 如果未连接CH376的中断引脚则查询串口中断状态码 */
//		RI = 0;
//		return( TRUE );
//	}
//	else return( FALSE );
//#endif
	return 0;
}

UINT8	mInitCH376Host( void )  /* 初始化CH376 */
{
	u8 i, j, res;	
		
	mDelaymS(200);
	mDelaymS(200);
	mDelaymS(200);
	CH376_PORT_INIT();           // 接口硬件初始化 
	
	//取反测试，输入数据，取反输出
	for (i = 0; i < 3; ++i)
	{
		TxBuf[0] = 0x57;
		TxBuf[1] = 0xab;
		TxBuf[2] = CMD11_CHECK_EXIST;
		TxBuf[3] = 0x55;
		SendUsart6TXBUF(4);
		res = xReadCH376Data();
	}
//	printf("取反测试：输入0x55, 输出0x%02X \r\n",(unsigned short)res);
	if (res != 0xAA) return(ERR_USB_UNKNOWN);  // 通讯接口不正常,可能原因有:接口连接异常,其它设备影响(片选不唯一),串口波特率,一直在复位,晶振不工作 
	
	//配置usb工作模式
	for (i = 0; i < 3; ++i)
	{
		USART6_RX_REC_ATCOMMAD = 1;
		//TxBuf[0] = 0x57;
		//TxBuf[1] = 0xab;
		TxBuf[2] = CMD11_SET_USB_MODE;
		TxBuf[3] = 0x05;
		SendUsart6TXBUF(4);
		res = xReadCH376Data();
	}
//	printf("配置USB工作模式，输出0x%02X \r\n",(unsigned short)res);
	
	if (res == CMD_RET_SUCCESS || res == USB_INT_CONNECT)  //RES=51  命令操作成功
	{
		return(USB_INT_SUCCESS); //USB事务或者传输操作成功
	}
	else
	{
		return(ERR_USB_UNKNOWN);// 设置模式错误
	}
}
