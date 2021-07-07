#include "ch376.h"
#include "SysTick.h"
#include "usart.h"
//#include "usart.h"
//#include "string.h"
//#include "spi.h"

u8 *RxBuf = USART6_RX_BUF;
u8 *TxBuf = USART6_TX_BUF;
/////////////////////////////////////////////-----SPI端口初始化-----////////////////////////////////////////////
 
void CH376S_Init(void)
{
	/*
	GPIO_InitTypeDef  GPIO_InitStructure, GPIO_InitStructure_in;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //使能GPIOA时钟
	
	//中断，上拉输入
	GPIO_InitStructure_in.GPIO_Pin = GPIO_Pin_13;       //PA13
	GPIO_InitStructure_in.GPIO_Mode = GPIO_Mode_IN;     //输入模式
	GPIO_InitStructure_in.GPIO_PuPd = GPIO_PuPd_UP;     //上拉
	GPIO_InitStructure_in.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
	GPIO_Init(GPIOA,&GPIO_InitStructure_in);            //初始化结构体
		
	//复位，推挽输出,可以去掉不要
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;          //PA14
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉？
	GPIO_Init(GPIOA, &GPIO_InitStructure);              //初始化结构体
	*/
	USART6_Init(2000000);
}


//CH376S初始化
void CH376S_Reset()
{
//  CH376_RST	= 1;			 //复位
  delay_us(1);
//  CH376_RST	= 0;			 //禁止复位
  delay_ms(100);     //延时100毫秒
}

/////////////////////////////////////////////-----低层驱动-----////////////////////////////////////////////

//延时指定微秒时间,根据单片机主频调整,不精确.
void mDelayuS(u8 us)
{
	delay_us(us);
}

//延时指定毫秒时间,根据单片机主频调整,不精确
void mDelaymS(u8 ms)
{
	delay_ms(ms);
}

/////////////////////////////////////////////-----高层驱动-----////////////////////////////////////////////

//写命令
void	xWriteCH376Cmd(u8 cmd)
{
	TxBuf[0] = 0x57;
	TxBuf[1] = 0xab;
	TxBuf[2] = cmd;
	SendUsart6TXBUF(3);
}


//写数据
void	xWriteCH376Data(u8 data)
{
	TxBuf[0] = data;
	SendUsart6TXBUF(1);
}


//读数据
u8 xReadCH376Data(void)
{
	u8 res;
	while (USART_GetFlagStatus(USART6, USART_FLAG_RXNE) == RESET);
	res = USART_ReceiveData(USART6);
	USART_ClearFlag(USART6, USART_FLAG_RXNE);
	return res;
}


/*
//查询CH376中断(INT#低电平). 
//返回: FALSE:无中断.   TRUE:有中断.
u8	Query376Interrupt(void)
{
	u8 i;
	//i = (CH376_INT_WIRE ? FALSE : TRUE); 	// 如果连接了CH376的中断引脚则直接查询中断引脚 
	return 0; 
}
*/
/////////////////////////////////////////////-----usb初始化-----////////////////////////////////////////////


//CH376端口初始化
void	CH376_PORT_INIT(void)
{
   //CH376_INT_WIRE = 1;   //默认为高电平,SPI模式3,也可以用SPI模式0,但模拟程序可能需稍做修改
                         //对于双向I/O引脚模拟SPI接口,那么必须在此设置SPI_SCS,SPI_SCK,SPI_SDI为输出方向,SPI_SDO为输入方向
}


//初始化CH376.
//返回: FALSE:无中断.  TRUE:有中断.
u8	mInitCH376Host(void)
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
	printf("取反测试：输入0x55, 输出0x%02X \r\n",(unsigned short)res);
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
	printf("配置USB工作模式，输出0x%02X \r\n",(unsigned short)res);
	
	if (res == CMD_RET_SUCCESS || res == USB_INT_CONNECT)  //RES=51  命令操作成功
	{
		return(USB_INT_SUCCESS); //USB事务或者传输操作成功
	}
	else
	{
		return(ERR_USB_UNKNOWN);// 设置模式错误
	}
}
