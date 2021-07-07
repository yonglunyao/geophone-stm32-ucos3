#include "dma.h"
/* SPI DR寄存器的地址，可通过查看SPI2一步一步得到 */

/* 定义接收缓冲和发送缓冲 */
u8 SPI1_RX_BUFFER[SPI1_RX_LEN];
u8 SPI1_TX_BUFFER[SPI1_TX_LEN]; 
u8 SPI2_RX_BUFFER[SPI2_RX_LEN];
u8 SPI2_TX_BUFFER[SPI2_TX_LEN]; 
u8 SPI3_RX_BUFFER[SPI3_RX_LEN];
u8 SPI3_TX_BUFFER[SPI3_TX_LEN]; 

/*******************************************************************************
* 函 数 名         : DMAx_Init
* 函数功能		   : DMA初始化函数
* 输    入         : DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
					 chx:DMA通道选择,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
					 par:外设地址
					 mar:存储器地址
					 ndtr:数据传输量
* 输    出         : 无
*******************************************************************************/ 
void DMAx_Init(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr)
{
	DMA_InitTypeDef  DMA_InitStructure;
	
	if((u32)DMA_Streamx>(u32)DMA2)//得到当前stream是属于DMA2还是DMA1
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 	
	}
	else 
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
	}
	DMA_DeInit(DMA_Streamx);
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}//等待DMA可配置 
	
	/* 配置 DMA Stream */
	DMA_InitStructure.DMA_Channel = chx;  //通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = par;//DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = mar;//DMA 存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
	DMA_InitStructure.DMA_BufferSize = ndtr;//数据传输量 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA_Streamx, &DMA_InitStructure);//初始化DMA Stream
	
}

/*******************************************************************************
* 函 数 名         : DMAx_Enable
* 函数功能		   : 开启一次DMA传输
* 输    入         : DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
					 ndtr:数据传输量
* 输    出         : 无
*******************************************************************************/ 
void DMAx_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
 
	DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输 
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE);	//确保DMA可以被设置  
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量  
	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输 
}	  

/* 如果定义为DMA传输方式，进行DMA初始化 */
/**
  *	@breif  The spi dma init function.
  * @param  None
  * @retval None
  */  
void spi1_dma_init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	/* 打开DMA2时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	/* DMA RX config */
	DMA_InitStructure.DMA_Channel = DMA_Channel_3;                          //选择DMA通道
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI1_DR_ADDR;                //外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SPI1_RX_BUFFER;        //存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //内存到外部设备
	DMA_InitStructure.DMA_BufferSize = 32;                                  //传输数据长度
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不自增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存地址自增
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //内存数据为8位
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //不采取循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //优先级为中等
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //不采取FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           //不采取FIFO模式，该参数无关紧要
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //单次传输
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);
	
	
	/* DMA TX Config */
	DMA_InitStructure.DMA_Channel = DMA_Channel_3;                          //选择DMA通道
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI1_DR_ADDR;                //外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SPI1_TX_BUFFER;        //存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //内存到外部设备
	DMA_InitStructure.DMA_BufferSize = 32;                                  //传输数据长度
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不自增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存地址自增
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //内存数据为8位
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //不采取循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //优先级为中等
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //不采取FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           //不采取FIFO模式，该参数无关紧要
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //单次传输
	DMA_Init(DMA2_Stream5, &DMA_InitStructure);
}

void spi2_dma_init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	/* 打开DMA1时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	/* DMA RX config */
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;                          //选择DMA通道
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI2_DR_ADDR;                //外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SPI2_RX_BUFFER;        //存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //内存到外部设备
	
	DMA_InitStructure.DMA_BufferSize = 30;                                  //传输数据长度
	
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不自增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存地址自增
	
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //内存数据为8位
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //不采取循环模式
	
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //优先级为中等
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //不采取FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           //不采取FIFO模式，该参数无关紧要
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //单次传输
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);
	
	
	/* DMA TX Config */
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;                          //选择DMA通道
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI2_DR_ADDR;                //外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SPI2_TX_BUFFER;        //存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //内存到外部设备
	
	DMA_InitStructure.DMA_BufferSize = 30;                                  //传输数据长度
	
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不自增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存地址自增
	
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //内存数据为8位
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //不采取循环模式
	
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //优先级为中等
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //不采取FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           //不采取FIFO模式，该参数无关紧要
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //单次传输
	DMA_Init(DMA1_Stream4, &DMA_InitStructure);
}

void spi3_dma_init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	/* 打开DMA2时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	/* DMA RX config */
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;                          //选择DMA通道
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI3_DR_ADDR;                //外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SPI3_RX_BUFFER;        //存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //内存到外部设备
	
	DMA_InitStructure.DMA_BufferSize = 30;                                  //传输数据长度
	
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不自增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存地址自增
	
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //内存数据为8位
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //不采取循环模式
	
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //优先级为中等
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //不采取FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           //不采取FIFO模式，该参数无关紧要
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //单次传输
	DMA_Init(DMA1_Stream0, &DMA_InitStructure);
	
	
	/* DMA TX Config */
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;                          //选择DMA通道
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI3_DR_ADDR;                //外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SPI3_TX_BUFFER;        //存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //内存到外部设备
	
	DMA_InitStructure.DMA_BufferSize = 30;                                  //传输数据长度
	
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不自增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存地址自增
	
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //内存数据为8位
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //不采取循环模式
	
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //优先级为中等
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //不采取FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           //不采取FIFO模式，该参数无关紧要
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //单次传输
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);
}

/**
  *	@breif  The spi dma trans function.
  * @param  rx_buf -- the point of rx buffer
  * @param  tx_buf -- the point of tx buffer
  * @elngth length -- the size of data.
  * @retval None
  */  
void spi1_trans(uint8_t *rx_buf,     //接收buffer指针
			   uint8_t *tx_buf,     //发送buffer指针
			   uint16_t length)     //传输数据长度
{
	/* 关闭DMA通道 */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream5, DISABLE);
	
	/* 设置传输字节数 */
	DMA_SetCurrDataCounter(DMA2_Stream2, (uint16_t)length);
	DMA_SetCurrDataCounter(DMA2_Stream5, (uint16_t)length);
	/* 发送接收数据存储地址自增 */
	DMA2_Stream2->CR |= (1 << 10);
	DMA2_Stream5->CR |= (1 << 10);
	
	/* 设置接收和发送的内存地址 */
	DMA2_Stream2->M0AR = (uint32_t)rx_buf;
	DMA2_Stream5->M0AR = (uint32_t)tx_buf;

	/* 读取一次DR，使其清空 */
	SPI1->DR;
	
	/* 等待发送区为空 */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	
	/* 打开DMA通道 */
	DMA_Cmd(DMA2_Stream2, ENABLE);
	DMA_Cmd(DMA2_Stream5, ENABLE);
	
	/* 传输完成 */
	while( DMA_GetFlagStatus(DMA2_Stream5, DMA_FLAG_TCIF5) == RESET);
	while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
	
	/* 关闭DMA通道 */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream5, DISABLE);	
	
	/* 清除DMA传输完成标志 */
	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
	DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);
}

void spi2_trans(uint8_t *rx_buf,     //接收buffer指针
			   uint8_t *tx_buf,     //发送buffer指针
			   uint16_t length)     //传输数据长度
{
	/* 关闭DMA通道 */
	DMA_Cmd(DMA1_Stream3, DISABLE);
	DMA_Cmd(DMA1_Stream4, DISABLE);
	
	/* 设置传输字节数 */
	DMA_SetCurrDataCounter(DMA1_Stream3, (uint16_t)length);
	DMA_SetCurrDataCounter(DMA1_Stream4, (uint16_t)length);
	/* 发送接收数据存储地址自增 */
	DMA1_Stream3->CR |= (1 << 10);
	DMA1_Stream4->CR |= (1 << 10);
	
	/* 设置接收和发送的内存地址 */
	DMA1_Stream3->M0AR = (uint32_t)rx_buf;
	DMA1_Stream4->M0AR = (uint32_t)tx_buf;

	/* 读取一次DR，使其清空 */
	SPI2->DR;
	
	/* 等待发送区为空 */
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	
	/* 打开DMA通道 */
	DMA_Cmd(DMA1_Stream3, ENABLE);
	DMA_Cmd(DMA1_Stream4, ENABLE);
	
	/* 传输完成 */
	while( DMA_GetFlagStatus(DMA1_Stream3, DMA_FLAG_TCIF3) == RESET);
	while( DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4) == RESET);
	
	/* 关闭DMA通道 */
	DMA_Cmd(DMA1_Stream3, DISABLE);
	DMA_Cmd(DMA1_Stream4, DISABLE);	
	
	/* 清除DMA传输完成标志 */
	DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
	DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
}

void spi3trans(uint8_t *rx_buf,     //接收buffer指针
			   uint8_t *tx_buf,     //发送buffer指针
			   uint16_t length)     //传输数据长度
{
	/* 关闭DMA通道 */
	DMA_Cmd(DMA1_Stream0, DISABLE);
	DMA_Cmd(DMA1_Stream5, DISABLE);
	
	/* 设置传输字节数 */
	DMA_SetCurrDataCounter(DMA1_Stream0, (uint16_t)length);
	DMA_SetCurrDataCounter(DMA1_Stream5, (uint16_t)length);
	/* 发送接收数据存储地址自增 */
	DMA1_Stream0->CR |= (1 << 10);
	DMA1_Stream5->CR |= (1 << 10);
	
	/* 设置接收和发送的内存地址 */
	DMA1_Stream0->M0AR = (uint32_t)rx_buf;
	DMA1_Stream5->M0AR = (uint32_t)tx_buf;

	/* 读取一次DR，使其清空 */
	SPI3->DR;
	
	/* 等待发送区为空 */
	while(SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
	
	/* 打开DMA通道 */
	DMA_Cmd(DMA1_Stream0, ENABLE);
	DMA_Cmd(DMA1_Stream5, ENABLE);
	
	/* 传输完成 */
	while( DMA_GetFlagStatus(DMA1_Stream0, DMA_FLAG_TCIF0) == RESET);
	while( DMA_GetFlagStatus(DMA1_Stream5, DMA_FLAG_TCIF5) == RESET);
	
	/* 关闭DMA通道 */
	DMA_Cmd(DMA1_Stream0, DISABLE);
	DMA_Cmd(DMA1_Stream5, DISABLE);	
	
	/* 清除DMA传输完成标志 */
	DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);
	DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
}

/**
  *	@breif  The spi dma trans function.
  * @param  rx_buf -- the point of rx buffer
  * @param  tx_data -- the spi tx data.
  * @elngth length -- the size of data.
  * @retval None
  */  

void spi1_trans_read(uint8_t *rx_buf,
					uint8_t *tx_data,
					uint16_t length)
{
	/* 关闭DMA通道 */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream5, DISABLE);
	
	/* 设置传输字节数 */
	DMA_SetCurrDataCounter(DMA2_Stream2, (uint16_t)length);
	DMA_SetCurrDataCounter(DMA2_Stream5, (uint16_t)length);
	
	/* 发送DMA流的发送地址不自增 */
	DMA2_Stream5->CR &= ~(1 << 10);
	
	/* 设置接收和发送的内存地址 */
	DMA2_Stream2->M0AR = (uint32_t)rx_buf;
	DMA2_Stream5->M0AR = (uint32_t)tx_data;

	/* 读取一次DR，使其清空 */
	SPI1->DR;
	
	/* 等待发送区为空 */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	
	/* 打开DMA通道 */
	DMA_Cmd(DMA2_Stream2, ENABLE);
	DMA_Cmd(DMA2_Stream5, ENABLE);
	
	/* 传输完成 */
	while( DMA_GetFlagStatus(DMA2_Stream5, DMA_FLAG_TCIF5) == RESET);
	while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
	
	/* 关闭DMA通道 */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream5, DISABLE);	
	
	/* 清除DMA传输完成标志 */
	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
	DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);	
}


void spi2_trans_read(uint8_t *rx_buf,
					uint8_t *tx_data,
					uint16_t length)
{
	/* 关闭DMA通道 */
	DMA_Cmd(DMA1_Stream3, DISABLE);
	DMA_Cmd(DMA1_Stream4, DISABLE);
	
	/* 设置传输字节数 */
	DMA_SetCurrDataCounter(DMA1_Stream3, (uint16_t)length);
	DMA_SetCurrDataCounter(DMA1_Stream4, (uint16_t)length);
	
	/* 发送DMA流的发送地址不自增 */
	DMA1_Stream4->CR &= ~(1 << 10);
	
	/* 设置接收和发送的内存地址 */
	DMA1_Stream3->M0AR = (uint32_t)rx_buf;
	DMA1_Stream4->M0AR = (uint32_t)tx_data;

	/* 读取一次DR，使其清空 */
	SPI2->DR;
	
	/* 等待发送区为空 */
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	
	/* 打开DMA通道 */
	DMA_Cmd(DMA1_Stream3, ENABLE);
	DMA_Cmd(DMA1_Stream4, ENABLE);
	
	/* 传输完成 */
	while( DMA_GetFlagStatus(DMA1_Stream3, DMA_FLAG_TCIF3) == RESET);
	while( DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4) == RESET);
	
	/* 关闭DMA通道 */
	DMA_Cmd(DMA1_Stream3, DISABLE);
	DMA_Cmd(DMA1_Stream4, DISABLE);	
	
	/* 清除DMA传输完成标志 */
	DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
	DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);	
}

void spi3_trans_read(uint8_t *rx_buf,
					uint8_t *tx_data,
					uint16_t length)
{
		/* 关闭DMA通道 */
	DMA_Cmd(DMA1_Stream0, DISABLE);
	DMA_Cmd(DMA1_Stream5, DISABLE);
	
	/* 设置传输字节数 */
	DMA_SetCurrDataCounter(DMA1_Stream0, (uint16_t)length);
	DMA_SetCurrDataCounter(DMA1_Stream5, (uint16_t)length);
	
	/* 发送DMA流的发送地址不自增 */
	DMA1_Stream5->CR &= ~(1 << 10);
	
	/* 设置接收和发送的内存地址 */
	DMA1_Stream0->M0AR = (uint32_t)rx_buf;
	DMA1_Stream5->M0AR = (uint32_t)tx_data;

	/* 读取一次DR，使其清空 */
	SPI3->DR;
	
	/* 等待发送区为空 */
	while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
	
	/* 打开DMA通道 */
	DMA_Cmd(DMA1_Stream0, ENABLE);
	DMA_Cmd(DMA1_Stream5, ENABLE);
	
	/* 传输完成 */
	while( DMA_GetFlagStatus(DMA1_Stream0, DMA_FLAG_TCIF0) == RESET);
	while( DMA_GetFlagStatus(DMA1_Stream5, DMA_FLAG_TCIF5) == RESET);
	
	/* 关闭DMA通道 */
	DMA_Cmd(DMA1_Stream0, DISABLE);
	DMA_Cmd(DMA1_Stream5, DISABLE);	
	
	/* 清除DMA传输完成标志 */
	DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);
	DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);	
}


/**
  *	@breif  The spi dma trans function.
  * @param  tx_buf -- the point of tx buffer
  * @param  rx_data -- the spi rx data.
  * @elngth length -- the size of data.
  * @retval None
  */  

void spi1_trans_write(uint8_t *rx_data,
					 uint8_t *tx_buffer,
					 uint16_t length)
{
	/* 关闭DMA通道 */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream5, DISABLE);
	
	/* 设置传输字节数 */
	DMA_SetCurrDataCounter(DMA2_Stream2, (uint16_t)length);
	DMA_SetCurrDataCounter(DMA2_Stream5, (uint16_t)length);
	
	/* 接收DMA流的接收地址不自增 */
	DMA2_Stream2->CR &= ~(1 << 10);
	
	/* 设置接收和发送的内存地址 */
	DMA2_Stream2->M0AR = (uint32_t)rx_data;
	DMA2_Stream5->M0AR = (uint32_t)tx_buffer;

	/* 读取一次DR，使其清空 */
	SPI1->DR;
	
	/* 等待发送区为空 */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	
	/* 打开DMA通道 */
	DMA_Cmd(DMA2_Stream2, ENABLE);
	DMA_Cmd(DMA2_Stream5, ENABLE);
	
	/* 传输完成 */
	while( DMA_GetFlagStatus(DMA2_Stream5, DMA_FLAG_TCIF5) == RESET);
	while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
	
	/* 关闭DMA通道 */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream5, DISABLE);	
	
	/* 清除DMA传输完成标志 */
	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
	DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);	
}


void spi2_trans_write(uint8_t *rx_data,
					 uint8_t *tx_buffer,
					 uint16_t length)
{
	/* 关闭DMA通道 */
	DMA_Cmd(DMA1_Stream3, DISABLE);
	DMA_Cmd(DMA1_Stream4, DISABLE);
	
	/* 设置传输字节数 */
	DMA_SetCurrDataCounter(DMA1_Stream3, (uint16_t)length);
	DMA_SetCurrDataCounter(DMA1_Stream4, (uint16_t)length);
	
	/* 接收DMA流的接收地址不自增 */
	DMA1_Stream3->CR &= ~(1 << 10);
	
	/* 设置接收和发送的内存地址 */
	DMA1_Stream3->M0AR = (uint32_t)rx_data;
	DMA1_Stream4->M0AR = (uint32_t)tx_buffer;

	/* 读取一次DR，使其清空 */
	SPI2->DR;
	
	/* 等待发送区为空 */
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	
	/* 打开DMA通道 */
	DMA_Cmd(DMA1_Stream3, ENABLE);
	DMA_Cmd(DMA1_Stream4, ENABLE);
	
	/* 传输完成 */
	while( DMA_GetFlagStatus(DMA1_Stream3, DMA_FLAG_TCIF3) == RESET);
	while( DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4) == RESET);
	
	/* 关闭DMA通道 */
	DMA_Cmd(DMA1_Stream3, DISABLE);
	DMA_Cmd(DMA1_Stream4, DISABLE);	
	
	/* 清除DMA传输完成标志 */
	DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
	DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);	
}

void spi3_trans_write(uint8_t *rx_data,
					 uint8_t *tx_buffer,
					 uint16_t length)
{
		/* 关闭DMA通道 */
	DMA_Cmd(DMA1_Stream0, DISABLE);
	DMA_Cmd(DMA1_Stream5, DISABLE);
	
	/* 设置传输字节数 */
	DMA_SetCurrDataCounter(DMA1_Stream0, (uint16_t)length);
	DMA_SetCurrDataCounter(DMA1_Stream5, (uint16_t)length);
	
	/* 接收DMA流的接收地址不自增 */
	DMA1_Stream0->CR &= ~(1 << 10);
	
	/* 设置接收和发送的内存地址 */
	DMA1_Stream0->M0AR = (uint32_t)rx_data;
	DMA1_Stream5->M0AR = (uint32_t)tx_buffer;

	/* 读取一次DR，使其清空 */
	SPI3->DR;
	
	/* 等待发送区为空 */
	while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
	
	/* 打开DMA通道 */
	DMA_Cmd(DMA1_Stream0, ENABLE);
	DMA_Cmd(DMA1_Stream5, ENABLE);
	
	/* 传输完成 */
	while( DMA_GetFlagStatus(DMA1_Stream0, DMA_FLAG_TCIF0) == RESET);
	while( DMA_GetFlagStatus(DMA1_Stream5, DMA_FLAG_TCIF5) == RESET);
	
	/* 关闭DMA通道 */
	DMA_Cmd(DMA1_Stream0, DISABLE);
	DMA_Cmd(DMA1_Stream5, DISABLE);	
	
	/* 清除DMA传输完成标志 */
	DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);
	DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);	
}


