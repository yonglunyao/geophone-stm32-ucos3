#ifndef _dma_H
#define _dma_H

#include "system.h"
#define SPI1_DR_ADDR ((u32)0x4001300C)

#define RX_LEN 			(uint8_t)20
#define TX_LEN      (uint8_t)20

extern u8 SPI_RX_BUFFER[RX_LEN];
extern u8 SPI_TX_BUFFER[TX_LEN]; 
void DMAx_Init(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr);//配置DMAx_CHx
void DMAx_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);	//使能一次DMA传输
void spi_dma_init(void);
void spi_trans(u8 *rx_buf, u8 *tx_buf,u16 length);
void spi_trans_read(u8 *rx_buf,u8 *tx_data,u16 length);
void spi_trans_write(u8 *rx_data,u8 *tx_buffer, u16 length);
#endif
