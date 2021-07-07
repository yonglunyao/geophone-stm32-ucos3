#ifndef _dma_H
#define _dma_H

#include "system.h"
#define SPI1_DR_ADDR   (u32)&SPI1->DR   
#define SPI2_DR_ADDR   (u32)&SPI2->DR   
#define SPI3_DR_ADDR   (u32)&SPI3->DR     
#define SPI1_RX_LEN    (u8)30
#define SPI1_TX_LEN    (u8)30
#define SPI2_RX_LEN    (u8)30
#define SPI2_TX_LEN    (u8)30
#define SPI3_RX_LEN    (u8)30
#define SPI3_TX_LEN    (u8)30

extern u8 SPI1_RX_BUFFER[SPI1_RX_LEN];
extern u8 SPI1_TX_BUFFER[SPI1_TX_LEN]; 
extern u8 SPI2_RX_BUFFER[SPI2_RX_LEN];
extern u8 SPI2_TX_BUFFER[SPI2_TX_LEN]; 
extern u8 SPI3_RX_BUFFER[SPI3_RX_LEN];
extern u8 SPI3_TX_BUFFER[SPI3_TX_LEN]; 

void DMAx_Init(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr);//配置DMAx_CHx
void DMAx_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);	//使能一次DMA传输
void spi2_dma_init(void);
void spi2_trans(u8 *rx_buf, u8 *tx_buf,u16 length);
void spi2_trans_read(u8 *rx_buf,u8 *tx_data,u16 length);
void spi2_trans_write(u8 *rx_data,u8 *tx_buffer, u16 length);
void spi1_dma_init(void);
void spi1_trans(u8 *rx_buf, u8 *tx_buf,u16 length);
void spi1_trans_read(u8 *rx_buf,u8 *tx_data,u16 length);
void spi1_trans_write(u8 *rx_data,u8 *tx_buffer, u16 length);
void spi3_dma_init(void);
void spi3_trans(u8 *rx_buf, u8 *tx_buf,u16 length);
void spi3_trans_read(u8 *rx_buf,u8 *tx_data,u16 length);
void spi3_trans_write(u8 *rx_data,u8 *tx_buffer, u16 length);
#endif
