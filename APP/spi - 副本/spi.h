#ifndef _spi_H
#define _spi_H

#include "system.h"

void SPI1_Init(void);			 //初始化SPI1口
void SPI1_SetSpeed(u8 SpeedSet); //设置SPI1速度   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI1总线读写一个字节
void SPI1_SendArray(const u8 TxData[], u8 *RxData, u8 len); //发送一个数组
void SPI1_WriteBytes(const u8 *TxBuffer,u8 TxLenth);
#endif
