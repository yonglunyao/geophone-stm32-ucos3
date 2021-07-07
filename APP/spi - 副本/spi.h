#ifndef _spi_H
#define _spi_H

#include "system.h"

void SPI1_Init(void);			 //��ʼ��SPI1��
void SPI1_SetSpeed(u8 SpeedSet); //����SPI1�ٶ�   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI1���߶�дһ���ֽ�
void SPI1_SendArray(const u8 TxData[], u8 *RxData, u8 len); //����һ������
void SPI1_WriteBytes(const u8 *TxBuffer,u8 TxLenth);
#endif
