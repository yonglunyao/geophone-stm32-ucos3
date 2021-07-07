#ifndef __usart_H
#define __usart_H

#include "system.h"
#include "stdio.h"


#define USART2_MAX_RECV_LEN		50				//最大接收缓存字节数
#define USART2_MAX_SEND_LEN		500				//最大发送缓存字节数
#define USART2_RX_EN 			1					//0,不接收;1,接收.

#define USART3_REC_LEN  800

#define USART6_MAX_RECV_LEN		50				//最大接收缓存字节数
#define USART6_MAX_SEND_LEN		300				//最大发送缓存字节数
#define USART6_RX_EN 			0					//0,不接收;1,接收.

void USART1_Init(u32 bound);
void USART2_Init(u32 bound);				//串口2初始化 
void USART3_Init(u32 bound);
void USART6_Init(u32 bound);

extern u8  USART2_RX_BUF[USART2_MAX_RECV_LEN]; 		//接收缓冲,最大USART2_MAX_RECV_LEN字节
extern u8  USART2_TX_BUF[USART2_MAX_SEND_LEN]; 		//发送缓冲,最大USART2_MAX_SEND_LEN字节
extern u16 USART2_RX_STA;   						//接收数据状态
extern u8  USART2_RX_REC_ATCOMMAD;

extern u8 USART3_TX_BUF[1]; 
extern u8 USART3_RX_BUF[USART3_REC_LEN];     //????,??USART_REC_LEN???.
extern u16 USART3_RX_STA;

extern u8  USART6_RX_BUF[USART6_MAX_RECV_LEN]; 		//接收缓冲,最大USART2_MAX_RECV_LEN字节
extern u8  USART6_TX_BUF[USART6_MAX_SEND_LEN]; 		//发送缓冲,最大USART2_MAX_SEND_LEN字节
extern u16 USART6_RX_STA;   						//接收数据状态
extern volatile u8  USART6_RX_REC_ATCOMMAD;

void u2_printf(char* fmt, ...);
void SendUsart2TXBUF(u16 len);

void u6_printf(char* fmt, ...);
void SendUsart6TXBUF(u16 len);


void TIM3_Set(u8 sta);
void TIM3_Init(u16 arr, u16 psc);
void TIM4_Set(u8 sta);
void TIM4_Init(u16 arr, u16 psc);
void TIM7_Int_Init(u16 arr, u16 psc);

#endif




