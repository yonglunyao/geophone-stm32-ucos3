#ifndef __usart_H
#define __usart_H

#include "system.h"
#include "stdio.h"


#define USART2_MAX_RECV_LEN		50				//�����ջ����ֽ���
#define USART2_MAX_SEND_LEN		500				//����ͻ����ֽ���
#define USART2_RX_EN 			1					//0,������;1,����.

#define USART3_REC_LEN  800

#define USART6_MAX_RECV_LEN		50				//�����ջ����ֽ���
#define USART6_MAX_SEND_LEN		300				//����ͻ����ֽ���
#define USART6_RX_EN 			0					//0,������;1,����.

void USART1_Init(u32 bound);
void USART2_Init(u32 bound);				//����2��ʼ�� 
void USART3_Init(u32 bound);
void USART6_Init(u32 bound);

extern u8  USART2_RX_BUF[USART2_MAX_RECV_LEN]; 		//���ջ���,���USART2_MAX_RECV_LEN�ֽ�
extern u8  USART2_TX_BUF[USART2_MAX_SEND_LEN]; 		//���ͻ���,���USART2_MAX_SEND_LEN�ֽ�
extern u16 USART2_RX_STA;   						//��������״̬
extern u8  USART2_RX_REC_ATCOMMAD;

extern u8 USART3_TX_BUF[1]; 
extern u8 USART3_RX_BUF[USART3_REC_LEN];     //????,??USART_REC_LEN???.
extern u16 USART3_RX_STA;

extern u8  USART6_RX_BUF[USART6_MAX_RECV_LEN]; 		//���ջ���,���USART2_MAX_RECV_LEN�ֽ�
extern u8  USART6_TX_BUF[USART6_MAX_SEND_LEN]; 		//���ͻ���,���USART2_MAX_SEND_LEN�ֽ�
extern u16 USART6_RX_STA;   						//��������״̬
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




