/*
*********************************************************************************************************
*                                                �����Ƽ�
*	ģ������ : ADS1256 ����ģ��(8ͨ����PGA��24λADC)
*	�ļ����� : bsp_ads1256.h
*
*********************************************************************************************************
*/
/*
    ADS1256ģ��    STM32������
      +5V   <------  5.0V      5V����
      GND   -------  GND       ��

      DRDY  ------>  PC3       ׼������
      CS    <------  PC12      SPI_CS
      DIN   <------  PC10      SPI_MOSI
      DOUT  ------>  PC9       SPI_MISO
      SCLK  <------  PC11      SPIʱ��
      GND   -------  GND       ��
      PDWN  <------  PC8       ������� ����
      RST   <------  PC13      ��λ�ź� ����
      NC   �ս�
      NC   �ս�
*/

#ifndef _BSP_ADS1256_H
#define _BSP_ADS1256_H
#include "system.h"
#include "includes.h"
	#define RCC_CS 		RCC_AHB1Periph_GPIOA
	#define PORT_CS		GPIOA
	#define PIN_CS		GPIO_Pin_1

	#define RCC_DRDY 	RCC_AHB1Periph_GPIOE
	#define PORT_DRDY	GPIOE
	#define PIN_DRDY	GPIO_Pin_2

	#define RCC_SYNC 	RCC_AHB1Periph_GPIOA
	#define PORT_SYNC	GPIOA
	#define PIN_SYNC	GPIO_Pin_5
	
	#define RCC_RST 	RCC_AHB1Periph_GPIOA
	#define PORT_RST	GPIOA
	#define PIN_RST	  GPIO_Pin_4
//RST->HIGH	PC13
//SYNC->HIGH PC8
	/* ���������0����1�ĺ� */
	#define CS_0()		GPIO_ResetBits(PORT_CS, PIN_CS)
	#define CS_1()		GPIO_SetBits(PORT_CS, PIN_CS)
	
	#define SYNC_0()		GPIO_ResetBits(PORT_SYNC, PIN_SYNC)
	#define SYNC_1()		GPIO_SetBits(PORT_SYNC, PIN_SYNC)
	
	#define RST_0()		GPIO_ResetBits(PORT_RST, PIN_RST)
	#define RST_1()		GPIO_SetBits(PORT_RST, PIN_RST)

	#define DO_IS_HIGH()	(GPIO_ReadInputDataBit(PORT_DOUT, PIN_DOUT) == Bit_SET)

	#define DRDY_IS_LOW()	(GPIO_ReadInputDataBit(PORT_DRDY, PIN_DRDY) == Bit_RESET)

/* ����ѡ�� */
typedef enum
{
	ADS1256_GAIN_1			= (0),	/* ����1��ȱʡ�� */
	ADS1256_GAIN_2			= (1),	/* ����2 */
	ADS1256_GAIN_4			= (2),	/* ����4 */
	ADS1256_GAIN_8			= (3),	/* ����8 */
	ADS1256_GAIN_16			= (4),	/* ����16 */
	ADS1256_GAIN_32			= (5),	/* ����32 */
	ADS1256_GAIN_64			= (6),	/* ����64 */
}ADS1256_GAIN_E;

/* ��������ѡ�� */
/* ����ת����ѡ��
	11110000 = 30,000SPS (default)
	11100000 = 15,000SPS
	11010000 = 7,500SPS
	11000000 = 3,750SPS
	10110000 = 2,000SPS
	10100001 = 1,000SPS
	10010010 = 500SPS
	10000010 = 100SPS
	01110010 = 60SPS
	01100011 = 50SPS
	01010011 = 30SPS
	01000011 = 25SPS
	00110011 = 15SPS
	00100011 = 10SPS
	00010011 = 5SPS
	00000011 = 2.5SPS
*/
typedef enum
{
	ADS1256_30000SPS = 0,
	ADS1256_15000SPS,
	ADS1256_7500SPS,
	ADS1256_3750SPS,
	ADS1256_2000SPS,
	ADS1256_1000SPS,
	ADS1256_500SPS,
	ADS1256_100SPS,
	ADS1256_60SPS,
	ADS1256_50SPS,
	ADS1256_30SPS,
	ADS1256_25SPS,
	ADS1256_15SPS,
	ADS1256_10SPS,
	ADS1256_5SPS,
	ADS1256_2d5SPS,

	ADS1256_DRATE_MAX
}ADS1256_DRATE_E;

#define ADS1256_DRAE_COUNT = 15;

typedef struct
{
	ADS1256_GAIN_E Gain;		/* ���� */
	ADS1256_DRATE_E DataRate;	/* ����������� */
	int AdcNow[8];			/* 8·ADC�ɼ������ʵʱ���з����� */
	u8 Channel;			/* ��ǰͨ�� */
	u8 ScanMode;			/* ɨ��ģʽ��0��ʾ����8·�� 1��ʾ���4· */
}ADS1256_VAR_T;

void bsp_InitADS1256(void);
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate);

void ADS1256_Calib(void);
void ADS1256_CheckReg(u8 *a);
u8 ADS1256_ReadChipID(void);
void ADS1256_StartScan(uint8_t _ucScanMode);
void ADS1256_StopScan(void);
int32_t ADS1256_GetAdc(uint8_t _ch);
void ADS1256_ISR(void);
extern ADS1256_VAR_T g_tADS1256;

//�������
__packed typedef struct  
{
	u8 day;
	u8 hour;
	u8 min;
	u8 sec;
	u16 ms;
	float ch[3];
}queue; 

#define QSIZE  250
extern queue qdata[QSIZE];
extern u8 qfront;
extern u8 qcount;
extern u8 qrear;

#endif

