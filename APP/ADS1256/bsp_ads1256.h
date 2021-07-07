/*
*********************************************************************************************************
*                                                康威科技
*	模块名称 : ADS1256 驱动模块(8通道带PGA的24位ADC)
*	文件名称 : bsp_ads1256.h
*
*********************************************************************************************************
*/
/*
    ADS1256模块    STM32开发板
      +5V   <------  5.0V      5V供电
      GND   -------  GND       地

      DRDY  ------>  PC3       准备就绪
      CS    <------  PC12      SPI_CS
      DIN   <------  PC10      SPI_MOSI
      DOUT  ------>  PC9       SPI_MISO
      SCLK  <------  PC11      SPI时钟
      GND   -------  GND       地
      PDWN  <------  PC8       掉电控制 常高
      RST   <------  PC13      复位信号 常高
      NC   空脚
      NC   空脚
*/

#ifndef _BSP_ADS1256_H
#define _BSP_ADS1256_H
#include "system.h"
#include "includes.h"

#define CH0_ENABLE 1
#define CH1_ENABLE 1
#define CH2_ENABLE 1

	#define RCC_CS0 	RCC_AHB1Periph_GPIOG
	#define PORT_CS0	GPIOG
	#define PIN_CS0		GPIO_Pin_2
	
	#define RCC_CS1 	RCC_AHB1Periph_GPIOG
	#define PORT_CS1	GPIOG
	#define PIN_CS1		GPIO_Pin_3
	
	#define RCC_CS2 	RCC_AHB1Periph_GPIOG
	#define PORT_CS2	GPIOG
	#define PIN_CS2		GPIO_Pin_4

	#define RCC_DRDY0 	RCC_AHB1Periph_GPIOE
	#define PORT_DRDY0	GPIOE
	#define PIN_DRDY0	  GPIO_Pin_2
	
	#define RCC_DRDY1 	RCC_AHB1Periph_GPIOG
	#define PORT_DRDY1	GPIOG
	#define PIN_DRDY1	  GPIO_Pin_5
	
	#define RCC_DRDY2 	RCC_AHB1Periph_GPIOG
	#define PORT_DRDY2	GPIOG
	#define PIN_DRDY2	  GPIO_Pin_6

	#define RCC_SYNC 	RCC_AHB1Periph_GPIOG
	#define PORT_SYNC	GPIOG
	#define PIN_SYNC	GPIO_Pin_7
	
	#define RCC_RST 	RCC_AHB1Periph_GPIOG
	#define PORT_RST	GPIOG
	#define PIN_RST	  GPIO_Pin_8
//RST->HIGH	PC13
//SYNC->HIGH PC8
	/* 定义口线置0和置1的宏 */
	#define CS0_0()		GPIO_ResetBits(PORT_CS0, PIN_CS0)
	#define CS0_1()		GPIO_SetBits(PORT_CS0, PIN_CS0)
	
	#define CS1_0()		GPIO_ResetBits(PORT_CS1, PIN_CS1)
	#define CS1_1()		GPIO_SetBits(PORT_CS1, PIN_CS1)
	
	#define CS2_0()		GPIO_ResetBits(PORT_CS2, PIN_CS2)
	#define CS2_1()		GPIO_SetBits(PORT_CS2, PIN_CS2)
	
	#define SYNC_0()		GPIO_ResetBits(PORT_SYNC, PIN_SYNC)
	#define SYNC_1()		GPIO_SetBits(PORT_SYNC, PIN_SYNC)
	
	#define RST_0()		GPIO_ResetBits(PORT_RST, PIN_RST)
	#define RST_1()		GPIO_SetBits(PORT_RST, PIN_RST)

	#define DRDY0_IS_LOW()	(GPIO_ReadInputDataBit(PORT_DRDY0, PIN_DRDY0) == Bit_RESET)
	#define DRDY1_IS_LOW()	(GPIO_ReadInputDataBit(PORT_DRDY1, PIN_DRDY1) == Bit_RESET)
	#define DRDY2_IS_LOW()	(GPIO_ReadInputDataBit(PORT_DRDY2, PIN_DRDY2) == Bit_RESET)

/* 增益选项 */
typedef enum
{
	ADS1256_GAIN_1			= (0),	/* 增益1（缺省） */
	ADS1256_GAIN_2			= (1),	/* 增益2 */
	ADS1256_GAIN_4			= (2),	/* 增益4 */
	ADS1256_GAIN_8			= (3),	/* 增益8 */
	ADS1256_GAIN_16			= (4),	/* 增益16 */
	ADS1256_GAIN_32			= (5),	/* 增益32 */
	ADS1256_GAIN_64			= (6),	/* 增益64 */
}ADS1256_GAIN_E;

/* 采样速率选项 */
/* 数据转换率选择
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
	ADS1256_GAIN_E Gain;		/* 增益 */
	ADS1256_DRATE_E DataRate;	/* 数据输出速率 */
	int AdcNow[8];			/* 8路ADC采集结果（实时）有符号数 */
	u8 Channel;			/* 当前通道 */
	u8 ScanMode;			/* 扫描模式，0表示单端8路， 1表示差分4路 */
}ADS1256_VAR_T;

typedef struct
{
	u32 Ach0;
	u32 Ach1;
	u32 Ach2;
}Correct_Array;

void bsp_InitADS1256(void);
void ADS1256_CfgADC(u8 device, ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate);

void ADS1256_Calib(u8 device);
void ADS1256_CheckReg(u8 device, u8 *a);
u8 ADS1256_ReadChipID(u8 device);
void ADS1256_StartScan(uint8_t _ucScanMode);
void ADS1256_StopScan(void);
int32_t ADS1256_GetAdc(uint8_t _ch);
void ADS1256_ISR(void);
extern ADS1256_VAR_T g_tADS1256;

//缓存队列
__packed typedef struct  
{
	u8 day;
	u8 hour;
	u8 min;
	u8 sec;
	u16 ms;
	float ch[3];
}queue; 

#define QSIZE  500
extern queue qdata[QSIZE];
extern u8 qfront;
extern u8 qcount;
extern u8 qrear;

#endif

