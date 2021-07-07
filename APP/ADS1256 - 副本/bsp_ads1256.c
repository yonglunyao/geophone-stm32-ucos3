/*
*********************************************************************************************************
*                                              �����Ƽ�
*	ģ������ : ADS1256 ����ģ��(8ͨ����PGA��24λADC)
*	�ļ����� : bsp_ads1256.c
*	��    �� : V1.0
*	˵    �� : ADS1256ģ���CPU֮�����SPI�ӿڡ�
*			 
*
*
*********************************************************************************************************
*/
#include "system.h"
#include "stdio.h"
#include "bsp_ads1256.h"
#include "SysTick.h"
#include "spi.h"
#include "dma.h"

#define HARD_SPI	1	/* ������б�ʾʹ��GPIOģ��SPI�ӿ� */

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

/*
	ADS1256��������:
	1��ģ�ⲿ�ֹ���5V;
	2��SPI���ֽӿڵ�ƽ��3.3V
	3��PGA���÷�Χ�� 1��2��4��8��16��32��64��
	4���ο���ѹ2.5V (�Ƽ�ȱʡ�ģ����õģ�
	5�������ѹ��Χ��PGA = 1 ʱ, ����������5V
	6. �Զ�У׼ ����������PGA,BUFʹ�ܡ����ݲ�����ʱ����������У׼)
	7. ����Ļ��������������ú͹رգ�һ��ѡ���ã�


	�ⲿ����Ƶ�� = 7.68MHz,
		ʱ��Ƶ�� tCLK = 1/7.68M = 0.13uS
		����������� tDATA =  1 / 30K = 0.033mS  (��30Ksps����)

	��SPI��ʱ���ٶ�Ҫ��: (ads1256.pdf page 6)
		��� 4��tCLK = 0.52uS
		���� 10��tDATA = 0.3mS (�� 30Ksps ����)

		SCL�ߵ�ƽ�͵͵�ƽ����ʱ����С 200ns

	RREG, WREG, RDATA ����֮����Ҫ�ӳ� 4 * tCLK = 0.52uS;
	RDATAC, RESET, SYNC ����֮����Ҫ�ӳ� 24 * tCLK = 3.12uS;

	ʵ�ʲ��ԣ���3.3V�ϵ��, ��ʱ�����κ����ã�ADS125��DRDY ���߼���ʼ��������źţ�2.6us��,33.4�ͣ�Ƶ��30KHz��
*/

/*
	���Լ�¼
	(1) ���üĴ���ʱ��SCK���쵼��оƬ����ÿ�ζ��յ����ݡ�ԭ��: ���͵����ڵ��ֽ�֮����Ҫ�ӳ�һС��ʱ��.
	(2) ������λCPUʱ��ż������оƬ����������쳣��
*/

#if !defined(SOFT_SPI) && !defined(HARD_SPI)
 	#error "Please define SPI Interface mode : SOFT_SPI or HARD_SPI"
#endif



#ifdef HARD_SPI		/* Ӳ��SPI */
	;
#endif

/* �Ĵ������壺 Table 23. Register Map --- ADS1256�����ֲ��30ҳ */
enum
{
	/* �Ĵ�����ַ�� �����Ǹ�λ��ȱʡֵ */
	REG_STATUS = 0,	// x1H
	REG_MUX    = 1, // 01H
	REG_ADCON  = 2, // 20H
	REG_DRATE  = 3, // F0H
	REG_IO     = 4, // E0H
	REG_OFC0   = 5, // xxH
	REG_OFC1   = 6, // xxH
	REG_OFC2   = 7, // xxH
	REG_FSC0   = 8, // xxH
	REG_FSC1   = 9, // xxH
	REG_FSC2   = 10, // xxH
};

/* ����壺 TTable 24. Command Definitions --- ADS1256�����ֲ��34ҳ */
enum
{
	CMD_WAKEUP  = 0x00,	// Completes SYNC and Exits Standby Mode 0000  0000 (00h)
	CMD_RDATA   = 0x01, // Read Data 0000  0001 (01h)
	CMD_RDATAC  = 0x03, // Read Data Continuously 0000   0011 (03h)
	CMD_SDATAC  = 0x0F, // Stop Read Data Continuously 0000   1111 (0Fh)
	CMD_RREG    = 0x10, // Read from REG rrr 0001 rrrr (1xh)
	CMD_WREG    = 0x50, // Write to REG rrr 0101 rrrr (5xh)
	CMD_SELFCAL = 0xF0, // Offset and Gain Self-Calibration 1111    0000 (F0h)
	CMD_SELFOCAL= 0xF1, // Offset Self-Calibration 1111    0001 (F1h)
	CMD_SELFGCAL= 0xF2, // Gain Self-Calibration 1111    0010 (F2h)
	CMD_SYSOCAL = 0xF3, // System Offset Calibration 1111   0011 (F3h)
	CMD_SYSGCAL = 0xF4, // System Gain Calibration 1111    0100 (F4h)
	CMD_SYNC    = 0xFC, // Synchronize the A/D Conversion 1111   1100 (FCh)
	CMD_STANDBY = 0xFD, // Begin Standby Mode 1111   1101 (FDh)
	CMD_RESET   = 0xFE, // Reset to Power-Up Values 1111   1110 (FEh)
};

static u8 ADS1256_Send8Bit(u8 _data);
static u8 ADS1256_Recive8Bit(void);
static void ADS1256_WaitDRDY(void);
static void ADS1256_ResetHard(void);
static void ADS1256_DelaySCLK(void);
static void ADS1256_DelayDATA(void);

static void ADS1256_WriteCmd(u8 _cmd);
static void ADS1256_WriteReg(u8 _RegID, u8 _RegValue);
static u8 ADS1256_ReadReg(u8 _RegID);
static int32_t ADS1256_ReadData(void);
static void ADS1256_SetChannal(u8 _ch);
//static void ADS1256_SetDiffChannal(u8 _ch);

//�������
queue qdata[QSIZE];
u8 qfront;
u8 qcount;
u8 qrear;

ADS1256_VAR_T g_tADS1256;
static const u8 s_tabDataRate[ADS1256_DRATE_MAX] =
{
	0xF0,		/* ��λʱȱʡֵ */
	0xE0,
	0xD0,
	0xC0,
	0xB0,
	0xA1,
	0x92,
	0x82,
	0x72,
	0x63,
	0x53,
	0x43,
	0x33,
	0x20,
	0x13,
	0x03
};
void bsp_DelayUS(u16 nus)
{
	delay_us(nus);
}
/*
*********************************************************************************************************
*	�� �� ��: bsp_InitADS1256
*	����˵��: ����STM32��GPIO��SPI�ӿڣ��������� ADS1256
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitADS1256(void)
{
	SPI1_Init();
	//�������ÿ�
	qfront=0;
	qcount=0;
	qrear=0;
	
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_CS | RCC_DRDY | RCC_RST | RCC_SYNC, ENABLE);

	/* ���ü����������IO */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	/* ��Ϊ��������� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	/* IO������ٶ� */
	GPIO_InitStructure.GPIO_Pin = PIN_CS | PIN_SYNC | PIN_RST;
	GPIO_Init(PORT_CS, &GPIO_InitStructure);
	
	/* ����GPIOΪ��������ģʽ(ʵ����CPU��λ���������״̬) */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = PIN_DRDY;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(PORT_DRDY, &GPIO_InitStructure);
	
////	RST_0();
////	delay_ms(1);
	SYNC_1();
	RST_1();
////	delay_ms(100);
	ADS1256_ResetHard();		/* Ӳ����λ */

	ADS1256_WaitDRDY();

	ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_500SPS);	/* ����ADC������ ����1:1, ����������� 1KHz */
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_CfgADC
*	����˵��: ����ADC����������������������
*	��    ��: _gain : ����
*			  _drate : �����������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate)
{
	g_tADS1256.Gain = _gain;
	g_tADS1256.DataRate = _drate;

	ADS1256_StopScan();			/* ��ͣCPU�ж� */

	ADS1256_ResetHard();		/* Ӳ����λ */

	ADS1256_WaitDRDY();

	{
		u8 buf[4];		/* �ݴ�ADS1256 �Ĵ������ò�����֮������д4���Ĵ��� */

		/* ״̬�Ĵ�������
			Bits 7-4 ID3, ID2, ID1, ID0  Factory Programmed Identification Bits (Read Only)

			Bit 3 ORDER: Data Output Bit Order
				0 = Most Significant Bit First (default)
				1 = Least Significant Bit First
			Input data  is always shifted in most significant byte and bit first. Output data is always shifted out most significant
			byte first. The ORDER bit only controls the bit order of the output data within the byte.

			Bit 2 ACAL : Auto-Calibration
				0 = Auto-Calibration Disabled (default)
				1 = Auto-Calibration Enabled
			When Auto-Calibration is enabled, self-calibration begins at the completion of the WREG command that changes
			the PGA (bits 0-2 of ADCON register), DR (bits 7-0 in the DRATE register) or BUFEN (bit 1 in the STATUS register)
			values.

			Bit 1 BUFEN: Analog Input Buffer Enable
				0 = Buffer Disabled (default)
				1 = Buffer Enabled

			Bit 0 DRDY :  Data Ready (Read Only)
				This bit duplicates the state of the DRDY pin.

			ACAL=1ʹ����У׼���ܡ��� PGA��BUFEEN, DRATE�ı�ʱ��������У׼
		*/
		buf[0] = (0 << 3) | (1 << 2) | (1 << 1);
		//ADS1256_WriteReg(REG_STATUS, (0 << 3) | (1 << 2) | (1 << 1));

		buf[1] = 0x08;	/* ����λ0��ʾAINP�� AIN0,  ����λ8��ʾ AINN �̶��� AINCOM */

		/*	ADCON: A/D Control Register (Address 02h)
			Bit 7 Reserved, always 0 (Read Only)
			Bits 6-5 CLK1, CLK0 : D0/CLKOUT Clock Out Rate Setting
				00 = Clock Out OFF
				01 = Clock Out Frequency = fCLKIN (default)
				10 = Clock Out Frequency = fCLKIN/2
				11 = Clock Out Frequency = fCLKIN/4
				When not using CLKOUT, it is recommended that it be turned off. These bits can only be reset using the RESET pin.

			Bits 4-2 SDCS1, SCDS0: Sensor Detect Current Sources
				00 = Sensor Detect OFF (default)
				01 = Sensor Detect Current = 0.5 �� A
				10 = Sensor Detect Current = 2 �� A
				11 = Sensor Detect Current = 10�� A
				The Sensor Detect Current Sources can be activated to verify  the integrity of an external sensor supplying a signal to the
				ADS1255/6. A shorted sensor produces a very small signal while an open-circuit sensor produces a very large signal.

			Bits 2-0 PGA2, PGA1, PGA0: Programmable Gain Amplifier Setting
				000 = 1 (default)
				001 = 2
				010 = 4
				011 = 8
				100 = 16
				101 = 32
				110 = 64
				111 = 64
		*/
		buf[2] = (0 << 5) | (0 << 3) | (_gain << 0);
		//ADS1256_WriteReg(REG_ADCON, (0 << 5) | (0 << 2) | (GAIN_1 << 0));	/* ѡ��1;1����, ��������5V */

		/* ��Ϊ�л�ͨ���Ͷ����ݺ�ʱ 123uS, ���ɨ���ж�ģʽ����ʱ��������� = DRATE_1000SPS */
		buf[3] = s_tabDataRate[_drate];	// DRATE_10SPS;	/* ѡ������������� */

		CS_0();	/* SPIƬѡ = 0 */
		ADS1256_Send8Bit(CMD_WREG | 0);	/* д�Ĵ���������, �����ͼĴ�����ַ */
		ADS1256_Send8Bit(0x03);			/* �Ĵ������� - 1, �˴�3��ʾд4���Ĵ��� */

		ADS1256_Send8Bit(buf[0]);	/* ����״̬�Ĵ��� */
		ADS1256_Send8Bit(buf[1]);	/* ��������ͨ������ */
		ADS1256_Send8Bit(buf[2]);	/* ����ADCON���ƼĴ��������� */
		ADS1256_Send8Bit(buf[3]);	/* ��������������� */

		CS_1();	/* SPIƬѡ = 1 */
	}

	bsp_DelayUS(50);
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_DelaySCLK
*	����˵��: CLK֮����ӳ٣�ʱ���ӳ�. ����STM32F407  168M��Ƶ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_DelaySCLK(void)
{
	uint16_t i;

	/*  for STM32F407 (168M��Ƶ)
		ȡ 5 ʱ��ʵ��ߵ�ƽ200ns, �͵�ƽ250ns <-- ���ȶ�
		ȡ 10 ���ϣ��������������� �͵�ƽ400ns �߶�400ns <--- �ȶ�
	*/
	for (i = 0; i < 9; i++);
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_DelayDATA
*	����˵��: ��ȡDOUT֮ǰ���ӳ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_DelayDATA(void)
{
	/*
		Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands
		��С 50 ��tCLK = 50 * 0.13uS = 6.5uS
	*/
	bsp_DelayUS(8);	/* ��С�ӳ� 6.5uS, �˴�ȡ10us */
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_ResetHard
*	����˵��: Ӳ����λ ADS1256оƬ.�͵�ƽ��λ�����4��ʱ�ӣ�Ҳ���� 4x0.13uS = 0.52uS
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_ResetHard(void)
{
	/* ADS1256�����ֲ��7ҳ */
	//RST_0();			/* ��λ */
	//delay_ms(150);
	//RST_1();
	//SYNC_0();			/* ������� ͬ��*/
	//delay_ms(150);
	//SYNC_1();			/* �˳����� */

	//delay_ms(600);

	ADS1256_WaitDRDY();	/* �ȴ� DRDY��Ϊ0, �˹���ʵ��: 630us */
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_Send8Bit
*	����˵��: ��SPI���߷���8��bit���ݡ� ����CS���ơ�
*	��    ��: _data : ����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static u8 ADS1256_Send8Bit(u8 _data)
{
	ADS1256_DelaySCLK();
	return SPI1_ReadWriteByte(_data);
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_Recive8Bit
*	����˵��: ��SPI���߽���8��bit���ݡ� ����CS���ơ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static u8 ADS1256_Recive8Bit(void)
{
	u8 rx, tx = 0;
	ADS1256_DelaySCLK();
	spi_trans_read(&rx, &tx, 1); //DMA��ʽ
	return rx;
	//return SPI1_ReadWriteByte(0);
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_WriteReg
*	����˵��: дָ���ļĴ���
*	��    ��:  _RegID : �Ĵ���ID
*			  _RegValue : �Ĵ���ֵ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_WriteReg(u8 _RegID, u8 _RegValue)
{
	CS_0();	/* SPIƬѡ = 0 */
	ADS1256_Send8Bit(CMD_WREG | _RegID);	/* д�Ĵ���������, �����ͼĴ�����ַ */
	ADS1256_Send8Bit(0x00);		/* �Ĵ������� - 1, �˴�д1���Ĵ��� */
  ADS1256_DelaySCLK();
	ADS1256_Send8Bit(_RegValue);	/* ���ͼĴ���ֵ */
	CS_1();	/* SPIƬѡ = 1 */
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_ReadReg
*	����˵��: дָ���ļĴ���
*	��    ��:  _RegID : �Ĵ���ID
*			  _RegValue : �Ĵ���ֵ��
*	�� �� ֵ: �����ļĴ���ֵ��
*********************************************************************************************************
*/
static u8 ADS1256_ReadReg(u8 _RegID)
{
	u8 read;

	CS_0();	/* SPIƬѡ = 0 */
	u8 tmp1 = ADS1256_Send8Bit(CMD_RREG | _RegID);	/* д�Ĵ���������, �����ͼĴ�����ַ */
	u8 tmp2 = ADS1256_Send8Bit(0x00);	/* �Ĵ������� - 1, �˴���1���Ĵ��� */

	ADS1256_DelayDATA();	/* �����ӳٲ��ܶ�ȡоƬ�������� */

	read = ADS1256_Recive8Bit();	/* ���Ĵ���ֵ */
	CS_1();	/* SPIƬѡ = 1 */

	return read;
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_WriteCmd
*	����˵��: ���͵��ֽ�����
*	��    ��:  _cmd : ����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_WriteCmd(u8 _cmd)
{
	CS_0();	/* SPIƬѡ = 0 */
	ADS1256_Send8Bit(_cmd);
	CS_1();	/* SPIƬѡ = 1 */
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_Calib
*	����˵��: У׼
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ADS1256_Calib(void)
{
	//CMD_SELFCAL 
	//CMD_SELFOCAL
	//CMD_SELFGCAL
	//CMD_SYSOCAL 
	//CMD_SYSGCAL
	ADS1256_WaitDRDY(); ADS1256_WriteCmd(CMD_SYSOCAL);
	ADS1256_WaitDRDY(); ADS1256_WriteCmd(CMD_SELFCAL);
	ADS1256_WaitDRDY(); ADS1256_WriteCmd(CMD_SELFGCAL);
	ADS1256_WaitDRDY();
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_CheckReg
*	����˵��: ��оƬID, ��״̬�Ĵ����еĸ�4bit
*	��    ��: a:���鳤������11
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ADS1256_CheckReg(u8 *a)
{
	ADS1256_WaitDRDY();
	a[0] = ADS1256_ReadReg(REG_STATUS);ADS1256_WaitDRDY();
	a[1] = ADS1256_ReadReg(REG_MUX);ADS1256_WaitDRDY();
	a[2] = ADS1256_ReadReg(REG_ADCON);ADS1256_WaitDRDY();
	a[3] = ADS1256_ReadReg(REG_DRATE);ADS1256_WaitDRDY();
	a[4] = ADS1256_ReadReg(REG_IO);ADS1256_WaitDRDY();
	a[5] = ADS1256_ReadReg(REG_OFC0);ADS1256_WaitDRDY();
	a[6] = ADS1256_ReadReg(REG_OFC1);ADS1256_WaitDRDY();
	a[7] = ADS1256_ReadReg(REG_OFC2);ADS1256_WaitDRDY();
	a[8] = ADS1256_ReadReg(REG_FSC0);ADS1256_WaitDRDY();
	a[9] = ADS1256_ReadReg(REG_FSC1);ADS1256_WaitDRDY();
	a[10] =ADS1256_ReadReg(REG_FSC2);ADS1256_WaitDRDY();
}
/*
*********************************************************************************************************
*	�� �� ��: ADS1256_ReadChipID
*	����˵��: ��оƬID, ��״̬�Ĵ����еĸ�4bit
*	��    ��: ��
*	�� �� ֵ: 8bit״̬�Ĵ���ֵ�ĸ�4λ
*********************************************************************************************************
*/
u8 ADS1256_ReadChipID(void)
{
	u8 id;

	ADS1256_WaitDRDY();
	id = ADS1256_ReadReg(REG_DRATE);
	printf("drate:%02x\r\n", id);
	
	ADS1256_WaitDRDY();
	id = ADS1256_ReadReg(REG_STATUS);
	printf("reg:%02x\r\n", id);
	
	return (id >> 4);
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_SetChannal
*	����˵��: ����ͨ���š���·���á�AIN- �̶��ӵأ�ACOM).
*	��    ��: _ch : ͨ���ţ� 0-7
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_SetChannal(u8 _ch)
{
	/*
	Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
		0000 = AIN0 (default)
		0001 = AIN1
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are ��don��t care��)

		NOTE: When using an ADS1255 make sure to only select the available inputs.

	Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
		0000 = AIN0
		0001 = AIN1 (default)
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are ��don��t care��)
	*/
	if (_ch > 7)
	{
		return;
	}
	ADS1256_WriteReg(REG_MUX, (_ch << 4) | (1 << 3));	/* Bit3 = 1, AINN �̶��� AINCOM */
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_SetDiffChannal
*	����˵��: ���ò��ͨ���š���·���á�
*	��    ��: _ch : ͨ����,0-3����4��
*	�� �� ֵ: 8bit״̬�Ĵ���ֵ�ĸ�4λ
*********************************************************************************************************
*/
static void ADS1256_SetDiffChannal(u8 _ch)
{
	/*
	Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
		0000 = AIN0 (default)
		0001 = AIN1
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are ��don��t care��)

		NOTE: When using an ADS1255 make sure to only select the available inputs.

	Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
		0000 = AIN0
		0001 = AIN1 (default)
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are ��don��t care��)
	*/
	if (_ch == 0)
	{
		ADS1256_WriteReg(REG_MUX, (0 << 4) | 1);	/* ������� AIN0�� AIN1 */
	}
	else if (_ch == 1)
	{
		ADS1256_WriteReg(REG_MUX, (2 << 4) | 3);	/* ������� AIN2�� AIN3 */
	}
	else if (_ch == 2)
	{
		ADS1256_WriteReg(REG_MUX, (4 << 4) | 5);	/* ������� AIN4�� AIN5 */
	}
	else if (_ch == 3)
	{
		ADS1256_WriteReg(REG_MUX, (6 << 4) | 7);	/* ������� AIN6�� AIN7 */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_WaitDRDY
*	����˵��: �ȴ��ڲ�������ɡ� ��У׼ʱ��ϳ�����Ҫ�ȴ���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_WaitDRDY(void)
{
	uint32_t i;

	for (i = 0; i < 40000000; i++)
	{
		if (DRDY_IS_LOW())
		{
			break;
		}
	}
	if (i >= 40000000)
	{
		printf("ADS1256_WaitDRDY() Time Out ...\r\n");		/* �������. �����Ŵ� */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_ReadData
*	����˵��: ��ADC����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static int32_t ADS1256_ReadData(void)
{
	uint32_t read = 0;
	u8 *p = (u8*)&read;
	u8 res[3] = {0, 0, 0};
	u8 zero[3] = {0, 0, 0};
	CS_0();	/* SPIƬѡ = 0 */
	
	ADS1256_Send8Bit(CMD_RDATA);	/* �����ݵ����� */

	ADS1256_DelayDATA();	/* �����ӳٲ��ܶ�ȡоƬ�������� */
	
	spi_trans_read(res, zero, 3);
	
  CS_1();	/* SPIƬѡ = 1 */
	
	/* �����������3���ֽڣ����ֽ���ǰ */
	//read = (res[0] << 16) + (res[1] << 8) + res[2];
	/*С����ֱ��д�ڴ�ķ�ʽ*/
	p[0] = res[2];
	p[1] = res[1];
	p[2] = res[0];
	

	/* ����������չ��24λ�з�������չΪ32λ�з����� */
	if (read & 0x800000)
	{
		read += 0xFF000000;
	}

	return (int32_t)read;
}


/*
*********************************************************************************************************
*	����ĺ�������DRDY�жϹ���ģʽ
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_StartScan
*	����˵��: �� DRDY���� ��PH9 �����ó��ⲿ�жϴ�����ʽ�� �жϷ��������ɨ��8��ͨ�������ݡ�
*	��    ��: _ucDiffMode : 0 ��ʾ����ģʽ��ɨ��8·���� 1��ʾ���ģʽ��ɨ��4·
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ADS1256_StartScan(u8 _ucScanMode)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	g_tADS1256.ScanMode = _ucScanMode;
	/* ��ʼɨ��ǰ, ������������ */
	{
		u8 i;

		g_tADS1256.Channel = 0;

		for (i = 0; i < 8; ++i)
		{
			g_tADS1256.AdcNow[i] = 0;
		}
	}

	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Connect EXTI3 Line to PA2 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);

	/* Configure EXTI3 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  /* �½��� */
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI3 Interrupt  priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_StopScan
*	����˵��: ֹͣ DRDY �ж�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ADS1256_StopScan(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	/* ���� EXTI LineXXX */
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	/* �½���(�ȴ� DRDY ��1��0��ʱ��) */
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;		/* ��ֹ */
	EXTI_Init(&EXTI_InitStructure);

	/* �ж����ȼ����� ������ȼ� ����һ��Ҫ�ֿ��������жϣ����ܹ��ϲ���һ���������� */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;		/* ��ֹ */
	NVIC_Init(&NVIC_InitStructure);
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_GetAdc
*	����˵��: �ӻ�������ȡADC��������������ṹ�����жϷ���������ġ�
*	��    ��: _ch ͨ���� (0 - 7)
*	�� �� ֵ: ADC�ɼ�������з�������
*********************************************************************************************************
*/
int32_t ADS1256_GetAdc(u8 _ch)
{
	int32_t iTemp;

	if (_ch > 7)
	{
		return 0;
	}

	__set_PRIMASK(1);	/* ��ֹ�ж� */

	iTemp = g_tADS1256.AdcNow[_ch];

	__set_PRIMASK(0);	/* ʹ���ж� */

	return iTemp;
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_ISR
*	����˵��: ��ʱ�ɼ��жϷ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ADS1256_ISR(void)
{
	if (g_tADS1256.ScanMode == 0)	/* 0 ��ʾ����8·ɨ�裬1��ʾ���4·ɨ�� */
	{
		/* ��ȡ�ɼ��ṹ��������ȫ�ֱ��� */
		ADS1256_SetChannal(g_tADS1256.Channel);	/* �л�ģ��ͨ�� */
		bsp_DelayUS(5);

		ADS1256_WriteCmd(CMD_SYNC);
		bsp_DelayUS(5);

		ADS1256_WriteCmd(CMD_WAKEUP);
		bsp_DelayUS(25);

		if (g_tADS1256.Channel == 0)
		{
			g_tADS1256.AdcNow[7] = ADS1256_ReadData();	/* ע�Ᵽ�������һ��ͨ�������� */
		}
		else
		{
			g_tADS1256.AdcNow[g_tADS1256.Channel-1] = ADS1256_ReadData();	/* ע�Ᵽ�������һ��ͨ�������� */
		}

		if (++g_tADS1256.Channel >= 8)
		{
			g_tADS1256.Channel = 0;
		}
	}
	else	/* ���4·ɨ�� */
	{
		/* ��ȡ�ɼ��ṹ��������ȫ�ֱ��� */
		ADS1256_SetDiffChannal(g_tADS1256.Channel);	/* �л�ģ��ͨ�� */
		bsp_DelayUS(5);

		ADS1256_WriteCmd(CMD_SYNC);
		bsp_DelayUS(5);

		ADS1256_WriteCmd(CMD_WAKEUP);
		bsp_DelayUS(25);

		if (g_tADS1256.Channel == 0)
		{
			g_tADS1256.AdcNow[2] = ADS1256_ReadData();	/* ע�Ᵽ�������һ��ͨ�������� */
		}
		else
		{
			g_tADS1256.AdcNow[g_tADS1256.Channel-1] = ADS1256_ReadData();	/* ע�Ᵽ�������һ��ͨ�������� */
		}

		if (++g_tADS1256.Channel >= 3)
		{
			g_tADS1256.Channel = 0;
		}
	}
}


/*
*********************************************************************************************************
*	�� �� ��: EXTI3_IRQHandler
*	����˵��: �ⲿ�жϷ������.
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
#include "time.h"



void EXTI2_IRQHandler(void)
{
	extern OS_TCB ReadTaskTCB;
	OS_ERR err;
	if (EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line2);		// ����жϱ�־λ
		
		ADS1256_ISR();
		++excount;
		//OS_TaskResume((OS_TCB*)&ReadTaskTCB,&err);
		
		EXTI_ClearITPendingBit(EXTI_Line2);		//ִ������Ĵ�����Ϻ��ٴ������жϱ�־ 
		//printf("%ld\r\n",TIM_GetCounter(TIM5));	//TIM_GetCounter(TIM5)
	}
}


