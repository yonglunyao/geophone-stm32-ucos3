#include "dma.h"
/* SPI DR�Ĵ����ĵ�ַ����ͨ���鿴SPI2һ��һ���õ� */

/* ������ջ���ͷ��ͻ��� */
u8 SPI1_RX_BUFFER[SPI1_RX_LEN];
u8 SPI1_TX_BUFFER[SPI1_TX_LEN]; 
u8 SPI2_RX_BUFFER[SPI2_RX_LEN];
u8 SPI2_TX_BUFFER[SPI2_TX_LEN]; 
u8 SPI3_RX_BUFFER[SPI3_RX_LEN];
u8 SPI3_TX_BUFFER[SPI3_TX_LEN]; 

/*******************************************************************************
* �� �� ��         : DMAx_Init
* ��������		   : DMA��ʼ������
* ��    ��         : DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7
					 chx:DMAͨ��ѡ��,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
					 par:�����ַ
					 mar:�洢����ַ
					 ndtr:���ݴ�����
* ��    ��         : ��
*******************************************************************************/ 
void DMAx_Init(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr)
{
	DMA_InitTypeDef  DMA_InitStructure;
	
	if((u32)DMA_Streamx>(u32)DMA2)//�õ���ǰstream������DMA2����DMA1
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2ʱ��ʹ�� 	
	}
	else 
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1ʱ��ʹ�� 
	}
	DMA_DeInit(DMA_Streamx);
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}//�ȴ�DMA������ 
	
	/* ���� DMA Stream */
	DMA_InitStructure.DMA_Channel = chx;  //ͨ��ѡ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = par;//DMA�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = mar;//DMA �洢��0��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//�洢��������ģʽ
	DMA_InitStructure.DMA_BufferSize = ndtr;//���ݴ����� 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ʹ����ͨģʽ 
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//�е����ȼ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	DMA_Init(DMA_Streamx, &DMA_InitStructure);//��ʼ��DMA Stream
	
}

/*******************************************************************************
* �� �� ��         : DMAx_Enable
* ��������		   : ����һ��DMA����
* ��    ��         : DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7
					 ndtr:���ݴ�����
* ��    ��         : ��
*******************************************************************************/ 
void DMAx_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
 
	DMA_Cmd(DMA_Streamx, DISABLE);                      //�ر�DMA���� 
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE);	//ȷ��DMA���Ա�����  
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //���ݴ�����  
	DMA_Cmd(DMA_Streamx, ENABLE);                      //����DMA���� 
}	  

/* �������ΪDMA���䷽ʽ������DMA��ʼ�� */
/**
  *	@breif  The spi dma init function.
  * @param  None
  * @retval None
  */  
void spi1_dma_init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	/* ��DMA2ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	/* DMA RX config */
	DMA_InitStructure.DMA_Channel = DMA_Channel_3;                          //ѡ��DMAͨ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI1_DR_ADDR;                //�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SPI1_RX_BUFFER;        //�洢��0��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //�ڴ浽�ⲿ�豸
	DMA_InitStructure.DMA_BufferSize = 32;                                  //�������ݳ���
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ��ַ����
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //�ڴ�����Ϊ8λ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //��������Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //����ȡѭ��ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //���ȼ�Ϊ�е�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //����ȡFIFOģʽ
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           //����ȡFIFOģʽ���ò����޹ؽ�Ҫ
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //���δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //���δ���
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);
	
	
	/* DMA TX Config */
	DMA_InitStructure.DMA_Channel = DMA_Channel_3;                          //ѡ��DMAͨ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI1_DR_ADDR;                //�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SPI1_TX_BUFFER;        //�洢��0��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //�ڴ浽�ⲿ�豸
	DMA_InitStructure.DMA_BufferSize = 32;                                  //�������ݳ���
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ��ַ����
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //�ڴ�����Ϊ8λ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //��������Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //����ȡѭ��ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //���ȼ�Ϊ�е�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //����ȡFIFOģʽ
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           //����ȡFIFOģʽ���ò����޹ؽ�Ҫ
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //���δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //���δ���
	DMA_Init(DMA2_Stream5, &DMA_InitStructure);
}

void spi2_dma_init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	/* ��DMA1ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	/* DMA RX config */
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;                          //ѡ��DMAͨ��
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI2_DR_ADDR;                //�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SPI2_RX_BUFFER;        //�洢��0��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //�ڴ浽�ⲿ�豸
	
	DMA_InitStructure.DMA_BufferSize = 30;                                  //�������ݳ���
	
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ��ַ����
	
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //�ڴ�����Ϊ8λ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //��������Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //����ȡѭ��ģʽ
	
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //���ȼ�Ϊ�е�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //����ȡFIFOģʽ
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           //����ȡFIFOģʽ���ò����޹ؽ�Ҫ
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //���δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //���δ���
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);
	
	
	/* DMA TX Config */
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;                          //ѡ��DMAͨ��
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI2_DR_ADDR;                //�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SPI2_TX_BUFFER;        //�洢��0��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //�ڴ浽�ⲿ�豸
	
	DMA_InitStructure.DMA_BufferSize = 30;                                  //�������ݳ���
	
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ��ַ����
	
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //�ڴ�����Ϊ8λ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //��������Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //����ȡѭ��ģʽ
	
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //���ȼ�Ϊ�е�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //����ȡFIFOģʽ
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           //����ȡFIFOģʽ���ò����޹ؽ�Ҫ
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //���δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //���δ���
	DMA_Init(DMA1_Stream4, &DMA_InitStructure);
}

void spi3_dma_init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	/* ��DMA2ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	/* DMA RX config */
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;                          //ѡ��DMAͨ��
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI3_DR_ADDR;                //�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SPI3_RX_BUFFER;        //�洢��0��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //�ڴ浽�ⲿ�豸
	
	DMA_InitStructure.DMA_BufferSize = 30;                                  //�������ݳ���
	
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ��ַ����
	
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //�ڴ�����Ϊ8λ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //��������Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //����ȡѭ��ģʽ
	
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //���ȼ�Ϊ�е�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //����ȡFIFOģʽ
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           //����ȡFIFOģʽ���ò����޹ؽ�Ҫ
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //���δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //���δ���
	DMA_Init(DMA1_Stream0, &DMA_InitStructure);
	
	
	/* DMA TX Config */
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;                          //ѡ��DMAͨ��
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI3_DR_ADDR;                //�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SPI3_TX_BUFFER;        //�洢��0��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //�ڴ浽�ⲿ�豸
	
	DMA_InitStructure.DMA_BufferSize = 30;                                  //�������ݳ���
	
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ��ַ����
	
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //�ڴ�����Ϊ8λ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //��������Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //����ȡѭ��ģʽ
	
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //���ȼ�Ϊ�е�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //����ȡFIFOģʽ
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           //����ȡFIFOģʽ���ò����޹ؽ�Ҫ
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //���δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //���δ���
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);
}

/**
  *	@breif  The spi dma trans function.
  * @param  rx_buf -- the point of rx buffer
  * @param  tx_buf -- the point of tx buffer
  * @elngth length -- the size of data.
  * @retval None
  */  
void spi1_trans(uint8_t *rx_buf,     //����bufferָ��
			   uint8_t *tx_buf,     //����bufferָ��
			   uint16_t length)     //�������ݳ���
{
	/* �ر�DMAͨ�� */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream5, DISABLE);
	
	/* ���ô����ֽ��� */
	DMA_SetCurrDataCounter(DMA2_Stream2, (uint16_t)length);
	DMA_SetCurrDataCounter(DMA2_Stream5, (uint16_t)length);
	/* ���ͽ������ݴ洢��ַ���� */
	DMA2_Stream2->CR |= (1 << 10);
	DMA2_Stream5->CR |= (1 << 10);
	
	/* ���ý��պͷ��͵��ڴ��ַ */
	DMA2_Stream2->M0AR = (uint32_t)rx_buf;
	DMA2_Stream5->M0AR = (uint32_t)tx_buf;

	/* ��ȡһ��DR��ʹ����� */
	SPI1->DR;
	
	/* �ȴ�������Ϊ�� */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	
	/* ��DMAͨ�� */
	DMA_Cmd(DMA2_Stream2, ENABLE);
	DMA_Cmd(DMA2_Stream5, ENABLE);
	
	/* ������� */
	while( DMA_GetFlagStatus(DMA2_Stream5, DMA_FLAG_TCIF5) == RESET);
	while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
	
	/* �ر�DMAͨ�� */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream5, DISABLE);	
	
	/* ���DMA������ɱ�־ */
	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
	DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);
}

void spi2_trans(uint8_t *rx_buf,     //����bufferָ��
			   uint8_t *tx_buf,     //����bufferָ��
			   uint16_t length)     //�������ݳ���
{
	/* �ر�DMAͨ�� */
	DMA_Cmd(DMA1_Stream3, DISABLE);
	DMA_Cmd(DMA1_Stream4, DISABLE);
	
	/* ���ô����ֽ��� */
	DMA_SetCurrDataCounter(DMA1_Stream3, (uint16_t)length);
	DMA_SetCurrDataCounter(DMA1_Stream4, (uint16_t)length);
	/* ���ͽ������ݴ洢��ַ���� */
	DMA1_Stream3->CR |= (1 << 10);
	DMA1_Stream4->CR |= (1 << 10);
	
	/* ���ý��պͷ��͵��ڴ��ַ */
	DMA1_Stream3->M0AR = (uint32_t)rx_buf;
	DMA1_Stream4->M0AR = (uint32_t)tx_buf;

	/* ��ȡһ��DR��ʹ����� */
	SPI2->DR;
	
	/* �ȴ�������Ϊ�� */
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	
	/* ��DMAͨ�� */
	DMA_Cmd(DMA1_Stream3, ENABLE);
	DMA_Cmd(DMA1_Stream4, ENABLE);
	
	/* ������� */
	while( DMA_GetFlagStatus(DMA1_Stream3, DMA_FLAG_TCIF3) == RESET);
	while( DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4) == RESET);
	
	/* �ر�DMAͨ�� */
	DMA_Cmd(DMA1_Stream3, DISABLE);
	DMA_Cmd(DMA1_Stream4, DISABLE);	
	
	/* ���DMA������ɱ�־ */
	DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
	DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
}

void spi3trans(uint8_t *rx_buf,     //����bufferָ��
			   uint8_t *tx_buf,     //����bufferָ��
			   uint16_t length)     //�������ݳ���
{
	/* �ر�DMAͨ�� */
	DMA_Cmd(DMA1_Stream0, DISABLE);
	DMA_Cmd(DMA1_Stream5, DISABLE);
	
	/* ���ô����ֽ��� */
	DMA_SetCurrDataCounter(DMA1_Stream0, (uint16_t)length);
	DMA_SetCurrDataCounter(DMA1_Stream5, (uint16_t)length);
	/* ���ͽ������ݴ洢��ַ���� */
	DMA1_Stream0->CR |= (1 << 10);
	DMA1_Stream5->CR |= (1 << 10);
	
	/* ���ý��պͷ��͵��ڴ��ַ */
	DMA1_Stream0->M0AR = (uint32_t)rx_buf;
	DMA1_Stream5->M0AR = (uint32_t)tx_buf;

	/* ��ȡһ��DR��ʹ����� */
	SPI3->DR;
	
	/* �ȴ�������Ϊ�� */
	while(SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
	
	/* ��DMAͨ�� */
	DMA_Cmd(DMA1_Stream0, ENABLE);
	DMA_Cmd(DMA1_Stream5, ENABLE);
	
	/* ������� */
	while( DMA_GetFlagStatus(DMA1_Stream0, DMA_FLAG_TCIF0) == RESET);
	while( DMA_GetFlagStatus(DMA1_Stream5, DMA_FLAG_TCIF5) == RESET);
	
	/* �ر�DMAͨ�� */
	DMA_Cmd(DMA1_Stream0, DISABLE);
	DMA_Cmd(DMA1_Stream5, DISABLE);	
	
	/* ���DMA������ɱ�־ */
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
	/* �ر�DMAͨ�� */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream5, DISABLE);
	
	/* ���ô����ֽ��� */
	DMA_SetCurrDataCounter(DMA2_Stream2, (uint16_t)length);
	DMA_SetCurrDataCounter(DMA2_Stream5, (uint16_t)length);
	
	/* ����DMA���ķ��͵�ַ������ */
	DMA2_Stream5->CR &= ~(1 << 10);
	
	/* ���ý��պͷ��͵��ڴ��ַ */
	DMA2_Stream2->M0AR = (uint32_t)rx_buf;
	DMA2_Stream5->M0AR = (uint32_t)tx_data;

	/* ��ȡһ��DR��ʹ����� */
	SPI1->DR;
	
	/* �ȴ�������Ϊ�� */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	
	/* ��DMAͨ�� */
	DMA_Cmd(DMA2_Stream2, ENABLE);
	DMA_Cmd(DMA2_Stream5, ENABLE);
	
	/* ������� */
	while( DMA_GetFlagStatus(DMA2_Stream5, DMA_FLAG_TCIF5) == RESET);
	while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
	
	/* �ر�DMAͨ�� */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream5, DISABLE);	
	
	/* ���DMA������ɱ�־ */
	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
	DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);	
}


void spi2_trans_read(uint8_t *rx_buf,
					uint8_t *tx_data,
					uint16_t length)
{
	/* �ر�DMAͨ�� */
	DMA_Cmd(DMA1_Stream3, DISABLE);
	DMA_Cmd(DMA1_Stream4, DISABLE);
	
	/* ���ô����ֽ��� */
	DMA_SetCurrDataCounter(DMA1_Stream3, (uint16_t)length);
	DMA_SetCurrDataCounter(DMA1_Stream4, (uint16_t)length);
	
	/* ����DMA���ķ��͵�ַ������ */
	DMA1_Stream4->CR &= ~(1 << 10);
	
	/* ���ý��պͷ��͵��ڴ��ַ */
	DMA1_Stream3->M0AR = (uint32_t)rx_buf;
	DMA1_Stream4->M0AR = (uint32_t)tx_data;

	/* ��ȡһ��DR��ʹ����� */
	SPI2->DR;
	
	/* �ȴ�������Ϊ�� */
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	
	/* ��DMAͨ�� */
	DMA_Cmd(DMA1_Stream3, ENABLE);
	DMA_Cmd(DMA1_Stream4, ENABLE);
	
	/* ������� */
	while( DMA_GetFlagStatus(DMA1_Stream3, DMA_FLAG_TCIF3) == RESET);
	while( DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4) == RESET);
	
	/* �ر�DMAͨ�� */
	DMA_Cmd(DMA1_Stream3, DISABLE);
	DMA_Cmd(DMA1_Stream4, DISABLE);	
	
	/* ���DMA������ɱ�־ */
	DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
	DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);	
}

void spi3_trans_read(uint8_t *rx_buf,
					uint8_t *tx_data,
					uint16_t length)
{
		/* �ر�DMAͨ�� */
	DMA_Cmd(DMA1_Stream0, DISABLE);
	DMA_Cmd(DMA1_Stream5, DISABLE);
	
	/* ���ô����ֽ��� */
	DMA_SetCurrDataCounter(DMA1_Stream0, (uint16_t)length);
	DMA_SetCurrDataCounter(DMA1_Stream5, (uint16_t)length);
	
	/* ����DMA���ķ��͵�ַ������ */
	DMA1_Stream5->CR &= ~(1 << 10);
	
	/* ���ý��պͷ��͵��ڴ��ַ */
	DMA1_Stream0->M0AR = (uint32_t)rx_buf;
	DMA1_Stream5->M0AR = (uint32_t)tx_data;

	/* ��ȡһ��DR��ʹ����� */
	SPI3->DR;
	
	/* �ȴ�������Ϊ�� */
	while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
	
	/* ��DMAͨ�� */
	DMA_Cmd(DMA1_Stream0, ENABLE);
	DMA_Cmd(DMA1_Stream5, ENABLE);
	
	/* ������� */
	while( DMA_GetFlagStatus(DMA1_Stream0, DMA_FLAG_TCIF0) == RESET);
	while( DMA_GetFlagStatus(DMA1_Stream5, DMA_FLAG_TCIF5) == RESET);
	
	/* �ر�DMAͨ�� */
	DMA_Cmd(DMA1_Stream0, DISABLE);
	DMA_Cmd(DMA1_Stream5, DISABLE);	
	
	/* ���DMA������ɱ�־ */
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
	/* �ر�DMAͨ�� */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream5, DISABLE);
	
	/* ���ô����ֽ��� */
	DMA_SetCurrDataCounter(DMA2_Stream2, (uint16_t)length);
	DMA_SetCurrDataCounter(DMA2_Stream5, (uint16_t)length);
	
	/* ����DMA���Ľ��յ�ַ������ */
	DMA2_Stream2->CR &= ~(1 << 10);
	
	/* ���ý��պͷ��͵��ڴ��ַ */
	DMA2_Stream2->M0AR = (uint32_t)rx_data;
	DMA2_Stream5->M0AR = (uint32_t)tx_buffer;

	/* ��ȡһ��DR��ʹ����� */
	SPI1->DR;
	
	/* �ȴ�������Ϊ�� */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	
	/* ��DMAͨ�� */
	DMA_Cmd(DMA2_Stream2, ENABLE);
	DMA_Cmd(DMA2_Stream5, ENABLE);
	
	/* ������� */
	while( DMA_GetFlagStatus(DMA2_Stream5, DMA_FLAG_TCIF5) == RESET);
	while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
	
	/* �ر�DMAͨ�� */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream5, DISABLE);	
	
	/* ���DMA������ɱ�־ */
	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
	DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);	
}


void spi2_trans_write(uint8_t *rx_data,
					 uint8_t *tx_buffer,
					 uint16_t length)
{
	/* �ر�DMAͨ�� */
	DMA_Cmd(DMA1_Stream3, DISABLE);
	DMA_Cmd(DMA1_Stream4, DISABLE);
	
	/* ���ô����ֽ��� */
	DMA_SetCurrDataCounter(DMA1_Stream3, (uint16_t)length);
	DMA_SetCurrDataCounter(DMA1_Stream4, (uint16_t)length);
	
	/* ����DMA���Ľ��յ�ַ������ */
	DMA1_Stream3->CR &= ~(1 << 10);
	
	/* ���ý��պͷ��͵��ڴ��ַ */
	DMA1_Stream3->M0AR = (uint32_t)rx_data;
	DMA1_Stream4->M0AR = (uint32_t)tx_buffer;

	/* ��ȡһ��DR��ʹ����� */
	SPI2->DR;
	
	/* �ȴ�������Ϊ�� */
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	
	/* ��DMAͨ�� */
	DMA_Cmd(DMA1_Stream3, ENABLE);
	DMA_Cmd(DMA1_Stream4, ENABLE);
	
	/* ������� */
	while( DMA_GetFlagStatus(DMA1_Stream3, DMA_FLAG_TCIF3) == RESET);
	while( DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4) == RESET);
	
	/* �ر�DMAͨ�� */
	DMA_Cmd(DMA1_Stream3, DISABLE);
	DMA_Cmd(DMA1_Stream4, DISABLE);	
	
	/* ���DMA������ɱ�־ */
	DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
	DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);	
}

void spi3_trans_write(uint8_t *rx_data,
					 uint8_t *tx_buffer,
					 uint16_t length)
{
		/* �ر�DMAͨ�� */
	DMA_Cmd(DMA1_Stream0, DISABLE);
	DMA_Cmd(DMA1_Stream5, DISABLE);
	
	/* ���ô����ֽ��� */
	DMA_SetCurrDataCounter(DMA1_Stream0, (uint16_t)length);
	DMA_SetCurrDataCounter(DMA1_Stream5, (uint16_t)length);
	
	/* ����DMA���Ľ��յ�ַ������ */
	DMA1_Stream0->CR &= ~(1 << 10);
	
	/* ���ý��պͷ��͵��ڴ��ַ */
	DMA1_Stream0->M0AR = (uint32_t)rx_data;
	DMA1_Stream5->M0AR = (uint32_t)tx_buffer;

	/* ��ȡһ��DR��ʹ����� */
	SPI3->DR;
	
	/* �ȴ�������Ϊ�� */
	while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
	
	/* ��DMAͨ�� */
	DMA_Cmd(DMA1_Stream0, ENABLE);
	DMA_Cmd(DMA1_Stream5, ENABLE);
	
	/* ������� */
	while( DMA_GetFlagStatus(DMA1_Stream0, DMA_FLAG_TCIF0) == RESET);
	while( DMA_GetFlagStatus(DMA1_Stream5, DMA_FLAG_TCIF5) == RESET);
	
	/* �ر�DMAͨ�� */
	DMA_Cmd(DMA1_Stream0, DISABLE);
	DMA_Cmd(DMA1_Stream5, DISABLE);	
	
	/* ���DMA������ɱ�־ */
	DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);
	DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);	
}


