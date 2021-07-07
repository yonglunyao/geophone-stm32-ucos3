#include "system.h"
#include "SysTick.h"
#include "includes.h"
#include <stdlib.h>
#include "sim7600.h"
#include "usart.h"
#include "gps.h"
#include "bsp_ads1256.h"
#include "spi.h"
#include "time.h"
#include "exti.h"
#include "sdio_sdcard.h" 
#include "malloc.h"
#include "ff.h" 
#include "fatfs_app.h"
#include "iwdg.h"
#include "math.h"
#include "file_sys.h"
#include "dma.h"
#include "oled.h"
#include "stmflash.h"

//����վ���
#define NUMBER 4
//�������ȼ�
#define START_TASK_PRIO		3
//�����ջ��С	
#define START_STK_SIZE 		128 //ԭֵ128
//������ƿ�
OS_TCB StartTaskTCB;
//�����ջ	
CPU_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *p_arg);

u8 SwitchFtptoFlash;//��ǹ���ģʽ��0��FTP��1��U��
void Set_Mode(void);

//�������ȼ�
#define READ_TASK_PRIO		4
//�����ջ��С	
#define READ_STK_SIZE 		150 //ԭֵ96
//������ƿ�
OS_TCB ReadTaskTCB;
//�����ջ	
CPU_STK READ_TASK_STK[READ_STK_SIZE];
void read_task(void *p_arg);


//�������ȼ�
#define GPS_TASK_PRIO		4
//�����ջ��С	
#define GPS_STK_SIZE 		128 //ԭֵ64
//������ƿ�
OS_TCB GpsTaskTCB;
//�����ջ	
CPU_STK GPS_TASK_STK[GPS_STK_SIZE];
void gps_task(void *p_arg);


//�������ȼ�
#define FILE_TASK_PRIO		5
//�����ջ��С	
#define FILE_STK_SIZE 		500 //ԭֵ330
//������ƿ�
OS_TCB FileTaskTCB;
//�����ջ	
CPU_STK FILE_TASK_STK[FILE_STK_SIZE];
void file_task(void *p_arg);

//�������ȼ�
#define FTP_TASK_PRIO		6
//�����ջ��С	
#define FTP_STK_SIZE 		500 //ԭֵ330
//������ƿ�
OS_TCB FtpTaskTCB;
//�����ջ	
CPU_STK FTP_TASK_STK[FTP_STK_SIZE];
void ftp_task(void *p_arg);

//�������ȼ�
#define USB_TASK_PRIO		6
//�����ջ��С	
#define USB_STK_SIZE 		330
//������ƿ�
OS_TCB UsbTaskTCB;
//�����ջ	
CPU_STK USB_TASK_STK[USB_STK_SIZE];
void usb_task(void *p_arg);

//�������ȼ�
#define MONITOR_TASK_PRIO		7
//�����ջ��С	
#define MONITOR_STK_SIZE 		150 //ԭֵ96
//������ƿ�
OS_TCB MonitorTaskTCB;
//�����ջ	
CPU_STK MONITOR_TASK_STK[MONITOR_STK_SIZE];
void monitor_task(void *p_arg);

OS_MUTEX mutex; //�����ź���
OS_SEM full; //��ֵ�ź���
OS_SEM empty; //��ֵ�ź���
OS_SEM sdinit; //��ֵ�ź���
OS_MUTEX sdmutex; //��ֵ�ź���
OS_SEM read; //��ֵ�ź���

u32 local_files_num=0;//��¼SD���д����͵��ļ�����
u32 sent_files_num=0;//��¼ftp������ɵ��ļ�����


int main()
{  	
	
	OS_ERR err;
	
	delay_init();
	USART1_Init(921600);
	USART3_Init(38400);		
	printf("��ǰվ�㣺%d\r\n\r\n",NUMBER);
#if OLED_ENABLE
	extern u8 oled_buf[17];
	OLED_Init();
	sprintf((char*)oled_buf,"   Station %d    ",NUMBER); //δ����ԭ�򣺿���������ʾ��B OSStartHang
	OLED_ShowString(0,0,(u8*)oled_buf);
//	OLED_ShowString(0,3,(u8*)"    WELCOME!    ");
	OLED_ShowString(0,6,(u8*)"Designed By DLUT");
#endif
	bsp_InitADS1256();
	SIM7600_Init();
	Set_Mode();
	delay_ms(500);
	while(1)
	{
		delay_ms(500);
		u8 id0 = 3, id1 = 3, id2 = 3;
#if CH0_ENABLE
		id0 = ADS1256_ReadChipID(0);
#endif
#if CH1_ENABLE
		id1 = ADS1256_ReadChipID(1);
#endif
#if CH2_ENABLE
		id2 = ADS1256_ReadChipID(2);
#endif
//		printf("��ȡоƬID\r\n");
		if (id0 != 3 || id1 != 3 || id2 != 3)
		{
			printf("��ADS1256����\r\nASD1256 Chip ID = 0x%02X & 0x%02X & 0x%02X\r\n", id0, id1, id2);
#if OLED_ENABLE
			if(id0 != 3) OLED_ShowString(0,3,(u8*)" ADS1256:1 error");
			if(id1 != 3) OLED_ShowString(0,3,(u8*)" ADS1256:2 error");
			if(id2 != 3) OLED_ShowString(0,3,(u8*)" ADS1256:3 error");
#endif
		}
		else
		{
			printf("Ok, ASD1256 Chip ID = 0x%02X & 0x%02X & 0x%02X\r\n\r\n", id0, id1, id2);	
#if OLED_ENABLE
			OLED_ShowString(0,3,(u8*)"   ADS1256 OK   ");
			delay_ms(1000);
#endif
			break;
		}
	}

//***************************************************************************
	OSInit(&err);		//��ʼ��UCOSIII
	//������ʼ����
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//������ƿ�
				 (CPU_CHAR	* )"start task", 		//��������
                 (OS_TASK_PTR )start_task, 			//������
                 (void		* )0,					//���ݸ��������Ĳ���
                 (OS_PRIO	  )START_TASK_PRIO,     //�������ȼ�
                 (CPU_STK   * )&START_TASK_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)START_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY  )0,					//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,					//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,					//�û�����Ĵ洢��
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);				//��Ÿú�������ʱ�ķ���ֵ 
	OSStart(&err);  //����UCOSIII
	while(1);
}

//����ģʽ

//Ҫд�뵽STM32 FLASH���ַ�������
const u8 FTP_Buffer[]={"FTP"};
const u8 FLASH_Buffer[]={"FLASH"};

#define FTP_LENTH sizeof(FTP_Buffer)	 		  	//���鳤��	
#define FLASH_LENTH sizeof(FLASH_Buffer)	 		  	//���鳤��
	
#define FTP_SIZE FTP_LENTH/4+((FTP_LENTH%4)?1:0)	//���ֽڶ�ȡ
#define FLASH_SIZE FLASH_LENTH/4+((FLASH_LENTH%4)?1:0)

#define FLASH_SAVE_ADDR  0X08020004 	//����FLASH �����ַ(����Ϊż��������������,Ҫ���ڱ�������ռ�õ�������.

void Set_Mode(void)
{
	u8 buff[8];	//�ֽڶ�ȡ������
	char* strx;	//������
	STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)buff,FLASH_SIZE);//�ȶ�ȡһ��
	strx=strstr((const char*)buff,"FTP");//����ȡ�����Ƿ���FTP
	//����ȡ����
	if(strx!=NULL)
		SwitchFtptoFlash=0;//0����FTPģʽ
	else
	{
		strx=strstr((const char*)buff,"FLASH");//����ȡ�����Ƿ���FLASH
		if(strx!=NULL)
			SwitchFtptoFlash=1;//1����U��ģʽ
		else
		{
			SwitchFtptoFlash=0;
			printf("δ��⵽��һ������ģʽ��Ĭ����FTPģʽ����\r\n\r\n");
		}
	}
	
	//��ʼ׼������ģʽ
	switch (SwitchFtptoFlash)
	{
		case 0:
			printf("ѡ����ģʽ(��ǰ����ģʽ��FTPģʽ���л��밴��)\r\n\r\n");
			break;
		case 1:
			printf("ѡ����ģʽ(��ǰ����ģʽ��U��ģʽ���л��밴��)\r\n\r\n");
			break;
	}
	//׼��ѡ��ģʽ
	Copy_EXTI_Init();//Copy�����жϳ�ʼ��
	u8 tmp=SwitchFtptoFlash;
	for(int i=0;i<5;i++)
	{
		printf("����%d����ѡ��\r\n\r\n",5-i);
		if(tmp!=SwitchFtptoFlash)break;//��⵽���������˳��ȴ�
		delay_ms(1000);
	}
	//��ģʽд���ڲ�FLASH������һ�β�ѯʹ��
	if(SwitchFtptoFlash)//�����1��U��ģʽ
	{
		printf("��ǰģʽ:U��ģʽ\r\n\r\n");	//1:U��
		STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)FLASH_Buffer,FLASH_SIZE);
	}
	else
	{
		printf("��ǰģʽ:FTPģʽ\r\n\r\n");	//0:FLASH
		STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)FTP_Buffer,FTP_SIZE);
	}
}

///*����ģʽ (���Գ������⣬f_write���ļ�ָ������޷��ر��ļ�)
//����������ÿ��������Ĭ�ϲ�����һ������ģʽ��ģʽ��Ϣ������SD��/config/config.txt/
//���ȳ��Զ�ȡSD��Ŀ¼,����Ҳ��������ļ���ֱ��д��"Mode=FTP"
//Ȼ���ٴζ�ȡ�����ļ���ȷ����һ�ι���ģʽ
//�򿪰����жϣ���ʼ��ʱ�ȴ������������Ҫ����ģʽ����Ҫ����ʱ�ڼ��ڰ�������ʱ��Ч�����������ģʽ������������ȴ���ʱ����
//�ж�ģʽ�Ƿ���ģ����Ѹ�����д�뵽SD�������ļ������δ�����򲻲���SD��
//*/
//	FIL sdfp;
//void Set_Mode(void)
//{
//	OS_ERR err;
//	const TCHAR sdpath[20];
//	FRESULT res=1;
//	
//	__align(4) u8 sdwbuff[20];
//	u32 hbw;
//	res = 1;
//	DIR dir;
//	char* strx;
//	
//	my_mem_init(SRAMIN);		//��ʼ���ڲ��ڴ��
//	//SD����ʼ��
//	while(SD_Init()!=0)
//	{	
//		delay_ms(4);
//	}
//	FATFS_Init();							//Ϊsd�����ݻ�����fatfs���ļ�1���ļ�2�������̹�������ر��������ڴ�		
//  f_mount(fs[0],"0:",1); 					//����SD����ʹ��fs[0]���̹�����0	
//	
//	res = f_opendir(&dir, "0:/config");
//	if(res==FR_NO_PATH)
//	{
//		f_opendir(&dir, "0:/");
//		res=1;
//		while(res)
//		{
//			res=f_mkdir((const TCHAR*)"config");
//		}
//		res=1;
//		while(res)
//		{
//			res = f_opendir(&dir, "0:/config");
//		}
//	}

//	sprintf((char*)sdpath,"0:/config/config.txt");
//	res = f_open(&sdfp,(const char *)sdpath,FA_READ);//�ȳ��Դ�
//	//�����ʧ�ܾʹ���һ�����ļ�
//	while(res)
//	{
//		res=1;
//		while(res)
//		{
//			res = f_open(&sdfp,(const char *)sdpath,FA_CREATE_ALWAYS|FA_WRITE);//�����ʧ�ܾʹ���һ��
//		}
//		res=1;
//		sprintf((char*)sdwbuff,"Mode=FTP");
//		while(res)
//		{
//			res = f_write(&sdfp,&sdwbuff,strlen((char*)sdwbuff),&hbw);
//		}
//		res=1;
//		while(res)
//		{
//			res = f_close(&sdfp);
//		}
//		
//		res=1;
//		while(res)//ˢ��Ŀ¼��׼����ȡ
//		{
//			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
//			res = f_opendir(&dir, "0:/config");
//			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
//			if(res) printf("opendir:%d\r\n", res);
//			OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err);
//		}
//		res=1;
//		while(res)
//		{
//			res = f_open(&sdfp,(const char *)sdpath,FA_READ);//��ȡ��ʽ���ļ�
//		}
//	}
//	
//	//��ʼ��ȡ�ļ�
//	res=1;
//	hbw=0;
//	memset(sdwbuff,0,sizeof(sdwbuff));//�������׼������
//	while(res)
//	{
//		res=f_read (&sdfp, &sdwbuff, 10, &hbw);
//	}
//	res=1;
//	while(res)
//	{
//		res = f_close(&sdfp);
//	}
//	
//	strx=strstr((const char*)sdwbuff,"FTP");//���ȼ���ļ����Ƿ���"FTP"
//	if(strx!=NULL) SwitchFtptoFlash=0;//0����FTPģʽ
//	else 
//	{
//		strx=strstr((const char*)sdwbuff,"FLASH");//����ļ����Ƿ���"FLASH"
//		if(strx!=NULL) SwitchFtptoFlash=1;//1����U��ģʽ
//		else 
//		{
//			printf("ģʽ�����ڲ�����Ĭ��FTPģʽ���У�\r\n\r\n");
//			SwitchFtptoFlash=0;
//			res=1;
//			while(res)
//			{
//				res = f_open(&sdfp,(const char *)sdpath,FA_CREATE_ALWAYS|FA_WRITE);
//			}
//			sprintf((char*)sdwbuff,"Mode=FTP");
//			res=1;
//			hbw=0;
//			int s=strlen((char*)sdwbuff);
//			while(res)
//			{
//				res = f_write(&sdfp,&sdwbuff,s,&hbw);
//			}
//			res=1;
//			while(res)
//			{
//				res = f_close(&sdfp);
//			}
//			return;
//		}		
//	}

//	
//	//��ʼ׼������ģʽ
//	switch (SwitchFtptoFlash)
//	{
//		case 0:
//			printf("ѡ����ģʽ(��ǰ����ģʽ��FTPģʽ���л��밴��)\r\n\r\n");
//			break;
//		case 1:
//			printf("ѡ����ģʽ(��ǰ����ģʽ��U��ģʽ���л��밴��)\r\n\r\n");
//			break;
//	}
//	Copy_EXTI_Init();//Copy�����жϳ�ʼ��
//	u8 tmp=SwitchFtptoFlash;
//	for(int i=0;i<5;i++)
//	{
//		printf("����%d����ѡ��\r\n\r\n",5-i);
//		if(tmp!=SwitchFtptoFlash)break;//��⵽��������
//		delay_ms(1000);
//	}
//	if(!SwitchFtptoFlash)printf("��ǰģʽ:FTPģʽ\r\n\r\n");
//	if(SwitchFtptoFlash)printf("��ǰģʽ:U��ģʽ\r\n\r\n");
//	//���ģʽ�ı���Ҫ����д�뵽SD��
//	if(tmp!=SwitchFtptoFlash)
//	{
//		res=1;
//		while(res)//ˢ��Ŀ¼��׼��д��
//		{
//			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
//			res = f_opendir(&dir, "0:/");
//			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
//			if(res) printf("opendir:%d\r\n", res);
//			OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err);
//		}
//		res=1;
//		while(res)
//		{
//			res = f_open(&sdfp,(const char *)sdpath,FA_OPEN_ALWAYS|FA_WRITE);//�����ʧ�ܾʹ���һ��
//		}
//		switch (SwitchFtptoFlash)
//		{
//			case 0:
//				sprintf((char*)sdwbuff,"Mode=FTP");
//				break;
//			case 1:
//				sprintf((char*)sdwbuff,"Mode=FLASH");
//				break;
//		}
//		res=1;
//		hbw=0;
//		while(res)
//		{
//			res = f_write(&sdfp,sdwbuff,sizeof(sdwbuff),&hbw);
//		}
//		res=1;
//		while(res)
//		{
//			res = f_close(&sdfp);
//		}
//	}
//}

//��ʼ������
void start_task(void *p_arg)
{
//	printf("start task\r\n\r\n"); //Edit By Yao
	OS_ERR err;
	
	CPU_INT32U  cpu_clk_freq;
  CPU_INT32U  cnts;
	
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
	
	cpu_clk_freq = BSP_CPU_ClkFreq();                           /* Determine SysTick reference freq.                    */
  cnts = cpu_clk_freq / (CPU_INT32U)OSCfg_TickRate_Hz;        /* Determine nbr SysTick increments                     */
  OS_CPU_SysTickInit(cnts);                                   /* Init uC/OS periodic time src (SysTick).              */

  Mem_Init();       
	
	
#if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&err);  	//ͳ������                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//���ʹ���˲����жϹر�ʱ��
  CPU_IntDisMeasMaxCurReset();	
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //��ʹ��ʱ��Ƭ��ת��ʱ��
	 //ʹ��ʱ��Ƭ��ת���ȹ���,ʱ��Ƭ����Ϊ1��ϵͳʱ�ӽ��ģ���1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif		
	
	OS_CRITICAL_ENTER();	//�����ٽ���
	
	//���������ź��� mutex ������������
  OSMutexCreate((OS_MUTEX *)&mutex,    //ָ���ź���������ָ��
               (CPU_CHAR    *)"mutex",    //�ź���������
               (OS_ERR      *)&err);         //��������
	//������ֵ�ź��� full ��������������
  OSSemCreate((OS_SEM      *)&full,    //ָ���ź���������ָ��
               (CPU_CHAR    *)"full",    //�ź���������
               (OS_SEM_CTR   )0,             //�ź������������е���Դ��Ŀ
               (OS_ERR      *)&err);         //��������
	//������ֵ�ź��� empty ��������λ
  OSSemCreate((OS_SEM      *)&empty,    //ָ���ź���������ָ��
               (CPU_CHAR    *)"empty",    //�ź���������
               (OS_SEM_CTR   )QSIZE,             //�ź������������е���Դ��Ŀ
               (OS_ERR      *)&err);         //��������	
	//������ֵ�ź��� sdinit sd����ʼ��
  OSSemCreate((OS_SEM      *)&sdinit,    //ָ���ź���������ָ��
               (CPU_CHAR    *)"sdinit",    //�ź���������
               (OS_SEM_CTR   )0,             //�ź������������е���Դ��Ŀ
               (OS_ERR      *)&err);         //��������			
	//���������ź��� sdmutex sd��������
	OSMutexCreate((OS_MUTEX *)&sdmutex,    //ָ���ź���������ָ��
               (CPU_CHAR    *)"sdmutex",    //�ź���������
               (OS_ERR      *)&err);         //��������			
	//������ֵ�ź��� read ��ads1282
  OSSemCreate((OS_SEM      *)&read,    //ָ���ź���������ָ��
               (CPU_CHAR    *)"read",    //�ź���������
               (OS_SEM_CTR   )0,             //�ź������������е���Դ��Ŀ
               (OS_ERR      *)&err);         //��������					
			
	//����CS5376A����
	OSTaskCreate((OS_TCB 	* )&ReadTaskTCB,		
				 (CPU_CHAR	* )"read task", 		
                 (OS_TASK_PTR )read_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )READ_TASK_PRIO,     
                 (CPU_STK   * )&READ_TASK_STK[0],	
                 (CPU_STK_SIZE)READ_STK_SIZE/10,	
                 (CPU_STK_SIZE)READ_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);	
							
	//����gps����
	OSTaskCreate((OS_TCB 	* )&GpsTaskTCB,		
				 (CPU_CHAR	* )"gps task", 		
                 (OS_TASK_PTR )gps_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )GPS_TASK_PRIO,     
                 (CPU_STK   * )&GPS_TASK_STK[0],	
                 (CPU_STK_SIZE)GPS_STK_SIZE/10,	
                 (CPU_STK_SIZE)GPS_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);
						 						
	//����file����
	OSTaskCreate((OS_TCB 	* )&FileTaskTCB,		
				 (CPU_CHAR	* )"file task", 		
                 (OS_TASK_PTR )file_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )FILE_TASK_PRIO,     
                 (CPU_STK   * )&FILE_TASK_STK[0],	
                 (CPU_STK_SIZE)FILE_STK_SIZE/10,	
                 (CPU_STK_SIZE)FILE_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )2,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);	
			
	//����ftp����
	OSTaskCreate((OS_TCB 	* )&FtpTaskTCB,		
				 (CPU_CHAR	* )"ftp task", 		
                 (OS_TASK_PTR )ftp_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )FTP_TASK_PRIO,     
                 (CPU_STK   * )&FTP_TASK_STK[0],	
                 (CPU_STK_SIZE)FTP_STK_SIZE/10,	
                 (CPU_STK_SIZE)FTP_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);	

 //����USB����							 
	OSTaskCreate((OS_TCB 	* )&UsbTaskTCB,		
				 (CPU_CHAR	* )"usb task", 		
                 (OS_TASK_PTR )usb_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )USB_TASK_PRIO,     
                 (CPU_STK   * )&USB_TASK_STK[0],	
                 (CPU_STK_SIZE)USB_STK_SIZE/10,	
                 (CPU_STK_SIZE)USB_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);	

/*
	//����monitor����
	OSTaskCreate((OS_TCB 	* )&MonitorTaskTCB,		
				 (CPU_CHAR	* )"monitor task", 		
                 (OS_TASK_PTR )monitor_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )MONITOR_TASK_PRIO,     
                 (CPU_STK   * )&MONITOR_TASK_STK[0],	
                 (CPU_STK_SIZE)MONITOR_STK_SIZE/10,	
                 (CPU_STK_SIZE)MONITOR_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);	
						*/		 
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//����ʼ����			 
	OS_TaskSuspend((OS_TCB*)&ReadTaskTCB,&err);		//����ɼ�����GPS��λ�ɹ�����
	OS_TaskSuspend((OS_TCB*)&FileTaskTCB,&err);		//�����ļ�����GPS��λ�ɹ�����
	OS_TaskSuspend((OS_TCB*)&FtpTaskTCB,&err);		//����FTP����GPS��λ�ɹ�����
	OS_TaskSuspend((OS_TCB*)&UsbTaskTCB,&err);		//����USB���񣬵ȴ������������л�
	
	OS_CRITICAL_EXIT();	//�����ٽ���
}

//cs5376a������
void read_task(void *p_arg)
{
//	printf("read task alive\r\n"); //Edit By Yao
	u8 reg0[11] = {0};
	u8 reg1[11] = {0};
	u8 reg2[11] = {0};
	
	while (reg0[3] != 0x92 || reg1[3] != 0x92 || reg2[3] != 0x92)
	{
		ADS1256_CfgADC(0, ADS1256_GAIN_1, ADS1256_500SPS);
		ADS1256_CfgADC(1, ADS1256_GAIN_1, ADS1256_500SPS);
		ADS1256_CfgADC(2, ADS1256_GAIN_1, ADS1256_500SPS);
		ADS1256_CheckReg(0, reg0);
		ADS1256_CheckReg(1, reg1);
		ADS1256_CheckReg(2, reg2);
	}
	
/* Edit By Yao
	printf("Status:0x%02x 0x%02x 0x%02x\r\n", reg0[0], reg1[0], reg2[0]);
	printf("Mux:0x%02x 0x%02x 0x%02x\r\n", reg0[1], reg1[1], reg2[1]);
	printf("ADCon:0x%02x 0x%02x 0x%02x\r\n", reg0[2], reg1[2], reg2[2]);
	printf("DRate:0x%02x 0x%02x 0x%02x\r\n", reg0[3], reg1[3], reg2[3]);
	printf("IO  :0x%02x 0x%02x 0x%02x\r\n", reg0[4], reg1[4], reg2[4]);
	printf("OFC0:0x%02x 0x%02x 0x%02x\r\n", reg0[5], reg1[5], reg2[5]);
	printf("OFC1:0x%02x 0x%02x 0x%02x\r\n", reg0[6], reg1[6], reg2[6]);
	printf("OFC2:0x%02x 0x%02x 0x%02x\r\n", reg0[7], reg1[7], reg2[7]);
	printf("FSC0:0x%02x 0x%02x 0x%02x\r\n", reg0[8], reg1[8], reg2[8]);
	printf("FSC1:0x%02x 0x%02x 0x%02x\r\n", reg0[9], reg1[9], reg2[9]);
	printf("FSC2:0x%02x 0x%02x 0x%02x\r\n", reg0[10], reg1[10], reg2[10]);
*/	
	ADS1256_StartScan(1);
	OS_ERR err;
	CPU_SR_ALLOC();
	u16 tmpms;
	u8 tmpsec;
	u8 tmpmin;
	u8 tmphour;
	u8 tmpday;
	int32_t temp;
	int32_t volt[3];
	u8 i;
	while(1)
	{
		OS_TaskSuspend((OS_TCB*)&ReadTaskTCB,&err);	
		//OSSemPend(&read,0,OS_OPT_PEND_BLOCKING,0,&err);  //��read����
		OSSchedLock(&err); //��ֹ���̵���
		
		tmpms = TIM5->CNT;
		tmpsec = sec;
		tmpmin = min;
		tmphour = hour;
		tmpday = day;
		
		count++;
		ADS1256_ISR();//��ʱ�ɼ�
		OSSchedUnlock(&err); //�ָ����̵���
		for (i = 0; i < 3; ++i)
		{
			temp = ADS1256_GetAdc(i); //��ȫ�ֻ�������ȡ��������� ������������жϷ�������ж�ȡ��
/*У׼ADC,����AD����Ҫ����У׼		Add By Yao																						*/
#if NUMBER==1
			volt[0] = ((int64_t)temp * 2500000) / 4220581;	//У׼������ʹ��AD�ɼ�2.5V��׼��ѹ���õ�v1
			volt[1] = ((int64_t)temp * 2500000) / 4221009;	//�����ϵ��v1/2.5=ԭֵ/��ֵ
			volt[2] = ((int64_t)temp * 2500000) / 4220475;
#else
			volt[i] = ((int64_t)temp * 2500000) / 4194303;	// ����ʵ�ʵ�ѹֵ��ԭֵ�����ƹ���ģ���4194303 = 2.5V
#endif
			

		}
		OSSemPend(&empty,0,OS_OPT_PEND_BLOCKING,0,&err);  //�ȴ�empty�ź���
		OSMutexPend(&mutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�mutux�ź���
		//printf("ch1:%d ch2:%d ch3%d\r\n", volt[0], volt[1], volt[2]);
		qdata[qrear].ch[0] = (float)((double)volt[0] / 1000000.0);
		qdata[qrear].ch[1] = (float)((double)volt[1] / 1000000.0);
		qdata[qrear].ch[2] = (float)((double)volt[2] / 1000000.0);
//		printf("%f\r\n",qdata[qrear].ch[1]);//������
		qdata[qrear].day = day;
		qdata[qrear].sec = tmpsec;
		qdata[qrear].min = tmpmin;
		qdata[qrear].hour = tmphour;
		qdata[qrear].ms = tmpms / 10;
		qrear = (qrear + 1) % QSIZE;
		
		OSMutexPost(&mutex,OS_OPT_POST_FIFO,&err); 
		OSSemPost(&full,OS_OPT_POST_FIFO,&err); //�ͷ�full�ź���
		if (full.Ctr >= 400) printf("Warning: %d Buffer\r\n", full.Ctr);
		
	}
	
}

//gps������
void gps_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	u32 t=0;
	TIM5_Init(10000-1, 8394-1); //8393-1 10-1
//TIM2_Init(10000-1, 8500-1); //���ڼ��GPS״̬
	while(!(USART3_RX_STA&0X8000)) //Edit By Yao ���GPSͨѶ���
	{
		t++;
		if(t==10000000)
		{
			printf("GPSͨѶ�쳣��\r\n");
#if OLED_ENABLE
			OLED_ShowString(0,3,(u8*)"   GPS error    ");
#endif
			t=0;
		}
	}
	printf("�ȴ�GPS��λ���...\r\n\r\n");	//Add By Yao
#if OLED_ENABLE
	OLED_ShowString(0,3,(u8*)"GPS is not ready");
#endif
 	USART3_RX_STA=0;	
	GPS_Analysis(&gpsx,(u8*)USART3_RX_BUF);
	My_EXTI_Init();
	while(1) 
	{
		if(USART3_RX_STA&0X8000)	
		{
 			USART3_RX_STA=0;	
			GPS_Analysis(&gpsx,(u8*)USART3_RX_BUF);
			OS_TaskSuspend((OS_TCB*)&GpsTaskTCB,&err);
			//printf("UTC Time:%02d:%02d:%02d.%03d   ",gpsx.utc.hour,gpsx.utc.min,gpsx.utc.sec,gpsx.utc.ms);	//??UTC??
			//printf("Time:%02d:%02d:%02d.%03d   \r\n",hour,min,sec,TIM5->CNT);	//??UTC??
#if OLED_ENABLE
			extern u8 oled_buf[17];
			sprintf((char*)oled_buf,"%02d-%02d   %02d:%02d:%02d",month,day,hour,min,sec);
			OLED_ShowString(0,0,(u8*)oled_buf);
#endif
 		}	   
		else
			OSTimeDlyHMSM(0,0,0,40,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

void file_task(void *p_arg)
{
	OS_ERR err;
	FRESULT res;
#if OLED_ENABLE
	extern u8 oled_buf[17];;
#endif
//	my_mem_init(SRAMIN);		//��ʼ���ڲ��ڴ��
//	//SD����ʼ��
//	while(SD_Init()!=0)
//	{	
//		OSTimeDlyHMSM(0,0,0,4,OS_OPT_TIME_HMSM_STRICT,&err);
//	}
//	FATFS_Init();							//Ϊsd�����ݻ�����fatfs���ļ�1���ļ�2�������̹�������ر��������ڴ�		
//  f_mount(fs[0],"0:",1); 					//����SD����ʹ��fs[0]���̹�����0	
//	OSSemPost(&sdinit,OS_OPT_POST_FIFO,&err);
	FIL sdfp; //�ļ�ָ��
	__align(4) u8 sdwbuff[30];
	u8 hpath[20];
	u8 dpath[20];
	u8 fname[20];
	u8 init = 0;
//	u8 i;
	u16 numa = 0;
	u16 numb = 0;
	u32 hbw=0;
	u32 dbw=0;
	u16 *tmp16;
	float *tmpf;	
	
	/*��������ɾ��SD���е������ļ�����Ҫ��ɾ��������δ�洢��ɵ��ļ����� Add By Yao*/
	FRESULT fres = 1;
	DIR dir;
	while(fres)//ˢ��Ŀ¼
	{
		OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
		fres = f_opendir(&dir, "0:/");
		OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
		if(fres) printf("opendir:%d\r\n", fres);
		OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err);
	}
	
	OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
	fres = f_findfirst(&dir, &fileinfo,"","*.*");	//Ѱ�ҵ�һ��ƥ����
	OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
	while (fres == FR_OK && fileinfo.fname[0])  //�ļ���ȡʧ�ܻ�ǺϷ��ļ�
	{
		if (fileinfo.fsize == 30 || fileinfo.fsize == 900000) //������ļ�
		{
			local_files_num++;
			fres=f_findnext(&dir, &fileinfo);
			continue;
		}
		else	//δ����ļ�
		{
			sprintf((char *)dpath,"0:/%s",fileinfo.fname);
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
			fres = f_unlink((const char *)dpath);
			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
			printf("��������ļ�:%s\r\n\r\n",fileinfo.fname);
			fres=f_findnext(&dir, &fileinfo);
			continue;
		}
	}
	
#if OLED_ENABLE
			sprintf((char*)oled_buf,"Wait:%02d Sent:%03d",local_files_num,sent_files_num);
			OLED_ShowString(0,6,(u8*)oled_buf);
#endif
//	OS_TaskSuspend((OS_TCB*)&FileTaskTCB,&err);		//�ڴ˹����ļ����񣬵ȴ�GPS��λ�ɹ�
	
	
	IWDG_Init(5,625); //���Ź���ʼ��
	while(1)
	{
		OSSemPend(&full,0,OS_OPT_PEND_BLOCKING,0,&err);  //�ȴ�full�ź���
		OSMutexPend(&mutex,0,OS_OPT_PEND_BLOCKING,0,&err);   //�ȴ�mutux�ź���	
		queue tmpq = qdata[qfront];
		qfront = (qfront + 1) % QSIZE;
		OSMutexPost(&mutex,OS_OPT_POST_FIFO,&err);  //�ͷ�mutux�ź���
		OSSemPost(&empty,OS_OPT_POST_FIFO,&err);
		if (full.Ctr <= QSIZE - 100)
		IWDG_FeedDog(); //ι��
		//continue;
		if (day == 0 || day > 31) 
		{
			//printf("Invalid Data: %d %d %d %d %d\r\n", tmpq.day, tmpq.hour, tmpq.min, tmpq.sec, tmpq.ms);
			continue;
		}
		if (init == 0)
		{
			sprintf((char *)fname,"0:/%02d%02d%02d%02d",month, day, hour, min);
			sprintf((char *)hpath,"%s.hea",fname);
			res = 1;
			while (res)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
				res = f_open(&sdfp,(const char *)hpath,FA_CREATE_ALWAYS | FA_WRITE);
				OSMutexPost(&sdmutex,OS_OPT_POST_NONE,&err);  //�ͷ�sdmutux�ź���,�Ҳ��������
//				printf("FOPEN:%d\r\n",res);
			}
			LBlocate(&gpsx);
			tmp16 = (u16 *)&sdwbuff[0];  //վ����
			*tmp16 = NUMBER;
			tmp16 = (u16 *)&sdwbuff[2];  //�������
			*tmp16 = 4;
			tmpf = (float *)&sdwbuff[4]; //վ������X
			*tmpf = gpsx.longitude / 100000.0;
			tmpf = (float *)&sdwbuff[8]; //վ������Y
			*tmpf = gpsx.latitude / 100000.0;
			tmpf = (float *)&sdwbuff[12];//վ��߶�
			*tmpf = gpsx.altitude / 10.0;
			tmp16 = (u16 *)&sdwbuff[16]; //��
			*tmp16 = year;
			tmp16 = (u16 *)&sdwbuff[18]; //��
			*tmp16 = month;
			tmp16 = (u16 *)&sdwbuff[20]; //��
			*tmp16 = tmpq.day;
			tmp16 = (u16 *)&sdwbuff[22]; //ʱ
			*tmp16 = tmpq.hour;
			tmp16 = (u16 *)&sdwbuff[24]; //��
			*tmp16 = tmpq.min;
			tmp16 = (u16 *)&sdwbuff[26]; //��
			*tmp16 = tmpq.sec;
			tmp16 = (u16 *)&sdwbuff[28]; //����
			*tmp16 = tmpq.ms;
			
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
			res = f_write(&sdfp,&sdwbuff,sizeof(sdwbuff),&hbw);  
			OSMutexPost(&sdmutex,OS_OPT_POST_NONE,&err);  //�ͷ�sdmutux�ź���,�Ҳ��������
			//res = f_write(&sdfp,&sdwbuff,sizeof(sdwbuff),&dbw); 
//			printf("head:%d\r\n",res);
			res = 1;
			while (res)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
				res = f_close(&sdfp);
				OSMutexPost(&sdmutex,OS_OPT_POST_NONE,&err);  //�ͷ�sdmutux�ź���,�Ҳ��������
//				printf("FCLOSE:%d\r\n",res);
			}
			hbw = 0;
			init = 1;
			numa = 0;
			numb = 0;
			local_files_num++;
#if OLED_ENABLE
			sprintf((char*)oled_buf,"Wait:%02d Sent:%03d",local_files_num,sent_files_num);
			OLED_ShowString(0,6,(u8*)oled_buf);
#endif
		}
		if (numb == 0)
		{
			sprintf((char *)dpath,"%s.%03d",fname, numa);
			res = 1;
			while (res)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
				res = f_open(&sdfp,(const char *)dpath,FA_CREATE_ALWAYS | FA_WRITE);
				OSMutexPost(&sdmutex,OS_OPT_POST_NONE,&err);  //�ͷ�sdmutux�ź���,�Ҳ��������
//				printf("FOPEN:%d\r\n",res);
			}			
		}
		tmp16 = (u16 *)&sdwbuff[0]; //���
		*tmp16 = numb;
		tmp16 = (u16 *)&sdwbuff[2];  //������ʽ
		*tmp16 = 3;
		tmp16 = (u16 *)&sdwbuff[4];  //��
		*tmp16 = year;
		tmp16 = (u16 *)&sdwbuff[6];  //��
		*tmp16 = month;
		tmp16 = (u16 *)&sdwbuff[8];  //��
		*tmp16 = tmpq.day;
		tmp16 = (u16 *)&sdwbuff[10];  //ʱ
		*tmp16 = tmpq.hour;
		tmp16 = (u16 *)&sdwbuff[12];  //��
		*tmp16 = tmpq.min;
		tmp16 = (u16 *)&sdwbuff[14];  //��
		*tmp16 = tmpq.sec;
		tmp16 = (u16 *)&sdwbuff[16];  //����
		*tmp16 = tmpq.ms;
		tmpf = (float *)&sdwbuff[18]; //ch0
		*tmpf = tmpq.ch[0] + 0.0;
		tmpf = (float *)&sdwbuff[22]; //ch1
		*tmpf = tmpq.ch[1] + 0.0;
		tmpf = (float *)&sdwbuff[26]; //ch2
		*tmpf = tmpq.ch[2] + 0.0;
		++numb;
		OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
		res = f_write(&sdfp,&sdwbuff,30,&dbw); //sizeof(sdwbuff),�˴�Ϊ��sdwbuffȡ��ַ
		OSMutexPost(&sdmutex,OS_OPT_POST_NONE,&err);  //�ͷ�sdmutux�ź���,�Ҳ��������
		//printf("write:%d ", res);
		if (numb % 2000 == 0)
		{
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
			res = f_sync(&sdfp); 
			OSMutexPost(&sdmutex,OS_OPT_POST_NONE,&err);  //�ͷ�sdmutux�ź���,�Ҳ��������
		}
		//if (full.Ctr >= 200) printf("sync:%d numb:%d Ctr:%d\r\n",res, numb, full.Ctr);
		if (numb >= 30000)
		{
			numb = 0;
			++numa;
			local_files_num++;
			res = 1;
			while (res)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
				res = f_close(&sdfp);
				OSMutexPost(&sdmutex,OS_OPT_POST_NONE,&err);  //�ͷ�sdmutux�ź���,�Ҳ��������
//				printf("FCLOSE:%d\r\n",res);
#if OLED_ENABLE
			sprintf((char*)oled_buf,"Wait:%02d Sent:%03d",local_files_num,sent_files_num);
			OLED_ShowString(0,6,(u8*)oled_buf);
			OLED_ShowString(0,3,(u8*)"Connected to FTP");
#endif
			}
			dbw = 0;
			printf("numa:%04d\r\n",numa);
		}
		if (numa >= 1000)
		{
			numa = 0;
			init = 0;
		}
	}
}


//ftp������
void ftp_task(void *p_arg)
{
	
	OS_ERR err; 
	OS_TaskSuspend((OS_TCB*)&UsbTaskTCB,&err);		//����FTP����
	OSSemPend(&sdinit,0,OS_OPT_PEND_BLOCKING,0,&err);  //�ȴ�full�ź���
//	static char lfn[_MAX_LFN * 2 + 1];
	u16 i;
	u8 j;
#if OLED_ENABLE
	extern u8 oled_buf[17];;
#endif
	USART2_Init(921600);
	//USART2_Init(115200);sim7600_send_cmd((u8 *)"AT+IPREX=921600",(u8 *)"OK",200);
	u8 res=1;
	USART2_TX_BUF[0] = 0x1A;

	
	USART2_TX_BUF[1] = '\r';
	USART2_TX_BUF[2] = '\n';
	SendUsart2TXBUF(3);
	
	
	while(res)
	{
		res=GSM_Dect();
		if (res != SIM_OK && res != SIM_CREG_FAIL)
		{
			printf("res:%d RX: %s\r\n", res, USART2_RX_BUF);
			OSTimeDlyHMSM(0,0,6,0,OS_OPT_TIME_HMSM_STRICT,&err);	//Add By Yao
			continue;
		}
		res=SIM7600_CONNECT_SERVER();
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err);
		printf("res:%d RX: %s\r\n", res, USART2_RX_BUF);

	}
#if OLED_ENABLE
	OLED_ShowString(0,3,(u8*)"Connected to FTP");
#endif
	printf("����FTP������ɣ�׼�����ݷ��ͣ�\r\n\r\n");
	/************����FTP���**********/
	FIL sdfp; //�ļ�ָ��
	//FILINFO fileinfo = {0};
	UINT br = 0;
	DIR dir;
	FRESULT fres = 1;
	u8 sdrbuff[30];
	u8 ctrlnum = 0;
	char *fn;
	u8 dpath[20];
	u8 ftppath[40];
	u16 t=0;//���ڼ�¼���¶�ȡ�ļ�����
	while(1)
	{
		fres = 1;
		while(fres)//ˢ��Ŀ¼
		{
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
			fres = f_opendir(&dir, "0:/");
			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
			if(fres) printf("opendir:%d\r\n", fres);
			OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err);
		}
		while(1) //��ȡĿ¼
		{
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
			fres = f_readdir(&dir, &fileinfo);
			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
			if (fres != FR_OK || fileinfo.fname[0] == 0)  //�ļ���ȡʧ�ܻ�ǺϷ��ļ�
				break;
			fn = fileinfo.fname;
			//printf("name:%s  size:%ld\r\n", fn, fileinfo.fsize);
			if (fileinfo.fsize == 30 || fileinfo.fsize == 900000) //������ļ�
			{
				t=0;
				break;
			}
			else
				continue;
		}
		if (fres != FR_OK || fileinfo.fname[0] == 0)
		{
			t++;
			//Add By Yao
			if(t%10==0)
			{
				if(t<=60)
				{
					printf("�ɼ���...\r\n");
					printf("FTP��������ȴ��ɼ����%d...\r\n\r\n",t/10);
#if OLED_ENABLE
					switch(t/10)
					{
						case 1: case 4:
							OLED_ShowString(0,3,(u8*)"   Sampling.    ");
							break;
						case 2: case 5:
							OLED_ShowString(0,3,(u8*)"   Sampling..   ");
							break;
						case 3: case 6:
							OLED_ShowString(0,3,(u8*)"   Sampling...  ");
							break;
						default:
							break;
					}				
#endif
				}
				else
				{
					printf("�ȴ�SD���ļ�������ʱ������ADS1256��GPS��\r\n\r\n");
					if(t>100)
					{
						printf("�ȴ���ʱ��׼�������λ��\r\n\r\n");
						soft_reset();//�����λ
					}
				}
			}
			OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err);
			continue;
		}
		
		//����FTP���䣬���Ƚ��м��SIM7600�Ļ�Ӧ���Ϻõؽ����ͨѶʧ������	Add By Yao 
		u8 t=0;
		while(sim7600_send_cmd((u8 *)"AT",(u8 *)"OK",100))
		{
			t++;
			if(t>200)
			{
				printf("�ȴ�SIM7600��Ӧ��ʱ,SIM7600���ڸ�λ�У�\r\n\r\n");	
#if OLED_ENABLE
				OLED_ShowString(0,3,(u8*)" 4G Timeout,RST ");
#endif
				SIM7600_Reset();
			}
		}
		printf("Uploading via FTP...%s \r\n", fn);
#if OLED_ENABLE
		sprintf((char*)oled_buf,"Send%s",fn);
		OLED_ShowString(0,3,(u8*)oled_buf);
#endif
		sprintf((char *)ftppath,"AT+CFTPPUT=\"/%d/%s\"",NUMBER, fn);	//Edit By Yao
		//�෢һ�ν����� Add By Yao
		USART2_TX_BUF[0] = 0x1A;
		SendUsart2TXBUF(1);
		USART2_RX_STA=0;
		res = sim7600_send_cmd((u8 *)ftppath,(u8 *)"BEGIN",10000);
		//��ʱ��λ���˴���FTP�������ٳٲ�����Ӧ������λAdd By Yao
		if(res==1)
		{
			printf("�ȴ�ftp��������Ӧ��ʱ,SIM7600���ڸ�λ�У�\r\n\r\n");	
#if OLED_ENABLE
			OLED_ShowString(0,3,(u8*)"FTP Timeout,RST ");
#endif
			SIM7600_Reset();
		}

		sprintf((char *)dpath,"0:/%s",fn);
		fres = 1;
		while (fres)
		{
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
			fres = f_open(&sdfp,(const char *)dpath,FA_OPEN_EXISTING | FA_READ);
			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
			printf("+FOPEN:%d\r\n",fres);
#if OLED_ENABLE
			OLED_ShowString(0,3,(u8*)"     Start      ");
#endif
			br = 0;
		}
		if (fileinfo.fsize == 30)
		{
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
			f_read(&sdfp, &sdrbuff, 30, &br);
			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
			for (i = 0, ctrlnum = 0; i < 30; ++i)
			{
				if (sdrbuff[i] == 0x03 || sdrbuff[i] == 0x1A)
				{
					USART2_TX_BUF[i + ctrlnum] = 0x03;
					ctrlnum++;
				}
				USART2_TX_BUF[i + ctrlnum] = sdrbuff[i];
			}
			USART2_TX_BUF[i + ctrlnum] = 0x1A;
			SendUsart2TXBUF(30 + ctrlnum + 1);
			fres = 1;
			while (fres)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
				fres = f_close(&sdfp);
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
				printf("+FCLOSE:%d\r\n",fres);
			}
			fres = 1;
			while (fres)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
				fres = f_unlink((const char *)dpath);
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
				local_files_num--;
				sent_files_num++;
				printf("+FUNLINK:%d\r\n",fres);
#if OLED_ENABLE
				sprintf((char*)oled_buf,"Wait:%02d Sent:%03d",local_files_num,sent_files_num);
				OLED_ShowString(0,6,(u8*)oled_buf);
				OLED_ShowString(0,3,(u8*)" Send completed ");
#endif
			}
		}
		else if (fileinfo.fsize == 900000)
		{
			u32 finish_size=0;
			for (i = 0; i < 30000; ++i)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
				fres = f_read(&sdfp, &sdrbuff, 30, &br);
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
				for (j = 0, ctrlnum = 0; j < 30; ++j)
				{
					if (sdrbuff[j] == 0x03 || sdrbuff[j] == 0x1A)  //0x1AΪ������־,���0x03ǰ׺
					{
						USART2_TX_BUF[j + ctrlnum] = 0x03;
						ctrlnum++;
					}
					USART2_TX_BUF[j + ctrlnum] = sdrbuff[j];
				}
				SendUsart2TXBUF(30 + ctrlnum);
				//��ʾ������ɰٷֱȣ�Add By Yao
				finish_size +=30;
				if(finish_size%45000==0)
				{
					printf("�����:%d%%\r\n",finish_size/45000*5);
#if OLED_ENABLE
					sprintf((char*)oled_buf,"  Finished:%02d%% ",finish_size/45000*5);
					OLED_ShowString(0,3,(u8*)oled_buf);
#endif
				}
				/*
					USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���2��DMA����     
					DMAx_Enable(DMA1_Stream6,30 + ctrlnum);     //��ʼһ��DMA���䣡
					while(1)
					{
						if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=0)//�ж�DMA������6�Ƿ������
						{
							DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);
							break;
						}
					}
				*/
			}
			//���ͽ�����
			USART2_TX_BUF[0] = 0x1A;
			SendUsart2TXBUF(1);
			//�ȴ��Ͽ����ӳɹ� Add By Yao
//			u8 t=0;
//			while(!sim7600_check_cmd((u8*)"OK"))
//			{
//				t++;
//				OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
//				if(t>20)
//				{
//					t=0;
//					printf("��������Ͽ����ӳ�ʱ��\r\n");
//				}
//			}
			
			fres = 1;
			while (fres)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
				fres = f_close(&sdfp);
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
				printf("+FCLOSE:%d\r\n",fres);
			}
			fres = 1;
			while (fres)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
				fres = f_unlink((const char *)dpath);
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
				printf("+FUNLINK:%d\r\n",fres);
				local_files_num--;
				sent_files_num++;
#if OLED_ENABLE
				sprintf((char*)oled_buf,"Wait:%02d Sent:%03d",local_files_num,sent_files_num);
				OLED_ShowString(0,6,(u8*)oled_buf);
				OLED_ShowString(0,3,(u8*)" Send completed ");
#endif
			}
		}
//		if(SwitchFtptoFlash)
//		{
//			printf("�л���U��ģʽ\r\n\r\n");
//			OS_TaskResume((OS_TCB*)&UsbTaskTCB,&err);			//���USB����
//			OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err); //�����������
//		}
	}
}

//usb������
//д�뵽U���ٶȱȽ������ﲻ���������ٶȣ����Բ�ͬU��д���ٶȲ�ͬ��Ŀǰ������70KB/s���ɳ��԰�����д�뷽ʽ
void usb_task(void *p_arg)
{
	OS_ERR err; 
//	OS_TaskSuspend((OS_TCB*)&FtpTaskTCB,&err);		//����FTP����
	OSSemPend(&sdinit,0,OS_OPT_PEND_BLOCKING,0,&err);  //�ȴ�full�ź���
	printf("USB����ʼ\r\n");

	u16 i;
	u8 s,t;
	FIL sdfp; //�ļ�ָ��
	//FILINFO fileinfo = {0};
	UINT br = 0;
	DIR dir;
	FRESULT fres = 1;
	u8 sdrbuff[1020];

	char *fn;
	u8 sdpath[20];
	u8 upath[20];
	u8 oled_buf[17];
	
	s = mInitCH376Host( );    // ��ʼ��CH376

	//ch376 hostģʽ��ʼ��ʧ�ܱ�־��led1��˸
	if(s == ERR_USB_UNKNOWN)
	{
		printf("CH376��ʼ��ʧ��\r\n");
		//�Զ��л���FTP����
		OS_TaskSuspend((OS_TCB*)&UsbTaskTCB,&err);		//����USB����
		OS_TaskResume((OS_TCB*)&FtpTaskTCB,&err);			//���FTP����
		printf("�洢��ʽ��U���л�ΪFTP\r\n\r\n");
	}
	
	//��ʼ���ɹ�����ʼ����usb
//  printf( "�ȴ�U�̲���\r\n" );
	//----- usb���� -----
	//�ȼ��һ��,����ÿ������ʾδ����U��
	CH376DiskConnect();
	OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err);
  while ( CH376DiskConnect() != USB_INT_SUCCESS )/* ���U���Ƿ�����,�ȴ�U�̲���,����SD��,�����ɵ�Ƭ��ֱ�Ӳ�ѯSD�����Ĳ��״̬���� */
  {  
		printf("δ����U��\r\n\r\n");
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err);
  }
	printf("��⵽U��\r\n\r\n");
	
	while(1)
	{
		//���U��
		u8 times=0;
		while ( CH376DiskConnect() != USB_INT_SUCCESS )/* ���U���Ƿ�����,�ȴ�U�̲���,����SD��,�����ɵ�Ƭ��ֱ�Ӳ�ѯSD�����Ĳ��״̬���� */
		{  
			if(times%50==0)printf("δ����U��\r\n\r\n");
			times++;
			OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err);
		}
		if(times)printf("��⵽U��\r\n\r\n");//����֮ǰδ��⵽U��Ȼ���⵽U�̺����ʾ"��⵽U��"		
		//ˢ��Ŀ¼
		fres=1;
		while(fres)//ˢ��Ŀ¼
		{
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
			fres = f_opendir(&dir, "0:/");
			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
			if(fres) printf("opendir:%d\r\n", fres);
			OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err);
		}
		//��ȡĿ¼
		while(1)
		{
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
			fres = f_readdir(&dir, &fileinfo);
			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
			if (fres != FR_OK || fileinfo.fname[0] == 0)  //�ļ���ȡʧ�ܻ�ǺϷ��ļ�
				break;
			fn = fileinfo.fname;
			//printf("name:%s  size:%ld\r\n", fn, fileinfo.fsize);
			if (fileinfo.fsize == 30 || fileinfo.fsize == 900000) //������ļ�
			{
				t=0;
				break;
			}
			else
				continue;
		}
		if (fres != FR_OK || fileinfo.fname[0] == 0)
		{
			t++;
			//Add By Yao
			if(t%10==0)
			{
				if(t<=70)
				{
					printf("�ɼ���...\r\n");
					printf("Copy����ȴ��ɼ����%d...\r\n\r\n",t/10);
#if OLED_ENABLE
					switch(t/10)
					{
						case 1: case 4:case 7://����7˵���Ѿ���������
							OLED_ShowString(0,3,(u8*)"   Sampling.    ");
							break;
						case 2: case 5:
							OLED_ShowString(0,3,(u8*)"   Sampling..   ");
							break;
						case 3: case 6:
							OLED_ShowString(0,3,(u8*)"   Sampling...  ");
							break;
						default:
							break;
					}				
#endif
				}
				else
				{
					printf("�ȴ�SD���ļ�������ʱ������ADS1256��GPS��\r\n\r\n");
					if(t>100)
					{
						printf("�ȴ���ʱ��׼�������λ��\r\n\r\n");
						soft_reset();//�����λ
					}
				}
			}
			OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err);
			continue;
		}
		printf("Copy To Flash Disk...%s \r\n", fn);
#if OLED_ENABLE
		sprintf((char*)oled_buf,"Send%s",fn);
		OLED_ShowString(0,3,(u8*)oled_buf);
#endif
		sprintf((char *)upath,"/%d/%s",NUMBER, fn);//U��Ŀ¼
		sprintf((char *)sdpath,"0:/%s",fn);//SD��Ŀ¼
		fres = 1;
		while (fres)
		{
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
			fres = f_open(&sdfp,(const char *)sdpath,FA_OPEN_EXISTING | FA_READ);
			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
			s = CH376FileCreatePath((PUINT8)upath);
			printf("+FOPEN:%d\r\n",fres);
#if OLED_ENABLE
			OLED_ShowString(0,3,(u8*)"     Start      ");
#endif
			br = 0;
		}
		if (fileinfo.fsize == 30)
		{
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
			f_read(&sdfp, &sdrbuff, 30, &br);
			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
			CH376ByteWrite((u8*)sdrbuff,30, NULL); //��U��д���ļ��������Ƿ�����жϷ���ֵ����
			CH376FileClose(TRUE);//�ر�U���ļ�
			
			fres = 1;
			while (fres)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
				fres = f_close(&sdfp);//�ر�SD���ļ�
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
				printf("+FCLOSE:%d\r\n",fres);
			}
			if(CH376DiskConnect() != USB_INT_SUCCESS) continue;//��������ж�ʧU�̲��ɼ���ִ��ɾ����ǰ�����ļ��������������ݶ�ʧ
			fres = 1;
			while (fres)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
				fres = f_unlink((const char *)sdpath);//ɾ����SD���п�����ɵ��ļ�
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
				local_files_num--;
				sent_files_num++;//���Ǹ���Ϊͳ��U���ļ�������ȫ�ֱ���
				printf("+FUNLINK:%d\r\n",fres);
#if OLED_ENABLE
				sprintf((char*)oled_buf,"Wait:%02d Copy:%03d",local_files_num,sent_files_num);
				OLED_ShowString(0,6,(u8*)oled_buf);
				OLED_ShowString(0,3,(u8*)" Copy completed ");
#endif
			}
		}
		else if (fileinfo.fsize == 900000)
		{
			u32 finish_size=0;
			for (i = 0; i < 900; ++i)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
				//��SD���ļ��ж�ȡ���ݷŵ���������һ�ζ�25����40�ζ��꣬Ȼ��һ����д��U��
				for(u8 j=0;j<40;j++)
				{
					fres = f_read(&sdfp, &sdrbuff+j*25, 25, &br);
				}
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
				s=CH376ByteWrite((u8*)sdrbuff,1000, NULL); //��U��д���ļ��������Ƿ�����жϷ���ֵ����
				//��ʾ������ɰٷֱȣ�Add By Yao
				finish_size +=1000;
				if(finish_size%45000==0)
				{
					printf("�����:%d%%\r\n",finish_size/45000*5);
#if OLED_ENABLE
					sprintf((char*)oled_buf,"  Finished:%02d%% ",finish_size/45000*5);
					OLED_ShowString(0,3,(u8*)oled_buf);
#endif
				}
			}
			s=CH376FileClose(TRUE);//�ر�U���ļ�
			fres = 1;
			while (fres)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
				fres = f_close(&sdfp);
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
				printf("+FCLOSE:%d\r\n",fres);
			}
			if(CH376DiskConnect() != USB_INT_SUCCESS) continue;//��������ж�ʧU�̲��ɼ���ִ��ɾ����ǰ�����ļ��������������ݶ�ʧ
			fres = 1;
			while (fres)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ�sdmutex�ź���
				fres = f_unlink((const char *)sdpath);
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //�ͷ�sdmutux�ź������Ҿ�������̬����
				printf("+FUNLINK:%d\r\n",fres);
				local_files_num--;
				sent_files_num++;
#if OLED_ENABLE
				sprintf((char*)oled_buf,"Wait:%02d Copy:%03d",local_files_num,sent_files_num);
				OLED_ShowString(0,6,(u8*)oled_buf);
				OLED_ShowString(0,3,(u8*)" Send completed ");
#endif
			}
		}
	}
//	if(!SwitchFtptoFlash)
//	{
//		printf("�л�FTPģʽ\r\n\r\n");
//		OS_TaskResume((OS_TCB*)&FtpTaskTCB,&err);			//����USB����
//		OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);
//	}
//                                //��������DISK_MOUNTED��,���ȴ�5*50mS,��Ҫ���DiskReady������
//  for ( i = 0; i < 100; i ++ )
//	{	
//		delay_ms( 50 );
//		s = CH376DiskMount( );      //��ʼ�����̲����Դ����Ƿ����.
//		if ( s == USB_INT_SUCCESS ) // ׼����
//			break;                                          
//		else if ( s == ERR_DISK_DISCON )    // ��⵽�Ͽ�,���¼�Ⲣ��ʱ
//			break;  
//		if ( CH376GetDiskStatus( ) >= DEF_DISK_MOUNTED && i >= 5 ) // �е�U�����Ƿ���δ׼����,�������Ժ���,ֻҪ�佨������MOUNTED�ҳ���5*50mS
//			break; 
//	}          
//	if ( s == ERR_DISK_DISCON )   // ��⵽�Ͽ�,���¼�Ⲣ��ʱ
//	{  
//		printf( "�豸�γ�\r\n" );
//	  //continue;
//	}		
//	if ( CH376GetDiskStatus( ) < DEF_DISK_MOUNTED ) // δ֪USB�豸,����USB���̡���ӡ����
//	{  
//		printf( "δ֪�豸\r\n" );
//	}
//	
//	
//	//----- ��ȡ������Ϣ -----
//	i = CH376ReadBlock( wbuf );  /* �����Ҫ,���Զ�ȡ���ݿ�CH376_CMD_DATA.DiskMountInq,���س��� */
//	if ( i)  /* U�̵ĳ��̺Ͳ�Ʒ��Ϣ */
//	{  
//		wbuf[ i ] = 0;
//		printf( "U����Ϣ: %s \r\n", ((P_INQUIRY_DATA)wbuf) -> VendorIdStr );
//	}
////	printf( "DiskQuery:  " );		/* ���U�̻���SD����ʣ��ռ� */
////	s = CH376DiskQuery( (PUINT32)wbuf );	/* ��ѯ����ʣ��ռ���Ϣ,������ */
////	printf("s=%02x \r\n",(unsigned short)s );
////	printf( "ʣ��ռ䣺 %ld MB\r\n", *(PUINT32)wbuf / ( 1000000 / DEF_SECTOR_SIZE ) );
//	
//	
//////////////////////////////////----- 30�ֽ��ļ� -----////////////////////////////////////////
//	

//	//----- �����ļ�file30 -----
//	printf("�����ļ�  ");
//	strcpy((char *)TarName, "/A181121.TXT"); // Ŀ���ļ���
//	s = CH376FileCreatePath(TarName);      	 // �½��༶Ŀ¼�µ��ļ�,֧�ֶ༶Ŀ¼·��,���뻺����������RAM��
//	printf("s=%02x \r\n",(unsigned short)s);    // 14�������ɹ�

//	//----- д���ļ� -----
//	printf("д���ļ�  ");
//	s = CH376ByteWrite((u8*)buf30,30, NULL);             // ���ֽ�Ϊ��λ��ǰλ��д�����ݿ�
//	printf("s=%02x \r\n",(unsigned short)s);                                 // 14����д��ɹ�
//	
//	//----- �ر��ļ� -----
//	printf("�ر��ļ�  "); 
//	  s = CH376FileClose(TRUE);               // �ر��ļ�,�����ֽڶ�д�����Զ������ļ�����
//	printf("s=%02x \r\n",(unsigned short)s);
//  delay_ms(500);


//	//���ļ�
//	printf("���ļ�  ");
//		s = CH376FileOpenPath(TarName);      	 // �½��༶Ŀ¼�µ��ļ�,֧�ֶ༶Ŀ¼·��,���뻺����������RAM��
//	printf("s=%02x \r\n",(unsigned short)s);    // 14�������ɹ�

//	//���ļ�
//	printf("��ȡ�ļ�  ");
//	s = CH376ByteRead(rbuf,30,NULL);
//	printf("s=%02x \r\n",(unsigned short)s);    // 14�������ɹ�
//	//for (i = 0; i < 30; ++i) printf("0x%02X ", rbuf[i]);
//	printf("\r\n");
//	printf("�ļ����ݣ�%s\r\n",rbuf);
//	delay_ms(500);
//		
//		//----- �ر��ļ� -----
//	printf("�ر��ļ�  "); 
//	s = CH376FileClose(TRUE);               // �ر��ļ�,�����ֽڶ�д�����Զ������ļ�����
//	printf("s=%02x \r\n",(unsigned short)s);
//	
//	
////////////////////////////////////////----- 220*1500�ֽ��ļ� -----//////////////////////////////////////////
//	//----- �����ļ�file22 -----
//	printf("�����ļ�: ");
//	strcpy((char *)TarName, "/DAWENJ.TXT");     // Ŀ���ļ���
//	s = CH376FileCreatePath(TarName);      	    // �½��༶Ŀ¼�µ��ļ�,֧�ֶ༶Ŀ¼·��,���뻺����������RAM��
//	printf("s=%02X \r\n",(unsigned short)s);    // 14�������ɹ�
//	for(j = 0; j < 100; j++)
//	{
//			//----- �������� -----
//		for(t = 0; t < 22; t++)
//			buffer[t + j * 22] = buf22[t];
//	}	
//	//----- д���ļ� -----
//	printf("д���ļ�:  ");
//	for(k = 0; k < 3000; k++)
//	{
//		s = CH376ByteWrite(buffer, 2100, NULL);             // ���ֽ�Ϊ��λ��ǰλ��д�����ݿ�
//		//ÿ2000�γ�ˢһ��
//	}	
//	printf("s=%02X \r\n",(unsigned short)s);                                 // 14����д��ɹ�
//	printf("�ļ���С: %ld\r\n",CH376GetFileSize());
//	
//	//----- �ر��ļ� -----
//	printf("�ر��ļ�:  "); 
//	s = CH376FileClose(TRUE);   // �ر��ļ�,�����ֽڶ�д�����Զ������ļ�����
//	printf("s=%02X \r\n",(unsigned short)s);

//	printf("----- ok -----\r\n");
//	
//	while (1);
}


//monitor������
void monitor_task(void *p_arg)
{
	OS_ERR err;
	CPU_STK_SIZE  n_free;
  CPU_STK_SIZE  n_used;
	//IWDG_Init(5,500);
	while (1)
	{
		//IWDG_FeedDog();
		OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err);
		
		OSTaskStkChk(&ReadTaskTCB,&n_free,&n_used,&err);
		printf("Read:Free:%d Used:%d\r\n", n_free, n_used);
		OSTaskStkChk(&GpsTaskTCB,&n_free,&n_used,&err);
		printf("Gps:Free:%d Used:%d\r\n", n_free, n_used);
		OSTaskStkChk(&FileTaskTCB,&n_free,&n_used,&err);
		printf("File:Free:%d Used:%d\r\n", n_free, n_used);
		OSTaskStkChk(&FtpTaskTCB,&n_free,&n_used,&err);
		printf("Ftp:Free:%d Used:%d\r\n", n_free, n_used);
		OSTaskStkChk(&MonitorTaskTCB,&n_free,&n_used,&err);
		printf("Monitor:Free:%d Used:%d\r\n", n_free, n_used);
		
	}
}

