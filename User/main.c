#include "system.h"
#include "SysTick.h"
#include "includes.h"
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

#define NUMBER 5
//任务优先级
#define START_TASK_PRIO		3
//任务堆栈大小	
#define START_STK_SIZE 		128
//任务控制块
OS_TCB StartTaskTCB;
//任务堆栈	
CPU_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *p_arg);

//任务优先级
#define READ_TASK_PRIO		4
//任务堆栈大小	
#define READ_STK_SIZE 		96
//任务控制块
OS_TCB ReadTaskTCB;
//任务堆栈	
CPU_STK READ_TASK_STK[READ_STK_SIZE];
void read_task(void *p_arg);


//任务优先级
#define GPS_TASK_PRIO		4
//任务堆栈大小	
#define GPS_STK_SIZE 		64
//任务控制块
OS_TCB GpsTaskTCB;
//任务堆栈	
CPU_STK GPS_TASK_STK[GPS_STK_SIZE];
void gps_task(void *p_arg);


//任务优先级
#define FILE_TASK_PRIO		5
//任务堆栈大小	
#define FILE_STK_SIZE 		330
//任务控制块
OS_TCB FileTaskTCB;
//任务堆栈	
CPU_STK FILE_TASK_STK[FILE_STK_SIZE];
void file_task(void *p_arg);

//任务优先级
#define FTP_TASK_PRIO		6
//任务堆栈大小	
#define FTP_STK_SIZE 		330
//任务控制块
OS_TCB FtpTaskTCB;
//任务堆栈	
CPU_STK FTP_TASK_STK[FTP_STK_SIZE];
void ftp_task(void *p_arg);

//任务优先级
#define USB_TASK_PRIO		6
//任务堆栈大小	
#define USB_STK_SIZE 		330
//任务控制块
OS_TCB UsbTaskTCB;
//任务堆栈	
CPU_STK USB_TASK_STK[USB_STK_SIZE];
void usb_task(void *p_arg);

//任务优先级
#define MONITOR_TASK_PRIO		7
//任务堆栈大小	
#define MONITOR_STK_SIZE 		96
//任务控制块
OS_TCB MonitorTaskTCB;
//任务堆栈	
CPU_STK MONITOR_TASK_STK[MONITOR_STK_SIZE];
void monitor_task(void *p_arg);

OS_MUTEX mutex; //互斥信号量
OS_SEM full; //多值信号量
OS_SEM empty; //多值信号量
OS_SEM sdinit; //多值信号量
OS_MUTEX sdmutex; //多值信号量
OS_SEM read; //多值信号量

int main()
{  	
	OS_ERR err;
	delay_init();
	USART1_Init(921600);
	USART3_Init(38400);		
	printf("串口还活着\r\n");
	bsp_InitADS1256();
	SIM7600_Init();
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
		printf("读取芯片ID\r\n");
		if (id0 != 3 || id1 != 3 || id2 != 3)
		{
			printf("Error, ASD1256 Chip ID = 0x%02X & 0x%02X & 0x%02X\r\n", id0, id1, id2);
		}
		else
		{
			printf("Ok, ASD1256 Chip ID = 0x%02X & 0x%02X & 0x%02X\r\n", id0, id1, id2);
			break;
		}
	}
//***************************************************************************
	OSInit(&err);		//初始化UCOSIII
	//创建开始任务
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//任务控制块
				 (CPU_CHAR	* )"start task", 		//任务名字
                 (OS_TASK_PTR )start_task, 			//任务函数
                 (void		* )0,					//传递给任务函数的参数
                 (OS_PRIO	  )START_TASK_PRIO,     //任务优先级
                 (CPU_STK   * )&START_TASK_STK[0],	//任务堆栈基地址
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)START_STK_SIZE,		//任务堆栈大小
                 (OS_MSG_QTY  )0,					//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,					//当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void   	* )0,					//用户补充的存储区
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                 (OS_ERR 	* )&err);				//存放该函数错误时的返回值 
	OSStart(&err);  //开启UCOSIII
	while(1);
}

//开始任务函数
void start_task(void *p_arg)
{
	printf("start task\r\n");
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
  OSStatTaskCPUUsageInit(&err);  	//统计任务                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//如果使能了测量中断关闭时间
  CPU_IntDisMeasMaxCurReset();	
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //当使用时间片轮转的时候
	 //使能时间片轮转调度功能,时间片长度为1个系统时钟节拍，既1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif		
	
	OS_CRITICAL_ENTER();	//进入临界区
	
	//创建互斥信号量 mutex 缓冲区互斥锁
  OSMutexCreate((OS_MUTEX *)&mutex,    //指向信号量变量的指针
               (CPU_CHAR    *)"mutex",    //信号量的名字
               (OS_ERR      *)&err);         //错误类型
	//创建多值信号量 full 缓冲区数据数量
  OSSemCreate((OS_SEM      *)&full,    //指向信号量变量的指针
               (CPU_CHAR    *)"full",    //信号量的名字
               (OS_SEM_CTR   )0,             //信号量这里是现有的资源数目
               (OS_ERR      *)&err);         //错误类型
	//创建多值信号量 empty 缓冲区空位
  OSSemCreate((OS_SEM      *)&empty,    //指向信号量变量的指针
               (CPU_CHAR    *)"empty",    //信号量的名字
               (OS_SEM_CTR   )QSIZE,             //信号量这里是现有的资源数目
               (OS_ERR      *)&err);         //错误类型	
	//创建多值信号量 sdinit sd卡初始化
  OSSemCreate((OS_SEM      *)&sdinit,    //指向信号量变量的指针
               (CPU_CHAR    *)"sdinit",    //信号量的名字
               (OS_SEM_CTR   )0,             //信号量这里是现有的资源数目
               (OS_ERR      *)&err);         //错误类型			
	//创建互斥信号量 sdmutex sd卡互斥锁
	OSMutexCreate((OS_MUTEX *)&sdmutex,    //指向信号量变量的指针
               (CPU_CHAR    *)"sdmutex",    //信号量的名字
               (OS_ERR      *)&err);         //错误类型			
	//创建多值信号量 read 读ads1282
  OSSemCreate((OS_SEM      *)&read,    //指向信号量变量的指针
               (CPU_CHAR    *)"read",    //信号量的名字
               (OS_SEM_CTR   )0,             //信号量这里是现有的资源数目
               (OS_ERR      *)&err);         //错误类型					
			
	//创建CS5376A任务
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
							
	//创建gps任务
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
						 						
	//创建file任务
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
			
	//创建ftp任务
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
			/*
	//创建USB任务							 
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

	//创建monitor任务
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
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//挂起开始任务			 
	OS_CRITICAL_EXIT();	//进入临界区
}

//cs5376a任务函数
void read_task(void *p_arg)
{
	printf("read task alive\r\n");
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
		//OSSemPend(&read,0,OS_OPT_PEND_BLOCKING,0,&err);  //等read号量
		OSSchedLock(&err); //禁止进程调度
		
		tmpms = TIM5->CNT;
		tmpsec = sec;
		tmpmin = min;
		tmphour = hour;
		tmpday = day;
		
		count++;
		ADS1256_ISR();//定时采集
		OSSchedUnlock(&err); //恢复进程调度
		for (i = 0; i < 3; ++i)
		{
			temp = ADS1256_GetAdc(i); //从全局缓冲区读取采样结果。 采样结果是在中断服务程序中读取的
/*校准ADC,更换AD板需要重新校准		Add By Yao																						*/
#if NUMBER==1
			volt[0] = ((int64_t)temp * 2500000) / 4220581;	//校准方法：使用AD采集2.5V标准电压，得到v1
			volt[1] = ((int64_t)temp * 2500000) / 4221009;	//满足关系：v1/2.5=原值/新值
			volt[2] = ((int64_t)temp * 2500000) / 4220475;
#else
			volt[i] = ((int64_t)temp * 2500000) / 4194303;	// 计算实际电压值（原值，近似估算的），4194303 = 2.5V
#endif
			

		}
		OSSemPend(&empty,0,OS_OPT_PEND_BLOCKING,0,&err);  //等待empty信号量
		OSMutexPend(&mutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待mutux信号量
		//printf("ch1:%d ch2:%d ch3%d\r\n", volt[0], volt[1], volt[2]);
		qdata[qrear].ch[0] = (float)((double)volt[0] / 1000000.0);
		qdata[qrear].ch[1] = (float)((double)volt[1] / 1000000.0);
		qdata[qrear].ch[2] = (float)((double)volt[2] / 1000000.0);
//		printf("%f\r\n",qdata[qrear].ch[1]);
		qdata[qrear].day = day;
		qdata[qrear].sec = tmpsec;
		qdata[qrear].min = tmpmin;
		qdata[qrear].hour = tmphour;
		qdata[qrear].ms = tmpms / 10;
		qrear = (qrear + 1) % QSIZE;
		
		OSMutexPost(&mutex,OS_OPT_POST_FIFO,&err); 
		OSSemPost(&full,OS_OPT_POST_FIFO,&err); //释放full信号量
		if (full.Ctr >= 400) printf("Warning: %d Buffer\r\n", full.Ctr);
		
	}
	
}

//gps任务函数
void gps_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	u32 t=10000000;
	TIM5_Init(10000-1, 8394-1); //8393-1 10-1
	TIM2_Init(10000-1, 8500-1); //用于检测GPS状态
	while(!(USART3_RX_STA&0X8000)) //Edit By Yao 检测GPS通讯情况
	{
		t--;
		if(t==0)
		{
			printf("GPS通讯异常！\r\n");
			t=10000000;
		}
	}
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
 		}	   
		else
			OSTimeDlyHMSM(0,0,0,40,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

void file_task(void *p_arg)
{
	OS_ERR err;
	FRESULT res;
	my_mem_init(SRAMIN);		//初始化内部内存池
	//SD卡初始化
	while(SD_Init()!=0)
	{	
		OSTimeDlyHMSM(0,0,0,4,OS_OPT_TIME_HMSM_STRICT,&err);
	}
	FATFS_Init();							//为sd卡数据缓冲区fatfs、文件1、文件2、各磁盘工作区相关变量申请内存		
  f_mount(fs[0],"0:",1); 					//挂载SD卡，使用fs[0]磁盘工作区0	
	OSSemPost(&sdinit,OS_OPT_POST_FIFO,&err);
	FIL sdfp; //head文件指针
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
//	IWDG_Init(5,625); //看门狗初始化
	IWDG_Init(5,1200); //增加看门狗时间，用于解决启动时反复复位的问题，Edit By Yao
	while(1)
	{
		OSSemPend(&full,0,OS_OPT_PEND_BLOCKING,0,&err);  //等待full信号量
		OSMutexPend(&mutex,0,OS_OPT_PEND_BLOCKING,0,&err);   //等待mutux信号量	
		queue tmpq = qdata[qfront];
		qfront = (qfront + 1) % QSIZE;
		OSMutexPost(&mutex,OS_OPT_POST_FIFO,&err);  //释放mutux信号量
		OSSemPost(&empty,OS_OPT_POST_FIFO,&err);
		if (full.Ctr <= QSIZE - 100)
		IWDG_FeedDog(); //喂狗
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
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
				res = f_open(&sdfp,(const char *)hpath,FA_CREATE_ALWAYS | FA_WRITE);
				OSMutexPost(&sdmutex,OS_OPT_POST_NONE,&err);  //释放sdmutux信号量,且不引起调度
//				printf("FOPEN:%d\r\n",res);
			}
			LBlocate(&gpsx);
			tmp16 = (u16 *)&sdwbuff[0];  //站点名
			*tmp16 = NUMBER;
			tmp16 = (u16 *)&sdwbuff[2];  //采样间隔
			*tmp16 = 4;
			tmpf = (float *)&sdwbuff[4]; //站点坐标X
			*tmpf = gpsx.longitude / 100000.0;
			tmpf = (float *)&sdwbuff[8]; //站点坐标Y
			*tmpf = gpsx.latitude / 100000.0;
			tmpf = (float *)&sdwbuff[12];//站点高度
			*tmpf = gpsx.altitude / 10.0;
			tmp16 = (u16 *)&sdwbuff[16]; //年
			*tmp16 = year;
			tmp16 = (u16 *)&sdwbuff[18]; //月
			*tmp16 = month;
			tmp16 = (u16 *)&sdwbuff[20]; //日
			*tmp16 = tmpq.day;
			tmp16 = (u16 *)&sdwbuff[22]; //时
			*tmp16 = tmpq.hour;
			tmp16 = (u16 *)&sdwbuff[24]; //分
			*tmp16 = tmpq.min;
			tmp16 = (u16 *)&sdwbuff[26]; //秒
			*tmp16 = tmpq.sec;
			tmp16 = (u16 *)&sdwbuff[28]; //毫秒
			*tmp16 = tmpq.ms;
			
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
			res = f_write(&sdfp,&sdwbuff,sizeof(sdwbuff),&hbw);  
			OSMutexPost(&sdmutex,OS_OPT_POST_NONE,&err);  //释放sdmutux信号量,且不引起调度
			//res = f_write(&sdfp,&sdwbuff,sizeof(sdwbuff),&dbw); 
//			printf("head:%d\r\n",res);
			res = 1;
			while (res)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
				res = f_close(&sdfp);
				OSMutexPost(&sdmutex,OS_OPT_POST_NONE,&err);  //释放sdmutux信号量,且不引起调度
//				printf("FCLOSE:%d\r\n",res);
			}
			hbw = 0;
			init = 1;
			numa = 0;
			numb = 0;
		}
		if (numb == 0)
		{
			sprintf((char *)dpath,"%s.%03d",fname, numa);
			res = 1;
			while (res)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
				res = f_open(&sdfp,(const char *)dpath,FA_CREATE_ALWAYS | FA_WRITE);
				OSMutexPost(&sdmutex,OS_OPT_POST_NONE,&err);  //释放sdmutux信号量,且不引起调度
//				printf("FOPEN:%d\r\n",res);
			}			
		}
		tmp16 = (u16 *)&sdwbuff[0]; //序号
		*tmp16 = numb;
		tmp16 = (u16 *)&sdwbuff[2];  //采样方式
		*tmp16 = 3;
		tmp16 = (u16 *)&sdwbuff[4];  //年
		*tmp16 = year;
		tmp16 = (u16 *)&sdwbuff[6];  //月
		*tmp16 = month;
		tmp16 = (u16 *)&sdwbuff[8];  //日
		*tmp16 = tmpq.day;
		tmp16 = (u16 *)&sdwbuff[10];  //时
		*tmp16 = tmpq.hour;
		tmp16 = (u16 *)&sdwbuff[12];  //分
		*tmp16 = tmpq.min;
		tmp16 = (u16 *)&sdwbuff[14];  //秒
		*tmp16 = tmpq.sec;
		tmp16 = (u16 *)&sdwbuff[16];  //毫秒
		*tmp16 = tmpq.ms;
		tmpf = (float *)&sdwbuff[18]; //ch0
		*tmpf = tmpq.ch[0] + 0.0;
		tmpf = (float *)&sdwbuff[22]; //ch1
		*tmpf = tmpq.ch[1] + 0.0;
		tmpf = (float *)&sdwbuff[26]; //ch2
		*tmpf = tmpq.ch[2] + 0.0;
		++numb;
		OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
		res = f_write(&sdfp,&sdwbuff,30,&dbw); //sizeof(sdwbuff)
		OSMutexPost(&sdmutex,OS_OPT_POST_NONE,&err);  //释放sdmutux信号量,且不引起调度
		//printf("write:%d ", res);
		if (numb % 2000 == 0)
		{
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
			res = f_sync(&sdfp); 
			OSMutexPost(&sdmutex,OS_OPT_POST_NONE,&err);  //释放sdmutux信号量,且不引起调度
		}
		//if (full.Ctr >= 200) printf("sync:%d numb:%d Ctr:%d\r\n",res, numb, full.Ctr);
		if (numb >= 30000)
		{
			numb = 0;
			++numa;
			res = 1;
			while (res)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
				res = f_close(&sdfp);
				OSMutexPost(&sdmutex,OS_OPT_POST_NONE,&err);  //释放sdmutux信号量,且不引起调度
//				printf("FCLOSE:%d\r\n",res);
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

//ftp任务函数
void ftp_task(void *p_arg)
{
	OS_ERR err; 
	OSSemPend(&sdinit,0,OS_OPT_PEND_BLOCKING,0,&err);  //等待full信号量
	static char lfn[_MAX_LFN * 2 + 1];
	u16 i;
	u8 j;
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
			continue;
		}
		res=SIM7600_CONNECT_SERVER();
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err);
		printf("res:%d RX: %s\r\n", res, USART2_RX_BUF);

	}
	printf("设置FTP配置完成，准备数据发送！\r\n\r\n");
	/************连接FTP完成**********/
	FIL sdfp; //文件指针
	//FILINFO fileinfo = {0};
	UINT br = 0;
	//DIR dir;
	FRESULT fres = 1;
	u8 sdrbuff[30];
	u8 ctrlnum = 0;
	char *fn;
	u8 dpath[20];
	u8 ftppath[40];
	u16 t=0;//用于记录重新读取文件次数
	while(1)
	{
		fres = 1;
		while(fres)//刷新目录
		{
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
			fres = f_opendir(&dir, "0:/");
			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
			if(fres) printf("opendir:%d\r\n", fres);
			OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err);
		}
		while(1) //读取目录
		{
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
			fres = f_readdir(&dir, &fileinfo);
			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
			if (fres != FR_OK || fileinfo.fname[0] == 0)  //文件读取失败或非合法文件
				break;
			fn = fileinfo.fname;
			//printf("name:%s  size:%ld\r\n", fn, fileinfo.fsize);
			if (fileinfo.fsize == 30 || fileinfo.fsize == 900000) //已完成文件
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
					printf("采集中...\r\n");
					printf("FTP发送任务等待SD卡文件就绪-%d...\r\n\r\n",t/10);
				}
				else
				{
					printf("等待SD卡文件就绪超时，请检查ADS1256和GPS！\r\n\r\n");
				}
			}
			OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err);
			continue;
		}
		//开启FTP传输，首先进行检测SIM7600的回应，较好地解决了通讯失败问题	Add By Yao 
		u8 t=0;
		while(sim7600_send_cmd((u8 *)"AT",(u8 *)"OK",100))
		{
			t++;
			if(t>200)
			{
				printf("等待SIM7600回应超时,SIM7600正在复位中！\r\n\r\n");	
				SIM7600_Reset();
			}
		}
		printf("Uploading via FTP...%s \r\n", fn);
//		sprintf((char *)ftppath,"AT+CFTPPUT=\"/%d/%s\"",NUMBER, fn);
		sprintf((char *)ftppath,"AT+CFTPPUT=\"/%d/%s\"",NUMBER, fn);	//Edit By Yao
//			res = sim7600_send_cmd((u8 *)ftppath,(u8 *)"BEGIN",6000);	//Edit By Yao
		//多发一次结束符
		USART2_TX_BUF[0] = 0x1A;
		SendUsart2TXBUF(1);
		USART2_RX_STA=0;
		res = sim7600_send_cmd((u8 *)ftppath,(u8 *)"BEGIN",60000);
		//超时复位，此处若FTP服务器迟迟不给回应会引起复位Add By Yao
		if(res==1)
		{
			printf("等待ftp服务器回应超时,SIM7600正在复位中！\r\n\r\n");	
			SIM7600_Reset();
		}
//		u8 errtimes = 0;
//		while(res)
//		{
//			++errtimes;
//			if (errtimes >= 3)
//			{
//				u8 rres = 1;
//				while(rres)
//				{
//					rres=GSM_Dect();
//					if (rres != SIM_OK && rres != SIM_CREG_FAIL)
//					{
//						printf("res:%d RX: %s\r\n", rres, USART2_RX_BUF);
//						continue;
//					}
//					rres=SIM7600_CONNECT_SERVER();
//					OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
//					printf("res:%d RX: %s\r\n", rres, USART2_RX_BUF);
//				}
//				errtimes = 0;
//			}
//			OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);

//			res = sim7600_send_cmd((u8 *)ftppath,(u8 *)"BEGIN",60000);
//			printf("ftpput:%d   %s\r\n", res, USART2_RX_BUF);

//		}
		sprintf((char *)dpath,"0:/%s",fn);
		fres = 1;
		while (fres)
		{
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
			fres = f_open(&sdfp,(const char *)dpath,FA_OPEN_EXISTING | FA_READ);
			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
			printf("+FOPEN:%d\r\n",fres);
			br = 0;
		}
		if (fileinfo.fsize == 30)
		{
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
			f_read(&sdfp, &sdrbuff, 30, &br);
			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
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
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
				fres = f_close(&sdfp);
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
				printf("+FCLOSE:%d\r\n",fres);
			}
			fres = 1;
			while (fres)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
				fres = f_unlink((const char *)dpath);
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
				printf("+FUNLINK:%d\r\n",fres);
			}
		}
		else if (fileinfo.fsize == 900000)
		{
			u32 finish_size=0;
			for (i = 0; i < 30000; ++i)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
				fres = f_read(&sdfp, &sdrbuff, 30, &br);
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
				for (j = 0, ctrlnum = 0; j < 30; ++j)
				{
					if (sdrbuff[j] == 0x03 || sdrbuff[j] == 0x1A)  //0x1A为结束标志,需加0x03前缀
					{
						USART2_TX_BUF[j + ctrlnum] = 0x03;
						ctrlnum++;
					}
					USART2_TX_BUF[j + ctrlnum] = sdrbuff[j];
				}
				SendUsart2TXBUF(30 + ctrlnum);
				//显示发送完成百分比，Add By Yao
				finish_size +=30;
				if(finish_size%45000==0)
				{
					printf("已完成:%d%%\r\n",finish_size/45000*5);
				}
				/*
					USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //使能串口2的DMA发送     
					DMAx_Enable(DMA1_Stream6,30 + ctrlnum);     //开始一次DMA传输！
					while(1)
					{
						if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=0)//判断DMA数据流6是否传输完成
						{
							DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);
							break;
						}
					}
				*/
			}
			//发送结束符
			USART2_TX_BUF[0] = 0x1A;
			SendUsart2TXBUF(1);
			//等待断开连接成功 Add By Yao
//			u8 t=0;
//			while(!sim7600_check_cmd((u8*)"OK"))
//			{
//				t++;
//				OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
//				if(t>20)
//				{
//					t=0;
//					printf("与服务器断开连接超时！\r\n");
//				}
//			}
			
			fres = 1;
			while (fres)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
				fres = f_close(&sdfp);
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
				printf("+FCLOSE:%d\r\n",fres);
			}
			fres = 1;
			while (fres)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
				fres = f_unlink((const char *)dpath);
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
				printf("+FUNLINK:%d\r\n",fres);
			}
		}
	}
}

//usb任务函数
void usb_task(void *p_arg)
{
	printf("USB任务开始\r\n");
	u8 i,s;
  u8 TarName[64];
	u8 PathName[64];
	u8 wbuf[64];
	u8 rbuf[64];
	u32 k,t,j;
	
	const u8 buf30[] = "012345678901234567890123456789";
	const u8 buf22[] = "0123456789012345678901";
	u8 buffer[2550];
	
  CH376S_Init();
	
	s = mInitCH376Host( );    // 初始化CH376

	//ch376 host模式初始化失败标志，led1闪烁
	if(s == ERR_USB_UNKNOWN)
	{
		printf("CH376初始化失败\r\n");
	}
	
	//初始化成功，开始连接usb
  printf( "等待U盘插入\r\n" );
	//----- usb连接 -----
  while ( CH376DiskConnect() != USB_INT_SUCCESS )/* 检查U盘是否连接,等待U盘插入,对于SD卡,可以由单片机直接查询SD卡座的插拔状态引脚 */
  {  
		delay_ms( 100 );
  }
  delay_ms( 200 );              // 对于检测到USB设备的,最多等待100*50mS,主要针对有些MP3太慢,对于检测到USB设备
                                //并且连接DISK_MOUNTED的,最多等待5*50mS,主要针对DiskReady不过的
  for ( i = 0; i < 100; i ++ )
	{	
		delay_ms( 50 );
		s = CH376DiskMount( );      //初始化磁盘并测试磁盘是否就绪.
		if ( s == USB_INT_SUCCESS ) // 准备好
			break;                                          
		else if ( s == ERR_DISK_DISCON )    // 检测到断开,重新检测并计时
			break;  
		if ( CH376GetDiskStatus( ) >= DEF_DISK_MOUNTED && i >= 5 ) // 有的U盘总是返回未准备好,不过可以忽略,只要其建立连接MOUNTED且尝试5*50mS
			break; 
	}          
	if ( s == ERR_DISK_DISCON )   // 检测到断开,重新检测并计时
	{  
		printf( "设备拔出\r\n" );
	  //continue;
	}		
	if ( CH376GetDiskStatus( ) < DEF_DISK_MOUNTED ) // 未知USB设备,例如USB键盘、打印机等
	{  
		printf( "未知设备\r\n" );
	}
	
	
	//----- 获取出厂信息 -----
	i = CH376ReadBlock( wbuf );  /* 如果需要,可以读取数据块CH376_CMD_DATA.DiskMountInq,返回长度 */
	if ( i)  /* U盘的厂商和产品信息 */
	{  
		wbuf[ i ] = 0;
		printf( "U盘信息: %s \r\n", ((P_INQUIRY_DATA)wbuf) -> VendorIdStr );
	}
	printf( "DiskQuery:  " );		/* 检查U盘或者SD卡的剩余空间 */
	s = CH376DiskQuery( (PUINT32)wbuf );	/* 查询磁盘剩余空间信息,扇区数 */
	printf("s=%02x \r\n",(unsigned short)s );
	printf( "剩余空间： %ld MB\r\n", *(PUINT32)wbuf / ( 1000000 / DEF_SECTOR_SIZE ) );
	
	
////////////////////////////////----- 30字节文件 -----////////////////////////////////////////
	

	//----- 创建文件file30 -----
	printf("创建文件  ");
	strcpy((char *)TarName, "/A181121.TXT"); // 目标文件名
	s = CH376FileCreatePath(TarName);      	 // 新建多级目录下的文件,支持多级目录路径,输入缓冲区必须在RAM中
	printf("s=%02x \r\n",(unsigned short)s);    // 14代表创建成功

	//----- 写入文件 -----
	printf("写入文件  ");
	s = CH376ByteWrite((u8*)buf30,30, NULL);             // 以字节为单位向当前位置写入数据块
	printf("s=%02x \r\n",(unsigned short)s);                                 // 14代表写入成功
	
	//----- 关闭文件 -----
	printf("关闭文件  "); 
	  s = CH376FileClose(TRUE);               // 关闭文件,对于字节读写建议自动更新文件长度
	printf("s=%02x \r\n",(unsigned short)s);
  delay_ms(500);


	//打开文件
	printf("打开文件  ");
		s = CH376FileOpenPath(TarName);      	 // 新建多级目录下的文件,支持多级目录路径,输入缓冲区必须在RAM中
	printf("s=%02x \r\n",(unsigned short)s);    // 14代表创建成功

	//读文件
	printf("读取文件  ");
	s = CH376ByteRead(rbuf,30,NULL);
	printf("s=%02x \r\n",(unsigned short)s);    // 14代表创建成功
	//for (i = 0; i < 30; ++i) printf("0x%02X ", rbuf[i]);
	printf("\r\n");
	printf("文件内容：%s\r\n",rbuf);
	delay_ms(500);
		
		//----- 关闭文件 -----
	printf("关闭文件  "); 
	s = CH376FileClose(TRUE);               // 关闭文件,对于字节读写建议自动更新文件长度
	printf("s=%02x \r\n",(unsigned short)s);
	
	
//////////////////////////////////////----- 220*1500字节文件 -----//////////////////////////////////////////
	//----- 创建文件file22 -----
	printf("创建文件: ");
	strcpy((char *)TarName, "/DAWENJ.TXT");     // 目标文件名
	s = CH376FileCreatePath(TarName);      	    // 新建多级目录下的文件,支持多级目录路径,输入缓冲区必须在RAM中
	printf("s=%02X \r\n",(unsigned short)s);    // 14代表创建成功
	for(j = 0; j < 100; j++)
	{
			//----- 拷贝数据 -----
		for(t = 0; t < 22; t++)
			buffer[t + j * 22] = buf22[t];
	}	
	//----- 写入文件 -----
	printf("写入文件:  ");
	for(k = 0; k < 3000; k++)
	{
		s = CH376ByteWrite(buffer, 2100, NULL);             // 以字节为单位向当前位置写入数据块
		//每2000次冲刷一次
	}	
	printf("s=%02X \r\n",(unsigned short)s);                                 // 14代表写入成功
	printf("文件大小: %ld\r\n",CH376GetFileSize());
	
	//----- 关闭文件 -----
	printf("关闭文件:  "); 
	s = CH376FileClose(TRUE);   // 关闭文件,对于字节读写建议自动更新文件长度
	printf("s=%02X \r\n",(unsigned short)s);

	printf("----- ok -----\r\n");
	
	while (1);
}


//monitor任务函数
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

