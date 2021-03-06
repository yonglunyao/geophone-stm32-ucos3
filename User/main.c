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
#include "socket.h"
#include "http.h"

//定义站点号
#define NUMBER 4

#define ENABLE_HTTP 1
//任务优先级
#define START_TASK_PRIO		3
//任务堆栈大小	
#define START_STK_SIZE 		128 //原值128
//任务控制块
OS_TCB StartTaskTCB;
//任务堆栈	
CPU_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *p_arg);

u8 SwitchFtptoFlash;//标记工作模式，0：FTP，1：U盘
void Set_Mode(void);

//任务优先级
#define READ_TASK_PRIO		4
//任务堆栈大小	
#define READ_STK_SIZE 		150 //原值96
//任务控制块
OS_TCB ReadTaskTCB;
//任务堆栈	
CPU_STK READ_TASK_STK[READ_STK_SIZE];
void read_task(void *p_arg);


//任务优先级
#define GPS_TASK_PRIO		4
//任务堆栈大小	
#define GPS_STK_SIZE 		128 //原值64
//任务控制块
OS_TCB GpsTaskTCB;
//任务堆栈	
CPU_STK GPS_TASK_STK[GPS_STK_SIZE];
void gps_task(void *p_arg);


//任务优先级
#define FILE_TASK_PRIO		5
//任务堆栈大小	
#define FILE_STK_SIZE 		500 //原值330
//任务控制块
OS_TCB FileTaskTCB;
//任务堆栈	
CPU_STK FILE_TASK_STK[FILE_STK_SIZE];
void file_task(void *p_arg);

//任务优先级
#define FTP_TASK_PRIO		6
//任务堆栈大小	
#define FTP_STK_SIZE 		500 //原值330
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
#define REPORT_TASK_PRIO		7
//任务堆栈大小	
#define REPORT_STK_SIZE 		330
//任务控制块
OS_TCB ReportTaskTCB;
//任务堆栈	
CPU_STK REPORT_TASK_STK[REPORT_STK_SIZE];
void report_task(void *p_arg);

//任务优先级
#define MONITOR_TASK_PRIO		7
//任务堆栈大小	
#define MONITOR_STK_SIZE 		150 //原值96
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
OS_MUTEX SIM_OK_mutex; //互斥信号量
OS_MUTEX SIM_Busy_mutex; //互斥信号量

u8 server_ip[]="207.246.83.11";	//数据上报服务器IP
u8 port[]="8888";	//数据上报服务器端口

Packet_connectData data={"0004","DLUTDLUT",RESET_NO,STATUS_OK,0,0,100};//数据上报结构体

u32 local_files_num=0;//记录SD卡中待发送的文件个数
u32 sent_files_num=0;//记录ftp发送完成的文件个数



int main()
{  	
	
	OS_ERR err;
	
	delay_init();
	USART1_Init(921600);
	USART2_Init(921600);
	USART3_Init(38400);		
	printf("当前站点：%d\r\n\r\n",NUMBER);
#if OLED_ENABLE
	extern u8 oled_buf[17];
	OLED_Init();
	sprintf((char*)oled_buf,"   Station %d    ",NUMBER); //未查明原因：开启三行显示卡B OSStartHang
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
//		printf("读取芯片ID\r\n");
		if (id0 != 3 || id1 != 3 || id2 != 3)
		{
			printf("读ADS1256错误\r\nASD1256 Chip ID = 0x%02X & 0x%02X & 0x%02X\r\n", id0, id1, id2);
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
 IWDG_Init(5,625); //看门狗初始化
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

/*
设置模式
本函数用于启动后在FTP和U盘模式之间切换，
同时，每次启动后默认采用上一次运行模式，模式信息存储在STM32内部FLASH中

*/
//要写入到STM32 FLASH的字符串数组
const u8 FTP_Buffer[]={"FTP"};
const u8 FLASH_Buffer[]={"FLASH"};

#define FTP_LENTH sizeof(FTP_Buffer)	 		  	//数组长度	
#define FLASH_LENTH sizeof(FLASH_Buffer)	 		  	//数组长度
	
#define FTP_SIZE FTP_LENTH/4+((FTP_LENTH%4)?1:0)	//四字节读取
#define FLASH_SIZE FLASH_LENTH/4+((FLASH_LENTH%4)?1:0)

#define FLASH_SAVE_ADDR  0X08060004 	//设置FLASH 保存地址(必须为偶数，且所在扇区,要大于本代码所占用到的扇区.

void Set_Mode(void)
{
	extern u8 oled_buf[17];
	u8 buff[8];	//字节读取缓冲区
	char* strx;	//查找用
	STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)buff,FLASH_SIZE);//先读取一次
	strx=strstr((const char*)buff,"FTP");//检查读取内容是否有FTP
	//检查读取内容
	if(strx!=NULL)
		SwitchFtptoFlash=0;//0代表FTP模式
	else
	{
		strx=strstr((const char*)buff,"FLASH");//检查读取内容是否有FLASH
		if(strx!=NULL)
			SwitchFtptoFlash=1;//1代表U盘模式
		else
		{
			SwitchFtptoFlash=0;
			printf("未检测到上一次运行模式，默认以FTP模式运行\r\n\r\n");
		}
	}
	
	//开始准备设置模式
	switch (SwitchFtptoFlash)
	{
		case 0:
			printf("选择工作模式(当前工作模式：FTP模式，切换请按键)\r\n\r\n");
#if OLED_ENABLE
		OLED_ShowString(0,3,(u8*)"    Mode:FTP    ");
#endif
			break;
		case 1:
			printf("选择工作模式(当前工作模式：U盘模式，切换请按键)\r\n\r\n");
#if OLED_ENABLE
		OLED_ShowString(0,3,(u8*)"    Mode:USB    ");
#endif
			break;
	}
	delay_ms(1000);
	
	
	//准备选择模式
	Mode_EXTI_Init();//模式切换按键中断初始化
	u8 tmp=SwitchFtptoFlash;
#if OLED_ENABLE
		OLED_ShowString(0,6,(u8*)" Press left key ");
#endif
	for(int i=0;i<5;i++)
	{
		printf("请在%d秒内选择\r\n\r\n",5-i);
#if OLED_ENABLE
		sprintf((char*)oled_buf," Change Mode:%d? ",5-i);
		OLED_ShowString(0,3,(u8*)oled_buf);
#endif
		if(tmp!=SwitchFtptoFlash)break;//检测到按键操作退出等待
		delay_ms(1000);
	}
	
#if OLED_ENABLE
		OLED_ShowString(0,6,(u8*)"                ");
#endif	

	//将模式写入内部FLASH，供下一次查询使用
	if(SwitchFtptoFlash)//如果是1：U盘模式
	{
		printf("当前模式:U盘模式\r\n\r\n");	//1:U盘
#if OLED_ENABLE
		OLED_ShowString(0,3,(u8*)"    Mode:USB    ");
#endif
		STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)FLASH_Buffer,FLASH_SIZE);
	}
	else
	{
		printf("当前模式:FTP模式\r\n\r\n");	//0:FLASH
#if OLED_ENABLE
		OLED_ShowString(0,3,(u8*)"    Mode:FTP    ");
#endif
		STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)FTP_Buffer,FTP_SIZE);
	}
}


//开始任务函数
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
               (OS_ERR      *)&err);         //错误类型					SIM_OK_mutex
	//创建互斥信号量 SIM_OK_mutex SIM故障检查
	OSMutexCreate((OS_MUTEX *)&SIM_OK_mutex,    //指向信号量变量的指针
               (CPU_CHAR    *)"SIM_OK_mutex",    //信号量的名字
               (OS_ERR      *)&err);         //错误类型		
							 
	//创建互斥信号量 SIM_OK_mutex SIM故障检查
	OSMutexCreate((OS_MUTEX *)&SIM_Busy_mutex,    //指向信号量变量的指针
               (CPU_CHAR    *)"SIM_Busy_mutex",    //信号量的名字
               (OS_ERR      *)&err);  
			
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

//创建REPORT任务							 
	OSTaskCreate((OS_TCB 	* )&ReportTaskTCB,		
				 (CPU_CHAR	* )"report task", 		
                 (OS_TASK_PTR )report_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )REPORT_TASK_PRIO,     
                 (CPU_STK   * )&REPORT_TASK_STK[0],	
                 (CPU_STK_SIZE)REPORT_STK_SIZE/10,	
                 (CPU_STK_SIZE)REPORT_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);	
/*
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
	OS_TaskSuspend((OS_TCB*)&ReadTaskTCB,&err);		//挂起采集任务，GPS定位成功后解挂
	OS_TaskSuspend((OS_TCB*)&FileTaskTCB,&err);		//挂起文件任务，GPS定位成功后解挂
	OS_TaskSuspend((OS_TCB*)&FtpTaskTCB,&err);		//挂起FTP任务，GPS定位成功后解挂
	OS_TaskSuspend((OS_TCB*)&ReportTaskTCB,&err);		//挂起USB任务，等待按键后任务切换
	OS_TaskSuspend((OS_TCB*)&UsbTaskTCB,&err);		//挂起USB任务，等待按键后任务切换
	
	OS_CRITICAL_EXIT();	//进入临界区
}

//cs5376a任务函数
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
	u32 error_times0=0;
	u32 error_times1=0;
	u32 error_times2=0;
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
//		printf("%f\r\n",qdata[qrear].ch[1]);//测试用
		qdata[qrear].day = day;
		qdata[qrear].sec = tmpsec;
		qdata[qrear].min = tmpmin;
		qdata[qrear].hour = tmphour;
		qdata[qrear].ms = tmpms / 10;
		qrear = (qrear + 1) % QSIZE;
		
		//判断读数是否正确，判断某一通道连续为0是否超过1分钟，主要用于解决的读取数据某一通道全零，//Add By Yao
		if(qdata[qrear].ch[0]==0)
			error_times0++;
		else
			error_times0=0;
		
		if(qdata[qrear].ch[1]==0)
			error_times1++;
		else
			error_times1=0;
		
		if(qdata[qrear].ch[2]==0)
			error_times2++;
		else
			error_times2=0;
		//判断次数
		if(error_times0>30000||error_times1>30000||error_times2>30000)
		{
			printf("数据采集出错，准备复位，请等待...\r\n\r\n");
#if OLED_ENABLE
			OLED_ShowString(0,3,(u8*)" Sampling error ");
#endif
			delay_ms(1000);
			soft_reset();//软件复位
		}
		
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
	u32 t=0;
	TIM5_Init(10000-1, 8394-1); //8393-1 10-1
//TIM2_Init(10000-1, 8500-1); //用于检测GPS状态
	while(!(USART3_RX_STA&0X8000)) //Edit By Yao 检测GPS通讯情况
	{
		t++;
		if(t==10000000)
		{
			printf("GPS通讯异常！\r\n");
#if OLED_ENABLE
			OLED_ShowString(0,3,(u8*)"   GPS error    ");
#endif
			t=0;
		}
	}
	printf("等待GPS定位完成...\r\n\r\n");	//Add By Yao
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
			//每隔10分钟屏幕复位，解决屏幕显示混乱问题
			
			if(min%10==0&&sec==0)
			{
				OLED_Init();			
				sprintf((char*)oled_buf,"Wait:%02d Sent:%03d",local_files_num,sent_files_num);
				OLED_ShowString(0,6,(u8*)oled_buf);				
			}
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
	my_mem_init(SRAMIN);		//初始化内部内存池
	//SD卡初始化
	while(SD_Init()!=0)
	{	
		OSTimeDlyHMSM(0,0,0,4,OS_OPT_TIME_HMSM_STRICT,&err);
	}
	FATFS_Init();							//为sd卡数据缓冲区fatfs、文件1、文件2、各磁盘工作区相关变量申请内存		
  f_mount(fs[0],"0:",1); 					//挂载SD卡，使用fs[0]磁盘工作区0	
	OSSemPost(&sdinit,OS_OPT_POST_FIFO,&err);
	FIL sdfp; //文件指针
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
	
	/*以下用于删除SD卡中的垃圾文件，主要是删除因意外未存储完成的文件垃圾 Add By Yao*/
	FRESULT fres = 1;
	DIR dir;
	while(fres)//刷新目录
	{
		OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
		fres = f_opendir(&dir, "0:/");
		OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
		if(fres) printf("opendir:%d\r\n", fres);
		OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err);
	}
	
	OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
	fres = f_findfirst(&dir, &fileinfo,"","*.*");	//寻找第一个匹配项
	OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
	while (fres == FR_OK && fileinfo.fname[0])  //文件读取失败或非合法文件
	{
		if (fileinfo.fsize == 30 || fileinfo.fsize == 900000) //已完成文件
		{
			local_files_num++;
			fres=f_findnext(&dir, &fileinfo);
			continue;
		}
		else	//未完成文件
		{
			sprintf((char *)dpath,"0:/%s",fileinfo.fname);
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
			fres = f_unlink((const char *)dpath);
			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
			printf("清除垃圾文件:%s\r\n\r\n",fileinfo.fname);
			fres=f_findnext(&dir, &fileinfo);
			continue;
		}
	}
	
#if OLED_ENABLE
			sprintf((char*)oled_buf,"Wait:%02d Sent:%03d",local_files_num,sent_files_num);
			OLED_ShowString(0,6,(u8*)oled_buf);
#endif
//	OS_TaskSuspend((OS_TCB*)&FileTaskTCB,&err);		//在此挂起文件任务，等待GPS定位成功
	
	
//	IWDG_Init(5,625); //看门狗初始化
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
			sprintf((char *)hpath,"%s.HEA",fname);
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
		res = f_write(&sdfp,&sdwbuff,30,&dbw); //sizeof(sdwbuff),此处为何sdwbuff取地址
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
			local_files_num++;
			data.Wait_Count++;
			res = 1;
			while (res)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
				res = f_close(&sdfp);
				OSMutexPost(&sdmutex,OS_OPT_POST_NONE,&err);  //释放sdmutux信号量,且不引起调度
//				printf("FCLOSE:%d\r\n",res);
#if OLED_ENABLE
			sprintf((char*)oled_buf,"Wait:%02d Sent:%03d",local_files_num,sent_files_num);
			OLED_ShowString(0,6,(u8*)oled_buf);
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


//ftp任务函数
void ftp_task(void *p_arg)
{
	
	OS_ERR err; 
	OSMutexPend(&SIM_OK_mutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待SIM_OK_mutex信号量
	OS_TaskSuspend((OS_TCB*)&UsbTaskTCB,&err);		//挂起FTP任务
	OSSemPend(&sdinit,0,OS_OPT_PEND_BLOCKING,0,&err);  //等待full信号量
//	static char lfn[_MAX_LFN * 2 + 1];
	u16 i;
	u8 j;
#if OLED_ENABLE
	extern u8 oled_buf[17];;
#endif
//	USART2_Init(921600);
	//USART2_Init(115200);sim7600_send_cmd((u8 *)"AT+IPREX=921600",(u8 *)"OK",200);
	u8 res=1;
	USART2_TX_BUF[0] = 0x1A;
	u8 buf[38];
	u8 buflen;
	
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
	OSMutexPost(&SIM_OK_mutex,OS_OPT_POST_FIFO,&err);  //释放SIM_OK_mutex信号量，且就绪阻塞态进程
#if OLED_ENABLE
	OLED_ShowString(0,3,(u8*)"Connected to FTP");
#endif
	printf("设置FTP配置完成，准备数据发送！\r\n\r\n");
	/************连接FTP完成**********/
	FIL sdfp; //文件指针
	//FILINFO fileinfo = {0};
	UINT br = 0;
	DIR dir;
	FRESULT fres = 1;
	u8 sdrbuff[30];
	u8 ctrlnum = 0;
	char *fn;
	u8 dpath[20];
	u8 ftppath[40];
	u16 t=0;//用于记录重新读取文件次数
	while(1)
	{
start:
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
					printf("FTP发送任务等待采集完成%d...\r\n\r\n",t/10);
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
					printf("等待SD卡文件就绪超时，请检查ADS1256和GPS！\r\n\r\n");
#if OLED_ENABLE
					OLED_ShowString(0,3,(u8*)" Sampling error ");
#endif
					if(t>100)
					{
						printf("等待超时，准备软件复位！\r\n\r\n");
#if OLED_ENABLE
					OLED_ShowString(0,3,(u8*)" Ready to reset ");
#endif
						delay_ms(1000);
						soft_reset();//软件复位
					}
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
#if OLED_ENABLE
				OLED_ShowString(0,3,(u8*)" 4G Timeout,RST ");
#endif
				SIM7600_Reset();
				goto start;//重新开始
			}
		}
		
		//发送前预处理 删除可能因前一次复位服务器上产生的垃圾残余文件。两种情况:1.文件存在，删除成功返回OK;2.文件不存在返回211 ADD By Yao
		sprintf((char *)ftppath,"AT+CFTPDELE=\"/%d/%s\"",NUMBER, fn);	//Edit By Yao
		res=sim7600_send_cmd(ftppath,(u8*)"211",500);//发送前先删除，用于删除上一次复位后的残余文件,此处仅检查返回是否为211，
																										//若文件存在会返回OK,在下面进行判断是哪一种情况
																										//如果此时返回OK，需要等待500倒计时为0才会继续执行
		while(res)//进行判断是哪一种情况:1.指令超时;2.返回为OK
		{

			//此时USART2_RX_STA已经被标记为0，但是USART2_RX_BUF中的数据还在，但是有时候会接收到PB DONE 将OK冲刷掉，引起误4G复位，但影响不大
			if(strstr((const char*)USART2_RX_BUF,"OK")) break;
			//超时4G复位
			else			
			{
				printf("等待ftp服务器回应超时,SIM7600正在复位中！\r\n\r\n");	
#if OLED_ENABLE
				OLED_ShowString(0,3,(u8*)"FTP Timeout,RST ");
#endif
				SIM7600_Reset();
				goto start;//重新开始
			}
		}
		USART2_RX_STA=0;
		
		//开启FTP传输，首先进行检测SIM7600的回应，较好地解决了通讯失败问题	Add By Yao 
		t=0;
		while(sim7600_send_cmd((u8 *)"AT",(u8 *)"OK",100))
		{
			if(t>200)
			{
				printf("等待SIM7600回应超时,SIM7600正在复位中！\r\n\r\n");	
#if OLED_ENABLE
				OLED_ShowString(0,3,(u8*)" 4G Timeout,RST ");
#endif
				SIM7600_Reset();
				goto start;//重新开始
			}
		}
		//准备发送
		printf("Uploading via FTP...%s \r\n", fn);
#if OLED_ENABLE
		sprintf((char*)oled_buf,"Send%s",fn);
		OLED_ShowString(0,3,(u8*)oled_buf);
#endif
		sprintf((char *)ftppath,"AT+CFTPPUT=\"/%d/%s\"",NUMBER, fn);	//Edit By Yao
		res = sim7600_send_cmd((u8 *)ftppath,(u8 *)"BEGIN",10000);
		//超时复位，此处若FTP服务器迟迟不给回应会引起复位Add By Yao
		if(res==1)
		{
			printf("等待ftp服务器回应超时,SIM7600正在复位中！\r\n\r\n");	
#if OLED_ENABLE
			OLED_ShowString(0,3,(u8*)"FTP Timeout,RST ");
#endif
			SIM7600_Reset();
			goto start;//重新开始
		}

		sprintf((char *)dpath,"0:/%s",fn);
		fres = 1;
		while (fres)
		{
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
			fres = f_open(&sdfp,(const char *)dpath,FA_OPEN_EXISTING | FA_READ);
			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
			printf("+FOPEN:%d\r\n",fres);
#if OLED_ENABLE
			OLED_ShowString(0,3,(u8*)"     Start      ");
#endif
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
			//发送结束符
			USART2_TX_BUF[i + ctrlnum] = 0x1A;
			SendUsart2TXBUF(30 + ctrlnum+1);
			//判断发送结束符后服务器响应
			t=0;
			while(sim7600_send_cmd((u8*)"AT",(u8*)"OK",200))//判断FTP服务器响应，超时复位
			{
				t++;
				if(t>50)
				{
					printf("等待FTP服务器回应超时,SIM7600正在复位中！\r\n\r\n");	
#if OLED_ENABLE
					OLED_ShowString(0,3,(u8*)" 4G Timeout,RST ");
#endif
					SIM7600_Reset();
					goto start;//重新开始
				}
			}
			t=0;
			//准备关闭SD卡文件
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
				local_files_num--;
				sent_files_num++;
				data.Wait_Count=local_files_num;
				data.Sent_Count=sent_files_num;
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
#if OLED_ENABLE
					sprintf((char*)oled_buf,"  Finished:%02d%% ",finish_size/45000*5);
					OLED_ShowString(0,3,(u8*)oled_buf);
#endif
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
			
			t=0;
			while(sim7600_send_cmd((u8*)"AT",(u8*)"OK",200))//判断FTP服务器响应，超时复位
			{
				t++;
				if(t>50)
				{
					printf("等待FTP服务器回应超时,SIM7600正在复位中！\r\n\r\n");	
#if OLED_ENABLE
					OLED_ShowString(0,3,(u8*)" FTP Timeout,RST");
#endif
					SIM7600_Reset();
					goto start;//重新开始
				}
			}
			t=0;
			
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
				local_files_num--;
				sent_files_num++;
				data.Wait_Count=local_files_num;
				data.Sent_Count=sent_files_num;
#if OLED_ENABLE
				sprintf((char*)oled_buf,"Wait:%02d Sent:%03d",local_files_num,sent_files_num);
				OLED_ShowString(0,6,(u8*)oled_buf);
				OLED_ShowString(0,3,(u8*)" Send completed ");
#endif
			}
		}
#if ENABLE_HTTP
		//数据上报HTTP
		data.time.year=(u8)(year-2000);
		data.time.month=month;
		data.time.day =day;
		data.time.hour=hour;
		data.time.minute=min;
		data.time.second=sec;
		Serialize_HttpReport(buf,data,&buflen);//数据串行化
		
		if(transport_HttpSendPacketBuffer(server_ip,port,buf,buflen))
		{
			printf("Report error\r\n\r\n");
#if OLED_ENABLE
				OLED_ShowString(0,3,(u8*)"  Report Error  ");
#endif
		}
		else
		{
			printf("Report Successfully\r\n\r\n");
#if OLED_ENABLE
				OLED_ShowString(0,3,(u8*)"   Report OK    ");
#endif
		
		}

#endif
	}
}

//usb任务函数
//写入到U盘速度比较慢，达不到波特率速度，测试不同U盘写入速度不同，目前最快测速70KB/s，可尝试按扇区写入方式
void usb_task(void *p_arg)
{
	OS_ERR err; 
//	OS_TaskSuspend((OS_TCB*)&FtpTaskTCB,&err);		//挂起FTP任务
	OSSemPend(&sdinit,0,OS_OPT_PEND_BLOCKING,0,&err);  //等待full信号量
	printf("USB任务开始\r\n");

	u16 i;
	u8 s,t;
	FIL sdfp; //文件指针
	//FILINFO fileinfo = {0};
	UINT br = 0;
	DIR dir;
	FRESULT fres = 1;
	u8* sdrbuff=mymalloc(SRAMIN,9030);

	char *fn;
	u8 sdpath[20];
	u8 upath[20];
	u8 oled_buf[17];
	
	UINT16 realCount=0;
	UINT16 totalCount=0;
	
	s = mInitCH376Host( );    // 初始化CH376

	//ch376 host模式初始化失败标志，led1闪烁
	if(s == ERR_USB_UNKNOWN)
	{
		printf("U盘初始化失败\r\n");
		//自动切换到FTP任务,无法正确执行
#if OLED_ENABLE
					OLED_ShowString(0,3,(u8*)"   USB Error   ");
#endif
		OS_TaskSuspend((OS_TCB*)&UsbTaskTCB,&err);		//挂起USB任务
		OS_TaskResume((OS_TCB*)&FtpTaskTCB,&err);			//解挂FTP任务
		printf("存储方式由U盘切换为FTP\r\n\r\n");
	}
	
	//初始化成功，开始连接usb
//  printf( "等待U盘插入\r\n" );
	//----- usb连接 -----
	//先检测一次,屏蔽每次先显示未插入U盘
	CH376DiskConnect();
	OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err);
  while ( CH376DiskConnect() != USB_INT_SUCCESS )/* 检查U盘是否连接,等待U盘插入,对于SD卡,可以由单片机直接查询SD卡座的插拔状态引脚 */
  {  
		printf("未插入U盘\r\n\r\n");
#if OLED_ENABLE
					OLED_ShowString(0,3,(u8*)"  No USB Disk   ");
#endif
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err);
  }
	printf("检测到U盘\r\n\r\n");
	
//	sprintf((char*)upath,"/%d",NUMBER);
//	CH376DirCreate(upath); //在没有该目录的情况下创建该目录
//	CH376FileClose(TRUE);
	while(1)
	{

		//检测U盘
		u8 times;
start:
		times=0;
		while ( CH376DiskConnect() != USB_INT_SUCCESS )/* 检查U盘是否连接,等待U盘插入,对于SD卡,可以由单片机直接查询SD卡座的插拔状态引脚 */
		{  
			if(times%50==0)printf("未插入U盘\r\n\r\n");
			times++;
#if OLED_ENABLE
					OLED_ShowString(0,3,(u8*)"  No USB Disk   ");
#endif
			OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err);
		}
		if(times)printf("检测到U盘\r\n\r\n");//仅当之前未检测到U盘然后检测到U盘后才显示"检测到U盘"		
		//刷新SD卡目录
		fres=1;
		while(fres)
		{
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
			fres = f_opendir(&dir, "0:/");
			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
			if(fres) printf("opendir:%d\r\n", fres);
			OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err);
		}
		//读取目录
		while(1)
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
		//判断当前是否有待复制的文件
		if (fileinfo.fname[0] == 0)
		{
			t++;
			//Add By Yao
			if(t%10==0)
			{
				if(t<=70)
				{
					printf("采集中...\r\n");
					printf("Copy任务等待采集完成%d...\r\n\r\n",t/10);
#if OLED_ENABLE
					switch(t/10)
					{
						case 1: case 4:case 7://出现7说明已经出现问题·
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
				//由于SD卡每1分钟产生一个文件，如果超过70S没有等到文件，直接进行软复位
				else
				{
					printf("等待SD卡文件就绪超时，请检查ADS1256和GPS！\r\n\r\n");
#if OLED_ENABLE
					OLED_ShowString(0,3,(u8*)" Sampling error ");
#endif
					if(t>100)
					{
						printf("等待超时，准备软件复位！\r\n\r\n");
#if OLED_ENABLE
						OLED_ShowString(0,3,(u8*)" Ready to reset ");
#endif
						delay_ms(1000);
						soft_reset();//软件复位
					}
				}
			}
			OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err);
			continue;
		}
		//找到待发送的文件。准备发送
		printf("Copy To Flash Disk...%s \r\n", fn);
#if OLED_ENABLE
		sprintf((char*)oled_buf,"Copy%s",fn);
		OLED_ShowString(0,3,(u8*)oled_buf);
#endif
		sprintf((char *)upath,"/%s",fn);//U盘目录
		sprintf((char *)sdpath,"0:/%s",fn);//SD卡目录
		//在U盘上创建文件
		delay_us(10);
		s=1;
		while(s!=0x14)
		{
			s = CH376FileCreatePath((PUINT8)upath);
			if(s!=0x14)printf("+UOPEN:%02x\r\n",s);
		}
		//打开SD卡文件
		fres = 1;
		while (fres)
		{
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
			fres = f_open(&sdfp,(const char *)sdpath,FA_OPEN_EXISTING | FA_READ);
			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程			
			printf("+FOPEN:%d\r\n",fres);
		}
#if OLED_ENABLE
			OLED_ShowString(0,3,(u8*)"     Start      ");
#endif
		br = 0;
		if (fileinfo.fsize == 30)
		{
			OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
			f_read(&sdfp, sdrbuff, 30, &br);
			OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
			s=1;
			while(s!=0x14)
			{
				s=CH376ByteWrite((u8*)sdrbuff,30, NULL); //向U盘写入文件，考虑是否加入判断返回值过程
				if(s!=0x14)printf("+UWR:%02x\r\n",s);
			}
			
			s = 1;
			while(s!=0x14)
			{
				s=CH376FileClose(TRUE);//关闭U盘文件
				if(s!=0x14)printf("+UCLOSE:%02x\r\n",s);
			}
			
			fres = 1;
			while (fres)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
				fres = f_close(&sdfp);//关闭SD卡文件
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
				printf("+FCLOSE:%d\r\n",fres);
			}
			if(CH376DiskConnect() != USB_INT_SUCCESS) continue;//如果传输中丢失U盘不可继续执行删除当前传输文件操作，避免数据丢失
			fres = 1;
			while (fres)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
				fres = f_unlink((const char *)sdpath);//删除已SD卡中拷贝完成的文件
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
				local_files_num--;
				sent_files_num++;//考虑更改为统计U盘文件数量的全局变量

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
			for (i = 0; i < 100; i++)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
				//从SD卡文件中读取数据放到缓冲区，一次读30，分300次读完，然后一次性写入U盘
				for(u16 j=0;j<300;j++)
				{
					fres = f_read(&sdfp, &sdrbuff[j*30], 30, &br);
					if(br!=30) printf("Copy error\r\n\r\n");
//					printf("%d\r\n",sdrbuff[j*30+18]);//测试用
				}
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
				
				s=1;
				realCount=0;
				totalCount=0;
				delay_us(10);
				//向U盘写入文件，计划写入9000字节，实测经常出现写不到9000字节而出错，以下程序增加了校验功能，
				//当写入不足9000时，重新请求从上次断点继续写入，解决了数据写入错误的问题//2020-07-29
				while(s!=0x14)
				{
					s=CH376ByteWrite((u8*)&sdrbuff[totalCount],9000-totalCount, &realCount); 
					if(s==0xb4) goto start;
					if(s!=0x14)
					{
						printf("+UWR:%02x\r\n",s);
					}
					if(realCount!=9000)
						printf("%d\r\n",realCount);
					
					totalCount+=realCount;
					if(totalCount==9000) break;
					
				}
				
				//显示发送完成百分比，Add By Yao
				finish_size +=9000;
//				printf("%d\r\n",finish_size);//测试用
				if(finish_size%45000==0)
				{
					printf("已完成:%d%%\r\n",finish_size/45000*5);
#if OLED_ENABLE
					sprintf((char*)oled_buf,"  Finished:%02d%% ",finish_size/45000*5);
					OLED_ShowString(0,3,(u8*)oled_buf);
#endif
				}
			}
			s = 1;
			while(s!=0x14)
			{
				s=CH376FileClose(TRUE);//关闭U盘文件
				if(s!=0x14)printf("+UCLOSE:%02x\r\n",s);
			}
			fres = 1;
			while (fres)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
				fres = f_close(&sdfp);
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
				printf("+FCLOSE:%d\r\n",fres);
			}
			if(CH376DiskConnect() != USB_INT_SUCCESS) continue;//如果传输中丢失U盘不可继续执行删除当前传输文件操作，避免数据丢失
			fres = 1;
			while (fres)
			{
				OSMutexPend(&sdmutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待sdmutex信号量
				fres = f_unlink((const char *)sdpath);
				OSMutexPost(&sdmutex,OS_OPT_POST_FIFO,&err);  //释放sdmutux信号量，且就绪阻塞态进程
				printf("+FUNLINK:%d\r\n",fres);
				local_files_num--;
				sent_files_num++;

#if OLED_ENABLE
				sprintf((char*)oled_buf,"Wait:%02d Copy:%03d",local_files_num,sent_files_num);
				OLED_ShowString(0,6,(u8*)oled_buf);
				OLED_ShowString(0,3,(u8*)" Copy completed ");
#endif
			}
		}
	}
}

void report_task(void *p_arg)
{
//	OS_ERR err; 
//	extern OS_MUTEX SIM_Busy_mutex;
//	printf("Report任务开始\r\n");
//	u8 buf[38];
//	u8 buflen;

//	//USART2_Init(115200);sim7600_send_cmd((u8 *)"AT+IPREX=921600",(u8 *)"OK",200);
//	u8 res=1;
//	USART2_TX_BUF[0] = 0x1A;

//	
//	USART2_TX_BUF[1] = '\r';
//	USART2_TX_BUF[2] = '\n';
//	SendUsart2TXBUF(3);
//	while(res)
//	{
//		res=GSM_Dect();
//		if (res != SIM_OK && res != SIM_CREG_FAIL)
//		{
//			printf("res:%d RX: %s\r\n", res, USART2_RX_BUF);
//			OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err);	//Add By Yao
//			continue;
//		}
//	}
//	
//	while(1)
//	{
////		OSMutexPend(&SIM_OK_mutex,0,OS_OPT_PEND_BLOCKING,0,&err); //等待SIM_OK_mutex信号量
////		sim7600_LinkToSever(server_ip,port);//连接服务器
//		data.time.year=(u8)(year-2000);
//		data.time.month=month;
//		data.time.day =day;
//		data.time.hour=hour;
//		data.time.minute=min;
//		data.time.second=sec;
//		res=1;
//		while(res)
//		{
//			res=GSM_Dect();
//			if (res != SIM_OK && res != SIM_CREG_FAIL)
//			{
//				printf("res:%d RX: %s\r\n", res, USART2_RX_BUF);
//				OSTimeDlyHMSM(0,0,3,0,OS_OPT_TIME_HMSM_STRICT,&err);	//Add By Yao
//				continue;
//			}
//		}
//		
//		Serialize_HttpReport(buf,data,&buflen);
//		transport_HttpSendPacketBuffer(server_ip,port,buf,buflen);
//		printf("Report 1 package\r\n\r\n");
//		OSTimeDlyHMSM(0,0,10,0,OS_OPT_TIME_HMSM_STRICT,&err);//延时
////		OSMutexPost(&SIM_OK_mutex,OS_OPT_POST_FIFO,&err);  //释放SIM_OK_mutex信号量，且就绪阻塞态进程
//	}
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

