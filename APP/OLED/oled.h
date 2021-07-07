//===================================================================================================
/* 函数: 
   描述: 
   注意事项：
   申明：深圳尚视界科技有限公司   （2008-2020 版权所有，盗版必究）
	 公司网站： www.sun-lcm.com
	 淘宝网站： https://shop151604432.taobao.com/index.htm?spm=a1z10.5-c.w5002-14603162597.2.4c2619d6w9oYgh
	 技术支持:QQ:3085638545
	 业务咨询电话：13509671256
	 //  功能描述   : OLED 4接口演示例程
//              说明: 
//              ----------------------------------------------------------------
//              GND    电源地
//              VCC  接5V或3.3v电源
//              D0   接PB13（SCL）
//              D1   接PB15（SDA）
//              RES  接PC11
//              DC   接PC10
//              CS   接P12               
//              ----------------------------------------------------------------
 */ 
//====================================================================================================

//******************************************************************************/
#ifndef __OLED_H
#define __OLED_H			  	 
#include "system.h"
#include "stdlib.h"	 
//使用OLED
#define OLED_ENABLE 1
//OLED模式设置
//0:4线串行模式
//1:并行8080模式
#define OLED_MODE 0
#define SIZE 16
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	    

//-----------------测试LED端口定义---------------- 
#define LED_ON GPIO_ResetBits(GPIOD,GPIO_Pin_2)
#define LED_OFF GPIO_SetBits(GPIOD,GPIO_Pin_2)

//-----------------OLED端口定义----------------  					   


#define OLED_SCLK_Clr() GPIO_ResetBits(GPIOF,GPIO_Pin_10)//D0
#define OLED_SCLK_Set() GPIO_SetBits(GPIOF,GPIO_Pin_10)

#define OLED_SDIN_Clr() GPIO_ResetBits(GPIOF,GPIO_Pin_8)//D1
#define OLED_SDIN_Set() GPIO_SetBits(GPIOF,GPIO_Pin_8)

#define OLED_RST_Clr() GPIO_ResetBits(GPIOF,GPIO_Pin_6)//RES
#define OLED_RST_Set() GPIO_SetBits(GPIOF,GPIO_Pin_6)

#define OLED_DC_Clr() GPIO_ResetBits(GPIOF,GPIO_Pin_2)//DC
#define OLED_DC_Set() GPIO_SetBits(GPIOF,GPIO_Pin_2)
 		     
#define OLED_CS_Clr()  GPIO_ResetBits(GPIOF,GPIO_Pin_0)//CS
#define OLED_CS_Set()  GPIO_SetBits(GPIOF,GPIO_Pin_0)

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据


//OLED控制用函数
void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);	   							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y, u8 *p);	 
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(u8 x,u8 y,u8 no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
#endif  
	 



