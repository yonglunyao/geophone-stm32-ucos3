stm32f407做的
app/ads1256/	ads1256的驱动。
app/ch376/	usb模块的驱动。
app/dma/	DMA实现。
app/exti/		外部中断的实现。
app/gps/		GPS模块驱动。
app/iic/		I2C驱动，用于控制ads1282演示板的，ads1256中没有用到。
app/iwdg/	看门狗的代码，用于程序跑飞后的重启。
app/sim7600	4g模块驱动。
app/spi		三路dma spi的驱动。
app/time		定时器。

app/sd/		这四个文件夹实现了SD卡的FAT32文件系统的支持，很庞大和复杂，只是为了用的话没必要去改和看它。占用rom空间和ram空间都很大，
app/flash/	rom空间主要在fatfs/ffconf.h文件中的设置，目前是英文8.3格式文件名，可以实现中文文件名或长文件名的支持，但rom空间需求会大大增加。
fatfs/		ram空间主要在malloc/malloc.h中的设置，单片机不支持malloc的，这里通过事先定义几十kb的数组当作内存池，在这个超大数组中进行动态
malloc/		内存分配，如果ram不够，可以考虑减小那几十kb的值。

Public/		软延时、系统时钟、USART驱动在这里。
UCOSIII/		ucos3操作系统部分。
user/main.c	主程序。

程序用到了ftp，服务器地址的设置在app/sim7600/sim7600.c中
第141行，服务器IP地址	123.206.19.33	if(sim7600_send_cmd((u8 *)"AT+CFTPSERV=\"123.206.19.33\"",(u8 *)"OK",500))	return 6;	 
第142行，服务器端口号	21		if(sim7600_send_cmd((u8 *)"AT+CFTPPORT=21",(u8 *)"OK",500))	return 7;	 
第145行，服务器用户名	ftpu		if(sim7600_send_cmd((u8 *)"AT+CFTPUN=\"ftpu\"",(u8 *)"OK",500))	return 10;	 
第146行，服务器密码	123456		if(sim7600_send_cmd((u8 *)"AT+CFTPPW=\"123456\"",(u8 *)"OK",500))	return 11;

联系方式：
周峰 wx: 739383146 Email: zf739383146@hotmail.com