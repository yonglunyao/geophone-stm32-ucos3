stm32f407����
app/ads1256/	ads1256��������
app/ch376/	usbģ���������
app/dma/	DMAʵ�֡�
app/exti/		�ⲿ�жϵ�ʵ�֡�
app/gps/		GPSģ��������
app/iic/		I2C���������ڿ���ads1282��ʾ��ģ�ads1256��û���õ���
app/iwdg/	���Ź��Ĵ��룬���ڳ����ܷɺ��������
app/sim7600	4gģ��������
app/spi		��·dma spi��������
app/time		��ʱ����

app/sd/		���ĸ��ļ���ʵ����SD����FAT32�ļ�ϵͳ��֧�֣����Ӵ�͸��ӣ�ֻ��Ϊ���õĻ�û��Ҫȥ�ĺͿ�����ռ��rom�ռ��ram�ռ䶼�ܴ�
app/flash/	rom�ռ���Ҫ��fatfs/ffconf.h�ļ��е����ã�Ŀǰ��Ӣ��8.3��ʽ�ļ���������ʵ�������ļ������ļ�����֧�֣���rom�ռ�����������ӡ�
fatfs/		ram�ռ���Ҫ��malloc/malloc.h�е����ã���Ƭ����֧��malloc�ģ�����ͨ�����ȶ��弸ʮkb�����鵱���ڴ�أ���������������н��ж�̬
malloc/		�ڴ���䣬���ram���������Կ��Ǽ�С�Ǽ�ʮkb��ֵ��

Public/		����ʱ��ϵͳʱ�ӡ�USART���������
UCOSIII/		ucos3����ϵͳ���֡�
user/main.c	������

�����õ���ftp����������ַ��������app/sim7600/sim7600.c��
��141�У�������IP��ַ	123.206.19.33	if(sim7600_send_cmd((u8 *)"AT+CFTPSERV=\"123.206.19.33\"",(u8 *)"OK",500))	return 6;	 
��142�У��������˿ں�	21		if(sim7600_send_cmd((u8 *)"AT+CFTPPORT=21",(u8 *)"OK",500))	return 7;	 
��145�У��������û���	ftpu		if(sim7600_send_cmd((u8 *)"AT+CFTPUN=\"ftpu\"",(u8 *)"OK",500))	return 10;	 
��146�У�����������	123456		if(sim7600_send_cmd((u8 *)"AT+CFTPPW=\"123456\"",(u8 *)"OK",500))	return 11;

��ϵ��ʽ��
�ܷ� wx: 739383146 Email: zf739383146@hotmail.com