/* CH376芯片 硬件抽象层 V1.0 */
/* 提供基本子程序 */

#include	"HAL.H"

/* 延时指定微秒时间,根据单片机主频调整,不精确 */
void	mDelayuS( UINT8 us )
{
	while ( us -- );  /* 24MHz MCS51 */
}

/* 延时指定毫秒时间,根据单片机主频调整,不精确 */
void	mDelaymS( UINT8 ms )
{
	while ( ms -- ) {
		mDelayuS( 250 );
		mDelayuS( 250 );
		mDelayuS( 250 );
		mDelayuS( 250 );
	}
}
