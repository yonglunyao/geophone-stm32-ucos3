#include "system.h"
//#include "core_cm3.h"

void soft_reset(void)
{
	//关闭所有中断
  __set_FAULTMASK(1); 
  // 软件服务
  NVIC_SystemReset(); 
}
