#include "system.h"
//#include "core_cm3.h"

void soft_reset(void)
{
	//�ر������ж�
  __set_FAULTMASK(1); 
  // �������
  NVIC_SystemReset(); 
}
