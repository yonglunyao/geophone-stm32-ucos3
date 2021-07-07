#ifndef _exti_H
#define _exti_H


#include "system.h"
#include "includes.h"

void My_EXTI_Init(void);
extern OS_TCB ReadTaskTCB;
extern OS_TCB GpsTaskTCB;
extern OS_SEM read; //多值信号量


#endif
