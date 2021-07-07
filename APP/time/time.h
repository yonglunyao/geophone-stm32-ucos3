#ifndef _time_H
#define _time_H

#include "system.h"

void TIM5_Init(u16 per,u16 psc);
void TIM2_Init(u16 per,u16 psc);
extern u16 count;
extern u16 excount;
#endif
