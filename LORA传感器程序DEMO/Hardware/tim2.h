#ifndef __TIM2_H
#define __TIM2_H
#include "stm8l15x.h"



/* Private define ------------------------------------------------------------*/
#define TIM2_PERIOD                   1000

extern uint32_t g_MsTimeCnt;
extern uint32_t adtime;

void tim2_init(void);

#endif