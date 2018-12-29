/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * 文件名称：tim4.c
 * 文件标识：
 * 摘    要：定时器4配置
 * 
 * 当前版本：V1.0.0
 * 作    者：yjd
 * 完成日期：2017-03-23
**************************************************/

#include "tim4.h"
#include "stm8l15x.h"
#include "stm8l15x_it.h"

void DelayTimer_Init(void)
{
  TIM4_DeInit();
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);
  TIM4_TimeBaseInit(TIM4_Prescaler_1 , TIM4_PERIOD);
    /* --清除TIM4溢出更新标志位-- */
  TIM4_ClearFlag(TIM4_FLAG_Update);
}