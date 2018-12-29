/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * �ļ����ƣ�tim4.c
 * �ļ���ʶ��
 * ժ    Ҫ����ʱ��4����
 * 
 * ��ǰ�汾��V1.0.0
 * ��    �ߣ�yjd
 * ������ڣ�2017-03-23
**************************************************/

#include "tim4.h"
#include "stm8l15x.h"
#include "stm8l15x_it.h"

void DelayTimer_Init(void)
{
  TIM4_DeInit();
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);
  TIM4_TimeBaseInit(TIM4_Prescaler_1 , TIM4_PERIOD);
    /* --���TIM4������±�־λ-- */
  TIM4_ClearFlag(TIM4_FLAG_Update);
}