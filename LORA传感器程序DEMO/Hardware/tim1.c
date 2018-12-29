/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * �ļ����ƣ�tim1.c
 * �ļ���ʶ��
 * ժ    Ҫ����ʱ��1���ú��ж�
 * 
 * ��ǰ�汾��V1.0.0
 * ��    �ߣ�yjd
 * ������ڣ�2017-03-23
**************************************************/
#include "tim1.h"
#include "led.h"
#include "stm8l15x.h"
#include "stm8l15x_itc.h"
#include "stm8l15x_it.h"

/*******************************************************************************
 * ����: void tim1_init(void)
 * ����: ��ʱ��1��ʼ��
 * �β�: ��
 * ����: ��
 * ˵��: �ⲿ8M����8��Ƶ����0������65535
 ******************************************************************************/
void tim1_init(void)
{
  TIM1_DeInit();
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM1, ENABLE);
  TIM1_TimeBaseInit(TIM1_PRESCALER, TIM1_CounterMode_Up, TIM1_PERIOD, TIM1_REPTETION_COUNTER);
    /* --���TIM1������±�־λ-- */
  TIM1_ClearFlag(TIM1_FLAG_Update);
  /* --ʹ��TIM1��������ж�-- */
  TIM1_ITConfig(TIM1_IT_Update, ENABLE);
  ITC_SetSoftwarePriority(TIM1_UPD_OVF_TRG_IRQn , ITC_PriorityLevel_1);
  TIM1_SetCounter(0); 
  TIM1_ClearITPendingBit(TIM1_IT_Update);
  TIM1_Cmd(ENABLE);
}

/*******************************************************************************
 * ����: INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_COM_IRQHandler,23)
 * ����: ��ʱ��1�жϺ���
 * �β�: TIM1_UPD_OVF_TRG_COM_IRQHandler
 * ����: ��
 * ˵��: 65.535ms�жϲ���
 ******************************************************************************/
INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_COM_IRQHandler, 23)
{
  if(TIM1_GetITStatus(TIM1_IT_Update))
  {
    TIM1_ClearITPendingBit(TIM1_IT_Update);
//    trig_pulse(); 
  }
}
  

