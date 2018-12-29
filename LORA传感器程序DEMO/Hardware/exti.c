/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * �ļ����ƣ�exti.c
 * �ļ���ʶ��
 * ժ    Ҫ���ⲿ�ж�0����
 * 
 * ��ǰ�汾��V1.0.0
 * ��    �ߣ�yjd
 * ������ڣ�2017-03-23
**************************************************/
#include "apply.h"  
#include "exti.h"


/*******************************************************************************
 * ����: void Exti_Init(void)
 * ����: �ⲿ�жϳ�ʼ��
 * �β�: ��
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/
void Exti_Init(void)
{
  EXTI_DeInit();
  GPIO_Init(SONIC_PORT, ECHO_PIN, GPIO_Mode_In_FL_IT);
  EXTI_SetPinSensitivity(EXTI_Pin_0, EXTI_Trigger_Falling);
  ITC_SetSoftwarePriority(EXTI0_IRQn, ITC_PriorityLevel_0);
  EXTI_ClearITPendingBit(EXTI_IT_Pin0);
  disableInterrupts();
}

/*******************************************************************************
 * ����: INTERRUPT_HANDLER(EXTI0_IRQHandler,8)
 * ����: �ⲿ�ж�0����
 * �β�: ��
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/
INTERRUPT_HANDLER(EXTI0_IRQHandler, 8)
{
  if(EXTI_GetITStatus(EXTI_IT_Pin0))
  {
     g_distance_data = TIM1_GetCounter();
     g_measure_succeed_flag = 1;
     EXTI_ClearITPendingBit(EXTI_IT_Pin0);
  }
}
