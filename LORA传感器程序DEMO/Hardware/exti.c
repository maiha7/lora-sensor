/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * 文件名称：exti.c
 * 文件标识：
 * 摘    要：外部中断0处理
 * 
 * 当前版本：V1.0.0
 * 作    者：yjd
 * 完成日期：2017-03-23
**************************************************/
#include "apply.h"  
#include "exti.h"


/*******************************************************************************
 * 名称: void Exti_Init(void)
 * 功能: 外部中断初始化
 * 形参: 无
 * 返回: 无
 * 说明: 无 
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
 * 名称: INTERRUPT_HANDLER(EXTI0_IRQHandler,8)
 * 功能: 外部中断0函数
 * 形参: 无
 * 返回: 无
 * 说明: 无 
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
