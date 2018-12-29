/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * 文件名称：tim1.c
 * 文件标识：
 * 摘    要：定时器1配置和中断
 * 
 * 当前版本：V1.0.0
 * 作    者：yjd
 * 完成日期：2017-03-23
**************************************************/
#include "tim1.h"
#include "led.h"
#include "stm8l15x.h"
#include "stm8l15x_itc.h"
#include "stm8l15x_it.h"

/*******************************************************************************
 * 名称: void tim1_init(void)
 * 功能: 定时器1初始化
 * 形参: 无
 * 返回: 无
 * 说明: 外部8M晶振，8分频，从0计数到65535
 ******************************************************************************/
void tim1_init(void)
{
  TIM1_DeInit();
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM1, ENABLE);
  TIM1_TimeBaseInit(TIM1_PRESCALER, TIM1_CounterMode_Up, TIM1_PERIOD, TIM1_REPTETION_COUNTER);
    /* --清除TIM1溢出更新标志位-- */
  TIM1_ClearFlag(TIM1_FLAG_Update);
  /* --使能TIM1溢出更新中断-- */
  TIM1_ITConfig(TIM1_IT_Update, ENABLE);
  ITC_SetSoftwarePriority(TIM1_UPD_OVF_TRG_IRQn , ITC_PriorityLevel_1);
  TIM1_SetCounter(0); 
  TIM1_ClearITPendingBit(TIM1_IT_Update);
  TIM1_Cmd(ENABLE);
}

/*******************************************************************************
 * 名称: INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_COM_IRQHandler,23)
 * 功能: 定时器1中断函数
 * 形参: TIM1_UPD_OVF_TRG_COM_IRQHandler
 * 返回: 无
 * 说明: 65.535ms中断产生
 ******************************************************************************/
INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_COM_IRQHandler, 23)
{
  if(TIM1_GetITStatus(TIM1_IT_Update))
  {
    TIM1_ClearITPendingBit(TIM1_IT_Update);
//    trig_pulse(); 
  }
}
  

