/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * 文件名称：iwdg.c
 * 文件标识：
 * 摘    要：独立看门狗配置
 * 
 * 当前版本：V1.0.0
 * 作    者：yjd
 * 完成日期：2017-03-23
**************************************************/

#include "iwdg.h"
#include "stm8l15x.h"

/*******************************************************************************
 * 名称: void Iwdg_Config(void)
 * 功能: 独立看门狗配置函数
 * 形参: 无
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/
void Iwdg_Config(void)
{
    /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();
  /* IWDG timeout equal to 1717.9ms (the timeout may varies due to LSI frequency
     dispersion) */
  /* Enable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  
  /* IWDG configuration: IWDG is clocked by LSI = 38KHz */
  IWDG_SetPrescaler(IWDG_Prescaler_256);
  
  /* IWDG timeout equal to 1717.9 ms (the timeout may varies due to LSI frequency dispersion) */
  /* IWDG timeout = (RELOAD_VALUE + 1) * Prescaler / LSI 
                  = (254 + 1) * 256 / 38 000 
                  = 1717.9 ms */
  IWDG_SetReload((uint8_t)RELOAD_VALUE);
  
  /* Reload IWDG counter */
  IWDG_ReloadCounter();
  
}
