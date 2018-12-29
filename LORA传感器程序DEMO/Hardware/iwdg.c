/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * �ļ����ƣ�iwdg.c
 * �ļ���ʶ��
 * ժ    Ҫ���������Ź�����
 * 
 * ��ǰ�汾��V1.0.0
 * ��    �ߣ�yjd
 * ������ڣ�2017-03-23
**************************************************/

#include "iwdg.h"
#include "stm8l15x.h"

/*******************************************************************************
 * ����: void Iwdg_Config(void)
 * ����: �������Ź����ú���
 * �β�: ��
 * ����: ��
 * ˵��: �� 
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
