/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * 文件名称：flash.c
 * 文件标识：
 * 摘    要：距离数据采集和处理
 * 
 * 当前版本：V1.0.0
 * 作    者：yjd
 * 完成日期：2017-03-23
**************************************************/
#include "flash.h" 

/*******************************************************************************
 * 名称: void Flash_Init(void)
 * 功能: flash初始化
 * 形参: 无
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/
void Flash_Init(void)
{
  FLASH_DeInit();
  FLASH_Unlock(FLASH_MemType_Data);//先解锁
}
  
