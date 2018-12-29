/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * 文件名称：tim1.c
 * 文件标识：
 * 摘    要：定时器2配置和中断
 * 
 * 当前版本：V1.0.0
 * 作    者：yjd
 * 完成日期：2017-03-23
**************************************************/

#include "tim2.h"
//#include "rs485.h"
#include "stm8l15x.h"
#include "stm8l15x_it.h"
#include "usart_t.h"

uint32_t g_MsTimeCnt = 0;//用于系统时钟运行
uint32_t adtime = 0;//用于ad采集时间
/*******************************************************************************
 * 名称: void tim2_init(void)
 * 功能: 定时器2初始化
 * 形参: 无
 * 返回: 无
 * 说明: 外部晶振8M，8分频，周期10ms
 ******************************************************************************/
void tim2_init(void)
{
  TIM2_DeInit();
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, ENABLE);
  TIM2_TimeBaseInit(TIM2_Prescaler_8 , TIM2_CounterMode_Up, TIM2_PERIOD);
    /* --清除TIM2溢出更新标志位-- */
  TIM2_ClearFlag(TIM2_FLAG_Update);
  /* --使能TIM2溢出更新中断-- */
  TIM2_ITConfig(TIM2_IT_Update, ENABLE);
//  ITC_SetSoftwarePriority(TIM1_UPD_OVF_TRG_IRQn, ITC_PriorityLevel_2);
//  TIM2_SetCounter(0); 
  TIM2_Cmd(ENABLE);
}



/*******************************************************************************
 * 名称: INTERRUPT_HANDLER(TIM2_UPD_OVF_TRG_BRK_USART2_TX_IRQHandler,19)
 * 功能: 定时器2中断函数
 * 形参: 无
 * 返回: 无
 * 说明: 周期10ms中断
 ******************************************************************************/
INTERRUPT_HANDLER(TIM2_UPD_OVF_TRG_BRK_USART2_TX_IRQHandler, 19)
{ 
    
    PCCOMBUFFER  pCComBuffer;
    pCComBuffer = &g_CComBuffer[0];
    int i = 0;
    if(TIM2_GetITStatus(TIM2_IT_Update) != RESET)
    {
        TIM2_ClearITPendingBit(TIM2_IT_Update);
        //串口截止时间
        if(pCComBuffer->RxdDelayCnt != 0)
        {
            pCComBuffer->RxdDelayCnt--;
        }
    
//        if(RS485_RX_CNT)  //接收到数据
//        {
//            if(RS485_RX_LEN != RS485_RX_CNT) //仍有数据接收到
//            {
//                RS485_RX_LEN =  RS485_RX_CNT;
//            }
//            else
//            {
//                RS485_RX_END = TRUE; //超时结束
//                RS485_RX_CNT = 0; //接收计数清零
//            }     
//        }
        
        g_MsTimeCnt++;  //用于系统时钟运行
        i++;
        if(i=1000)
        {
          i = 0;
          adtime = 1;
        }
        else adtime = 0;
          
    }
}