/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * �ļ����ƣ�tim1.c
 * �ļ���ʶ��
 * ժ    Ҫ����ʱ��2���ú��ж�
 * 
 * ��ǰ�汾��V1.0.0
 * ��    �ߣ�yjd
 * ������ڣ�2017-03-23
**************************************************/

#include "tim2.h"
//#include "rs485.h"
#include "stm8l15x.h"
#include "stm8l15x_it.h"
#include "usart_t.h"

uint32_t g_MsTimeCnt = 0;//����ϵͳʱ������
uint32_t adtime = 0;//����ad�ɼ�ʱ��
/*******************************************************************************
 * ����: void tim2_init(void)
 * ����: ��ʱ��2��ʼ��
 * �β�: ��
 * ����: ��
 * ˵��: �ⲿ����8M��8��Ƶ������10ms
 ******************************************************************************/
void tim2_init(void)
{
  TIM2_DeInit();
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, ENABLE);
  TIM2_TimeBaseInit(TIM2_Prescaler_8 , TIM2_CounterMode_Up, TIM2_PERIOD);
    /* --���TIM2������±�־λ-- */
  TIM2_ClearFlag(TIM2_FLAG_Update);
  /* --ʹ��TIM2��������ж�-- */
  TIM2_ITConfig(TIM2_IT_Update, ENABLE);
//  ITC_SetSoftwarePriority(TIM1_UPD_OVF_TRG_IRQn, ITC_PriorityLevel_2);
//  TIM2_SetCounter(0); 
  TIM2_Cmd(ENABLE);
}



/*******************************************************************************
 * ����: INTERRUPT_HANDLER(TIM2_UPD_OVF_TRG_BRK_USART2_TX_IRQHandler,19)
 * ����: ��ʱ��2�жϺ���
 * �β�: ��
 * ����: ��
 * ˵��: ����10ms�ж�
 ******************************************************************************/
INTERRUPT_HANDLER(TIM2_UPD_OVF_TRG_BRK_USART2_TX_IRQHandler, 19)
{ 
    
    PCCOMBUFFER  pCComBuffer;
    pCComBuffer = &g_CComBuffer[0];
    int i = 0;
    if(TIM2_GetITStatus(TIM2_IT_Update) != RESET)
    {
        TIM2_ClearITPendingBit(TIM2_IT_Update);
        //���ڽ�ֹʱ��
        if(pCComBuffer->RxdDelayCnt != 0)
        {
            pCComBuffer->RxdDelayCnt--;
        }
    
//        if(RS485_RX_CNT)  //���յ�����
//        {
//            if(RS485_RX_LEN != RS485_RX_CNT) //�������ݽ��յ�
//            {
//                RS485_RX_LEN =  RS485_RX_CNT;
//            }
//            else
//            {
//                RS485_RX_END = TRUE; //��ʱ����
//                RS485_RX_CNT = 0; //���ռ�������
//            }     
//        }
        
        g_MsTimeCnt++;  //����ϵͳʱ������
        i++;
        if(i=1000)
        {
          i = 0;
          adtime = 1;
        }
        else adtime = 0;
          
    }
}