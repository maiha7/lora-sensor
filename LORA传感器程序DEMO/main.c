/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * �ļ����ƣ�main.c
 * �ļ���ʶ��
 * ժ    Ҫ������������
 * 
 * ��ǰ�汾��V1.0.0
 * ��    �ߣ�
 * ������ڣ�2017-03-23
**************************************************/
#include "led.h"
#include "exti.h"
#include "tim4.h"
#include "tim2.h"
#include "iwdg.h"
#include "apply.h"
#include "flash.h"
#include "stm8l15x.h"
#include "string.h"
#include "DriverConfig.h"
#include "ADC_N.h"
//#include "stm8l15x.h"
/*
1������͹���ģʽ������
�ٹر�����ʱ�ӡ�
������IO����Ϊ����ͣ�ע��͵�ƽʹ�ܵĵ�����Ϊ����ߣ���
�۳�ʼ���жϿڵ�IO��
��ʱ��Ƶ�ʽ�����ͣ�LSI����
�ݴ��ж��ܿ��ء�
�޵���ͣ��ģʽ�ĺ�����

2���͹��Ļ��ѵ�����5555
�ٳ�ʼ��ϵͳʱ�ӡ�
�ڳ�ʼ���õ�������
*/

//ϵͳ����ʱ��
uint32_t ms = 0;  
void main(void)
{
    // Ӳ����ʼ��
    DriverConfig();
    // ���������Ϣ��ʼ��
    SoftwareInit();
    while (1)
    {
        //��ȡ����ʱ���ֵ
        ms = SysTimeGetms();
        //������������
        DriverTask(ms);
        //��������
        Comm_Run(ms);
        //ad����
        AD_Sample_data();
    }
}