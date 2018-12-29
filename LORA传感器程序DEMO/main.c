/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * 文件名称：main.c
 * 文件标识：
 * 摘    要：主函数处理
 * 
 * 当前版本：V1.0.0
 * 作    者：
 * 完成日期：2017-03-23
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
1）进入低功耗模式的流程
①关闭外设时钟。
②所有IO设置为输出低（注意低电平使能的得设置为输出高）。
③初始化中断口的IO。
④时钟频率降至最低（LSI）。
⑤打开中断总开关。
⑥调用停机模式的函数。

2）低功耗唤醒的流程5555
①初始化系统时钟。
②初始化用到的外设
*/

//系统运行时间
uint32_t ms = 0;  
void main(void)
{
    // 硬件初始化
    DriverConfig();
    // 软件配置信息初始化
    SoftwareInit();
    while (1)
    {
        //读取运行时间差值
        ms = SysTimeGetms();
        //驱动任务运行
        DriverTask(ms);
        //串口运行
        Comm_Run(ms);
        //ad采样
        AD_Sample_data();
    }
}