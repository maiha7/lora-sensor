#ifndef __APPLY_H
#define __APPLY_H
#include "stm8l15x.h"

//#define SAMPLE_NUM  10 //采样样本数目
//#define INTERVAL    50 //采样间隔时间

extern uint32_t m_sample_time;
extern uint8_t send_success;


void RTC_Config(void);
void Sysclk_LSI(void);
void Sysclk_Init(void);
void Sample_Init(void);
void Sampledata_Send(void);
void Sampledata_Read(int ms);
void Goto_Haltmode(void);
void Gpio_Output_LowPower(void);
void ascending_sort(uint16_t a[],uint8_t n);
float get_averagedata(float a[],uint8_t n);
int Round(float x);


#endif