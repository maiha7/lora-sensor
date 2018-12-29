// 驱动配置
#ifndef __DY_DRIVER_CONFIG_H__
#define __DY_DRIVER_CONFIG_H__
#include "stm8l15x.h"

//低功耗模式状态标志
extern uint8_t g_LowPowerCntState;
extern uint16_t acData;
//低功耗控制宏
#define  LOW_POWER_CNT_STATE_HALT    1 //停机模式
#define  LOW_POWER_CNT_STATE_ACTIVE  0 //工作状态

typedef enum
{
  SampleRead_State = 0,//采样读取状态
  DataSend_State, //数据发送状态
  GotoHalt_State //进入停机状态
}EDriverState;

extern EDriverState  eDriverState;
//AD
extern void DrADC_Init(void);
extern float ADC_NH3_Read(void);
//GPIO
void DrGPIO_Init(void);
void DrGPIO_Write(int index, int value);
int GPIO_Read(int index);
//UART
void UART_Init(void);
int UART_Write(void* p, int len);
int UART_Read(void* p, int len);
//ROM
void ROM_Init(void);
int ROM_Write(int addr, void* p, int len);
int ROM_Read(int addr, void* p, int len);


void Comm_Run(int ms);
// bsp初始化
extern void BspInit(void);
// 驱动配置
extern void DriverConfig(void);
// 驱动任务, ms:运行时间(毫秒)
extern void DriverTask(int ms);
// 时间延迟, ms:延迟时间(毫秒)
extern void TimeDelay(int ms);
// 模拟器睡眠(实际驱动的实现函数为空), ms:睡眠时间(毫秒)
extern  void SimSleep(uint8_t LowPowerCnt_State);
// 得到毫秒数
extern uint32_t SysTimeGetms(void);
// 清除看门狗标志
extern void  Clear_IWDG_Flag(void);
// 软件系统变量初始化
extern void SoftwareInit(void);
// 系统进入低功耗
extern void LowPowerConsumption_Cmd(uint8_t LowPowerCnt_State);  
#endif // __DY_DRIVER_CONFIG_H__
