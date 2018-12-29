#ifndef __DS18B20_H
#define __DS18B20_H
#include "stm8l15x.h"

typedef enum
{
  DS18B20_INIT = 0, //DS18B20采集初始化
  DS18B20SAMPLE_START,//DS18B20采集开始
  DS18B20SAMPLE_READ,//DS18B20采集读取
  DS18B20SAMPLE_COMPLETE//DS18B20采集完成
}EDs18b20sampleState;
extern EDs18b20sampleState  eDs18b20sampleState;
extern float g_ds18b20_temperature;
extern uint8_t g_ds18b20sample_complete;
#define DS18B20_PORT  GPIOD
#define DS18B20_DQ  GPIO_Pin_2
#define DS18B20_DQ_DATA  0X04 //PIN2高电平
#define DS18B20_PW  GPIO_Pin_1


#define DS18B20_PW_OUT  GPIO_Init(DS18B20_PORT,DS18B20_PW,GPIO_Mode_Out_PP_High_Slow)
#define DS18B20_PW_HIGH  GPIO_SetBits(DS18B20_PORT, DS18B20_PW)
#define DS18B20_PW_LOW   GPIO_ResetBits(DS18B20_PORT, DS18B20_PW)
#define DS18B20_DQ_OUT  GPIO_Init(DS18B20_PORT,DS18B20_DQ,GPIO_Mode_Out_PP_High_Slow)
#define DS18B20_DQ_IN   GPIO_Init(DS18B20_PORT,DS18B20_DQ,GPIO_Mode_In_PU_No_IT)
#define DS18B20_DQ_HIGH  GPIO_SetBits(DS18B20_PORT, DS18B20_DQ)
#define DS18B20_DQ_LOW   GPIO_ResetBits(DS18B20_PORT, DS18B20_DQ)
#define DS18B20_DQ_VALUE  GPIO_ReadInputDataBit(DS18B20_PORT, DS18B20_DQ)

void Ds18b20Gpio_Init(void);
void DS18B20_Init(void);
void DS18B20_WriteByte(uint8_t data);
uint8_t DS18B20_ReadByte(void);
void DS18B20_ReadTemperature(int ms);
float Get_DS18B20_ReadTemperature(void);
#endif