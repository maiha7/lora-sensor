/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * 文件名称：ds18b20.c
 * 文件标识：
 * 摘    要：温度传感器DS18B20驱动程序
 * 
 * 当前版本：V1.0.0
 * 作    者：yjd
 * 完成日期：2017-09-29
**************************************************/
#include "led.h"
#include "exti.h"
#include "stm8l15x.h"
#include "stdio.h"
#include "ds18b20.h"
#include "apply.h"

uint8_t g_ds18b20init_success = 0; //采集初始化标志位
float g_ds18b20_temperature = 0; //ds18b20 温度值
EDs18b20sampleState eDs18b20sampleState;
/*******************************************************************************
 * 名称: void Ds18b20Gpio_Init(void)
 * 功能: DS18B20管脚初始化
 * 形参: 无
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/
void Ds18b20Gpio_Init(void)
{ 

  DS18B20_PW_OUT;
  DS18B20_DQ_OUT;

  DS18B20_DQ_LOW;
  DS18B20_PW_HIGH;
}
/*******************************************************************************
 * 名称: void DS18B20_Init(void)
 * 功能: DS18B20初始化复位
 * 形参: 无
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/  
void DS18B20_Init(void)
{  
  DS18B20_DQ_OUT; 
  DS18B20_DQ_HIGH; 
  delay_us(5);
  DS18B20_DQ_LOW; 
  delay_us(600); //复位脉冲
  DS18B20_DQ_HIGH;
  DS18B20_DQ_IN;
  delay_us(100); //大于60us ,2分频实际为100us

  if(DS18B20_DQ_VALUE == DS18B20_DQ_DATA)
  {
     g_ds18b20init_success = 0;
  }
  else
  {
     g_ds18b20init_success = 1;
  }
  DS18B20_DQ_OUT;
  delay_us(400);

}

/*******************************************************************************
 * 名称: void DS18B20_WriteByte(uint8_t data)
 * 功能: DS18B20写字节函数
 * 形参: uint8_t data
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/  
void DS18B20_WriteByte(uint8_t data)
{
  uint8_t i = 0;
  DS18B20_DQ_OUT;
  for (i = 0; i < 8; i++)
  {
    if(data & 0x01)
    {
      DS18B20_DQ_HIGH;
      delay_us(1);
      DS18B20_DQ_LOW;
      delay_us(15);
      DS18B20_DQ_HIGH;
      delay_us(100);
    }
   else
   {
      DS18B20_DQ_HIGH;
      delay_us(1);
      DS18B20_DQ_LOW;
      delay_us(100);
      DS18B20_DQ_HIGH;
      delay_us(15);
   }
   data >>= 1; 
  }
}
//void DS18B20_WriteByte(uint8_t data)
//{
//  uint8_t i = 0;
//  DS18B20_DQ_OUT;
//  for (i = 0; i < 8; i++)
//  {
//    DS18B20_DQ_LOW;
//    delay_us(2);
//    if (data & 0x01)
//    {
//      DS18B20_DQ_HIGH;
//    }
//    data >>= 1;
//    delay_us(100);
//    DS18B20_DQ_HIGH;
//  }
//}
/*******************************************************************************
 * 名称: uint8_t DS18B20_ReadByte(void)
 * 功能: DS18B20读取字节函数
 * 形参: 无
 * 返回: data
 * 说明: 无 
 ******************************************************************************/ 
uint8_t DS18B20_ReadByte(void)
{
  uint8_t i = 0;
  uint8_t data = 0;
  for (i = 0; i < 8; i++)
  {
    DS18B20_DQ_OUT;
    DS18B20_DQ_HIGH;
    delay_us(1);
    DS18B20_DQ_LOW;
    delay_us(5);
    DS18B20_DQ_HIGH;
    DS18B20_DQ_IN;
    data >>= 1;
    if (DS18B20_DQ_VALUE == DS18B20_DQ_DATA)
    {
      data |= 0x80;
    }
    delay_us(100);
    DS18B20_DQ_OUT; 
    DS18B20_DQ_HIGH;
  }
  return data;
}

//uint8_t DS18B20_ReadByte(void)
//{
//  uint8_t i = 0;
//  uint8_t data = 0;
//  for (i = 0; i < 8; i++)
//  {
//    DS18B20_DQ_OUT;
//    DS18B20_DQ_LOW;
//    delay_us(5);
//    data >>= 1;
//    DS18B20_DQ_HIGH;
//    DS18B20_DQ_IN;
//    if (DS18B20_DQ_VALUE == DS18B20_DQ_DATA)
//    {
//      data |= 0x80;
//    }
//    DS18B20_DQ_OUT; 
//    DS18B20_DQ_HIGH;
//    delay_us(100);
//  }
//    return data;
//}

/*******************************************************************************
 * 名称: void DS18B20_ReadTemperature(int ms)
 * 功能: DS18B20读取温度转换
 * 形参: 无
 * 返回: t；温度值
 * 说明: 无 
 ******************************************************************************/  
void DS18B20_ReadTemperature(int ms)
{
  uint16_t templsb = 0; //低位字节
  uint16_t tempmsb = 0;//高位字节
  uint16_t temp = 0;//16位字节
  static  uint8_t init_cnt = 0;//初始化次数
  static uint8_t ds18b20_sample_cnt = 0;//连续采样次数
  static uint16_t ds18b20_sample_time = 0; //定义ds18b20采样时间
  
    switch(eDs18b20sampleState)
    {
     case DS18B20_INIT:
         DS18B20_Init();
         init_cnt++;
         if(g_ds18b20init_success)
         {
          init_cnt = 0;
          eDs18b20sampleState = DS18B20SAMPLE_START;
         }
         if(init_cnt > 3) //初始化失败3次以上
         {
          init_cnt = 0;
          g_ds18b20_temperature = 128.5;
          eDs18b20sampleState = DS18B20SAMPLE_COMPLETE;
         }         
       break;
     case DS18B20SAMPLE_START:
         DS18B20_WriteByte(0xcc);
         DS18B20_WriteByte(0x44);
         DS18B20_PW_LOW;
         eDs18b20sampleState = DS18B20SAMPLE_READ;
       break;
     case DS18B20SAMPLE_READ:
         ds18b20_sample_time += ms;
         if(ds18b20_sample_time > 800)
          {
            ds18b20_sample_time = 0;
            ds18b20_sample_cnt++;
            if(g_ds18b20init_success)
            {
              DS18B20_PW_HIGH;
              DS18B20_Init();
              DS18B20_WriteByte(0xcc);
              DS18B20_WriteByte(0xbe);

              templsb = DS18B20_ReadByte();
              tempmsb = DS18B20_ReadByte();
              temp = (tempmsb << 8) | templsb;
              if(temp & 0x8000) //最高位为1温度为负
              {
                g_ds18b20_temperature = - ((~temp)+ 1) * 0.0625;
              }
              else
              {
                g_ds18b20_temperature = temp * 0.0625;
              }
              if((g_ds18b20_temperature >= -55)&&(g_ds18b20_temperature <=125))
              {
                ds18b20_sample_cnt = 0; //采样次数清零
                eDs18b20sampleState = DS18B20SAMPLE_COMPLETE;
              }
              else
              {
                eDs18b20sampleState = DS18B20SAMPLE_START; //如果采样数据不对，再重新采样一次
              }
            }
            if(ds18b20_sample_cnt > 1)//连续2次采样失败,
            {
                g_ds18b20_temperature = 128.5;
                ds18b20_sample_cnt = 0;
                eDs18b20sampleState = DS18B20SAMPLE_COMPLETE;
            }
          }
         break;
       case DS18B20SAMPLE_COMPLETE:
         break;
       default:
         break; 
  }
}
/*******************************************************************************
 * 名称: float Get_DS18B20_ReadTemperature(void)
 * 功能: DS18B20读取温度值取平均
 * 形参: 无
 * 返回: avg_temperature；平均温度值
 * 说明: 无 
 ******************************************************************************/  
//float Get_DS18B20_ReadTemperature(void)
//{
//    float avg_temperature = 0;
//    float temperature[SAMPLE_NUM] = {0};
//    uint8_t i = 0;
//    for(i = 0; i < SAMPLE_NUM; i++)
//    {
//     temperature[i] = DS18B20_ReadTemperature();
//     delay_ms(INTERVAL);
//    }
//    avg_temperature = get_averagedata(temperature,SAMPLE_NUM);
//    return avg_temperature;
//}