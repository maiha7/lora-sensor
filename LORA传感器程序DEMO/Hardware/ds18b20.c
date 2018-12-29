/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * �ļ����ƣ�ds18b20.c
 * �ļ���ʶ��
 * ժ    Ҫ���¶ȴ�����DS18B20��������
 * 
 * ��ǰ�汾��V1.0.0
 * ��    �ߣ�yjd
 * ������ڣ�2017-09-29
**************************************************/
#include "led.h"
#include "exti.h"
#include "stm8l15x.h"
#include "stdio.h"
#include "ds18b20.h"
#include "apply.h"

uint8_t g_ds18b20init_success = 0; //�ɼ���ʼ����־λ
float g_ds18b20_temperature = 0; //ds18b20 �¶�ֵ
EDs18b20sampleState eDs18b20sampleState;
/*******************************************************************************
 * ����: void Ds18b20Gpio_Init(void)
 * ����: DS18B20�ܽų�ʼ��
 * �β�: ��
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/
void Ds18b20Gpio_Init(void)
{ 

  DS18B20_PW_OUT;
  DS18B20_DQ_OUT;

  DS18B20_DQ_LOW;
  DS18B20_PW_HIGH;
}
/*******************************************************************************
 * ����: void DS18B20_Init(void)
 * ����: DS18B20��ʼ����λ
 * �β�: ��
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/  
void DS18B20_Init(void)
{  
  DS18B20_DQ_OUT; 
  DS18B20_DQ_HIGH; 
  delay_us(5);
  DS18B20_DQ_LOW; 
  delay_us(600); //��λ����
  DS18B20_DQ_HIGH;
  DS18B20_DQ_IN;
  delay_us(100); //����60us ,2��Ƶʵ��Ϊ100us

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
 * ����: void DS18B20_WriteByte(uint8_t data)
 * ����: DS18B20д�ֽں���
 * �β�: uint8_t data
 * ����: ��
 * ˵��: �� 
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
 * ����: uint8_t DS18B20_ReadByte(void)
 * ����: DS18B20��ȡ�ֽں���
 * �β�: ��
 * ����: data
 * ˵��: �� 
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
 * ����: void DS18B20_ReadTemperature(int ms)
 * ����: DS18B20��ȡ�¶�ת��
 * �β�: ��
 * ����: t���¶�ֵ
 * ˵��: �� 
 ******************************************************************************/  
void DS18B20_ReadTemperature(int ms)
{
  uint16_t templsb = 0; //��λ�ֽ�
  uint16_t tempmsb = 0;//��λ�ֽ�
  uint16_t temp = 0;//16λ�ֽ�
  static  uint8_t init_cnt = 0;//��ʼ������
  static uint8_t ds18b20_sample_cnt = 0;//������������
  static uint16_t ds18b20_sample_time = 0; //����ds18b20����ʱ��
  
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
         if(init_cnt > 3) //��ʼ��ʧ��3������
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
              if(temp & 0x8000) //���λΪ1�¶�Ϊ��
              {
                g_ds18b20_temperature = - ((~temp)+ 1) * 0.0625;
              }
              else
              {
                g_ds18b20_temperature = temp * 0.0625;
              }
              if((g_ds18b20_temperature >= -55)&&(g_ds18b20_temperature <=125))
              {
                ds18b20_sample_cnt = 0; //������������
                eDs18b20sampleState = DS18B20SAMPLE_COMPLETE;
              }
              else
              {
                eDs18b20sampleState = DS18B20SAMPLE_START; //����������ݲ��ԣ������²���һ��
              }
            }
            if(ds18b20_sample_cnt > 1)//����2�β���ʧ��,
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
 * ����: float Get_DS18B20_ReadTemperature(void)
 * ����: DS18B20��ȡ�¶�ֵȡƽ��
 * �β�: ��
 * ����: avg_temperature��ƽ���¶�ֵ
 * ˵��: �� 
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