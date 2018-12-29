/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * �ļ����ƣ�gy3x.c
 * �ļ���ʶ��
 * ժ    Ҫ�����ն�ģ����������
 * 
 * ��ǰ�汾��V1.0.0
 * ��    �ߣ�yjd
 * ������ڣ�2017-09-30
**************************************************/
#include "led.h"
#include "exti.h"
#include "stm8l15x.h"
#include "stdio.h"
#include "gy3x.h"
#include "apply.h"

uint8_t gy3x_data[3] = {0};
float g_gy3x_light = 0; //gy3x ����ֵ
EGy3xsampleState  eGy3xsampleState;
/**************************************************
 * �������ƣ�void GY3x_Init(void)
 * �������ܣ�GY3X��ʼ��
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
void GY3x_Init(void)
{
    GPIO_Init(GY3X_SCL_PORT,GY3X_SCL_PIN, GPIO_Mode_Out_PP_High_Slow);//��ʼ��SCL�˿�
    GPIO_Init(GY3X_SDA_PORT,GY3X_SDA_PIN, GPIO_Mode_Out_PP_High_Slow);//��ʼ��SDA�˿�
}

/**************************************************
 * �������ƣ�void GY3x_I2cStartCondition(void)
 * �������ܣ�GY3X IIC��ʼ����
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
void GY3x_I2cStartCondition(void)
{
    GY3x_SCL_OUTPUT();
    GY3x_SDA_OUTPUT();
    
    GY3x_SDA_HIGH();
    GY3x_SCL_HIGH();
    delay_us(5);
    GY3x_SDA_LOW();
    delay_us(5);
    GY3x_SCL_LOW();

}
/**************************************************
 * �������ƣ�void GY3x_I2cStopCondition(void)
 * �������ܣ�GY3X IIC��������
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
void GY3x_I2cStopCondition(void)
{
    GY3x_SCL_OUTPUT();
    GY3x_SDA_OUTPUT();
    
    GY3x_SDA_LOW();
//    GY3x_SCL_LOW();
    GY3x_SCL_HIGH();
    delay_us(5);
    GY3x_SDA_HIGH();
    delay_us(5);
}

/**************************************************
 * �������ƣ�void Single_Write_GY3x(uint8_t REG_Address)
 * �������ܣ�����дGY3xģ��Ĵ���
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/

void Single_Write_GY3x(uint8_t REG_Address)
{
    GY3x_I2cStartCondition();        //��ʼ�ź�
    GY3x_SendByte(SlaveAddress);   //�����豸��ַ
    GY3x_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ 
  //  GY3x_SendByte(REG_data);       //�ڲ��Ĵ������� 
    GY3x_I2cStopCondition();       //ֹͣ�ź�
}

/**************************************************
 * �������ƣ�uint8_t Single_Read_GY3x(uint8_t REG_Address)
 * �������ܣ����ζ�GY3xģ��Ĵ���
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
uint8_t Single_Read_GY3x(uint8_t REG_Address)
{   
    uint8_t REG_data;
    GY3x_I2cStartCondition();        //��ʼ�ź�                          
    GY3x_SendByte(SlaveAddress);   //�����豸��ַ           
    GY3x_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ                   	
    GY3x_I2cStartCondition();        //��ʼ�ź�                       
    GY3x_SendByte(SlaveAddress+1);         
    REG_data = GY3x_RecvByte();              
    GY3x_SendACK(1);   
    GY3x_I2cStopCondition();       //ֹͣ�ź�                          
    return REG_data; 
}

/**************************************************
 * �������ƣ�void Multiple_Read_GY3x(void)
 * �������ܣ���������GY3xģ������
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
void Multiple_Read_GY3x(void)
{   
    uint8_t i;
    GY3x_I2cStartCondition();        //��ʼ�ź�                        
    GY3x_SendByte(SlaveAddress+1);         //
    for (i = 0; i < 3; i++)                   //
    {
        gy3x_data[i] = GY3x_RecvByte();          //
        if (i == 3)
        {
           GY3x_SendACK(1);                //
        }
        else
        {		
           GY3x_SendACK(0);                //
       }
   }

    GY3x_I2cStopCondition();       //ֹͣ�ź� 
    delay_ms(5);
}

/**************************************************
 * �������ƣ�void GY3x_SendByte(uint8_t dat)
 * �������ܣ���GY3Xģ�鷢��һ���ֽ�
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
void GY3x_SendByte(uint8_t dat)
{
    uint8_t i;
    for (i = 0; i < 8; i++)         
    {
        if(dat & 0x80) 
        {
            GY3x_SDA_HIGH();
        }
        else 
        {
            GY3x_SDA_LOW();
        }
        dat <<= 1;              //��������λ
        GY3x_SCL_HIGH();     //
        delay_us(5);          //
        GY3x_SCL_LOW();          //
        delay_us(5);            //
    }
    GY3x_RecvACK();
}
/**************************************************
 * �������ƣ�uint8_t GY3x_RecvByte(void)
 * �������ܣ���GY3Xģ�����һ���ֽ�
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
uint8_t GY3x_RecvByte(void)
{
    uint8_t i;
    uint8_t dat = 0;
    GY3x_SCL_OUTPUT();
    GY3x_SDA_OUTPUT();

    GY3x_SDA_HIGH();
    
    GY3x_SDA_INPUT();
        
    for(i = 0; i < 8; i++)
    {
        dat <<= 1; 
        
        GY3x_SCL_HIGH();   
        
        delay_us(5);
        
        if(0x40 == GY3x_SDA_STATE()) 
        {
            dat |= 0x01;
        }
        
        GY3x_SCL_LOW(); 
        
        delay_us(5);
    }

    GY3x_SDA_OUTPUT();

    GY3x_SDA_HIGH();
    
    return (dat);

}
/**************************************************
 * �������ƣ�uint8_t GY3x_RecvACK()
 * �������ܣ�����GY3Xģ���Ӧ��
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
uint8_t GY3x_RecvACK(void)
{
    uint8_t ack;
    GY3x_SDA_INPUT();   
    GY3x_SCL_HIGH();  
    delay_us(5);
    if(0x40 == GY3x_SDA_STATE()) 
    {
        ack = 1;
    }
    else
    {
        ack = 0;
    }
    GY3x_SCL_LOW();  
    delay_us(5);   
    GY3x_SDA_OUTPUT();
    GY3x_SDA_HIGH();   
    return (ack);   
}

/**************************************************
 * �������ƣ�void GY3x_SendACK(uint8_t ack)
 * �������ܣ���GY3Xģ�鷢��Ӧ��
 * ���������ack (0:ACK 1:NAK)
 * ���������
 * ����ֵ  ��
 **************************************************/
void GY3x_SendACK(uint8_t ack)
{
    GY3x_SDA_OUTPUT();
    if(ack)
    {
      GY3x_SDA_HIGH();  
    }
    else
    {
      GY3x_SDA_LOW();   
    }
    GY3x_SCL_HIGH();
    delay_us(5);
    GY3x_SCL_LOW();                    
    delay_us(5);
}

/**************************************************
 * �������ƣ�void GY3x_ReadLightdata(int ms)
 * �������ܣ���ȡ��������
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
void GY3x_ReadLightdata(int ms)
{
    int   light_temp; 
    static  uint8_t gy3x_sample_cnt = 0; //gy3x��������
    static  uint16_t gy3x_sample_time = 0; //gy3x����ʱ��
    switch(eGy3xsampleState)
    {
     case GY3X_SAMPLE_START:
        Single_Write_GY3x(0x01);   // power on
        Single_Write_GY3x(0x10);   // H- resolution mode
        eGy3xsampleState = GY3X_SAMPLE_READ;
       break;
     case GY3X_SAMPLE_READ:
        gy3x_sample_time += ms;
        if(gy3x_sample_time > 180)
        {
          gy3x_sample_cnt++; //����������1
          gy3x_sample_time = 0;
          Multiple_Read_GY3x(); 
          light_temp = gy3x_data[0];
          light_temp = (light_temp << 8) + gy3x_data[1];
          g_gy3x_light = (float) light_temp / 1.2;
          if(g_gy3x_light > 0)
          {
           gy3x_sample_cnt = 0;
           eGy3xsampleState = GY3X_SAMPLE_COMPLETE;
          }
          else
          {
             if(gy3x_sample_cnt > 3) //����3�β���ֵС���㣬����ֵ��ֵ��
             {
               g_gy3x_light = 0;
               gy3x_sample_cnt = 0;
               eGy3xsampleState = GY3X_SAMPLE_COMPLETE;
             }
             else
             {
               eGy3xsampleState = GY3X_SAMPLE_START; //С��3�����²���
             }
          }
        }
       break;
     case GY3X_SAMPLE_COMPLETE:
       break;
     default:
      break;
    }
}

/*******************************************************************************
 * ����: float Get_GY3x_Lightdata(void)
 * ����: GY3X��ȡ����ֵȡƽ��
 * �β�: ��
 * ����: avg_light��ƽ������ֵ
 * ˵��: �� 
 ******************************************************************************/  
//float Get_GY3x_Lightdata(void)
//{
//    float avg_light = 0;
//    float light[SAMPLE_NUM] = {0};
//    uint8_t i = 0;
//    for(i = 0; i < SAMPLE_NUM; i++)
//    {
//     light[i] = GY3x_ReadLightdata();
//     delay_ms(INTERVAL);
//    }
//    avg_light = get_averagedata(light,SAMPLE_NUM);
//    return avg_light;
//}