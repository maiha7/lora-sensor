/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * 文件名称：gy3x.c
 * 文件标识：
 * 摘    要：光照度模块驱动程序
 * 
 * 当前版本：V1.0.0
 * 作    者：yjd
 * 完成日期：2017-09-30
**************************************************/
#include "led.h"
#include "exti.h"
#include "stm8l15x.h"
#include "stdio.h"
#include "gy3x.h"
#include "apply.h"

uint8_t gy3x_data[3] = {0};
float g_gy3x_light = 0; //gy3x 光照值
EGy3xsampleState  eGy3xsampleState;
/**************************************************
 * 函数名称：void GY3x_Init(void)
 * 函数介绍：GY3X初始化
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/
void GY3x_Init(void)
{
    GPIO_Init(GY3X_SCL_PORT,GY3X_SCL_PIN, GPIO_Mode_Out_PP_High_Slow);//初始化SCL端口
    GPIO_Init(GY3X_SDA_PORT,GY3X_SDA_PIN, GPIO_Mode_Out_PP_High_Slow);//初始化SDA端口
}

/**************************************************
 * 函数名称：void GY3x_I2cStartCondition(void)
 * 函数介绍：GY3X IIC开始条件
 * 输入参数：
 * 输出参数：
 * 返回值  ：
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
 * 函数名称：void GY3x_I2cStopCondition(void)
 * 函数介绍：GY3X IIC结束条件
 * 输入参数：
 * 输出参数：
 * 返回值  ：
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
 * 函数名称：void Single_Write_GY3x(uint8_t REG_Address)
 * 函数介绍：单次写GY3x模块寄存器
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/

void Single_Write_GY3x(uint8_t REG_Address)
{
    GY3x_I2cStartCondition();        //起始信号
    GY3x_SendByte(SlaveAddress);   //发送设备地址
    GY3x_SendByte(REG_Address);    //内部寄存器地址 
  //  GY3x_SendByte(REG_data);       //内部寄存器数据 
    GY3x_I2cStopCondition();       //停止信号
}

/**************************************************
 * 函数名称：uint8_t Single_Read_GY3x(uint8_t REG_Address)
 * 函数介绍：单次读GY3x模块寄存器
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/
uint8_t Single_Read_GY3x(uint8_t REG_Address)
{   
    uint8_t REG_data;
    GY3x_I2cStartCondition();        //起始信号                          
    GY3x_SendByte(SlaveAddress);   //发送设备地址           
    GY3x_SendByte(REG_Address);    //内部寄存器地址                   	
    GY3x_I2cStartCondition();        //起始信号                       
    GY3x_SendByte(SlaveAddress+1);         
    REG_data = GY3x_RecvByte();              
    GY3x_SendACK(1);   
    GY3x_I2cStopCondition();       //停止信号                          
    return REG_data; 
}

/**************************************************
 * 函数名称：void Multiple_Read_GY3x(void)
 * 函数介绍：连续读出GY3x模块数据
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/
void Multiple_Read_GY3x(void)
{   
    uint8_t i;
    GY3x_I2cStartCondition();        //起始信号                        
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

    GY3x_I2cStopCondition();       //停止信号 
    delay_ms(5);
}

/**************************************************
 * 函数名称：void GY3x_SendByte(uint8_t dat)
 * 函数介绍：向GY3X模块发送一个字节
 * 输入参数：
 * 输出参数：
 * 返回值  ：
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
        dat <<= 1;              //左移数据位
        GY3x_SCL_HIGH();     //
        delay_us(5);          //
        GY3x_SCL_LOW();          //
        delay_us(5);            //
    }
    GY3x_RecvACK();
}
/**************************************************
 * 函数名称：uint8_t GY3x_RecvByte(void)
 * 函数介绍：从GY3X模块接收一个字节
 * 输入参数：
 * 输出参数：
 * 返回值  ：
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
 * 函数名称：uint8_t GY3x_RecvACK()
 * 函数介绍：接收GY3X模块的应答
 * 输入参数：
 * 输出参数：
 * 返回值  ：
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
 * 函数名称：void GY3x_SendACK(uint8_t ack)
 * 函数介绍：向GY3X模块发送应答
 * 输入参数：ack (0:ACK 1:NAK)
 * 输出参数：
 * 返回值  ：
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
 * 函数名称：void GY3x_ReadLightdata(int ms)
 * 函数介绍：读取光照数据
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/
void GY3x_ReadLightdata(int ms)
{
    int   light_temp; 
    static  uint8_t gy3x_sample_cnt = 0; //gy3x采样次数
    static  uint16_t gy3x_sample_time = 0; //gy3x采样时间
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
          gy3x_sample_cnt++; //采样次数加1
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
             if(gy3x_sample_cnt > 3) //超过3次采样值小于零，光照值赋值零
             {
               g_gy3x_light = 0;
               gy3x_sample_cnt = 0;
               eGy3xsampleState = GY3X_SAMPLE_COMPLETE;
             }
             else
             {
               eGy3xsampleState = GY3X_SAMPLE_START; //小于3次重新采样
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
 * 名称: float Get_GY3x_Lightdata(void)
 * 功能: GY3X读取光照值取平均
 * 形参: 无
 * 返回: avg_light；平均光照值
 * 说明: 无 
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