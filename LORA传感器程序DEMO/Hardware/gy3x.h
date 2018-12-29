#ifndef __GY3X_H
#define __GY3X_H
#include "stm8l15x.h"
#include "gy3x.h"


typedef  enum
{
  GY3X_SAMPLE_START = 0,//光照度采集开始
  GY3X_SAMPLE_READ,//光照度采集读取
  GY3X_SAMPLE_COMPLETE //光照度采集完成
}EGy3xsampleState ;

extern EGy3xsampleState  eGy3xsampleState ;
extern uint8_t gy3x_data[3];
extern float g_gy3x_light; //gy3x 光照值

#define  GY3X_SDA_PORT   GPIOD
#define  GY3X_SDA_PIN    GPIO_Pin_6
#define  GY3X_SCL_PORT   GPIOD
#define  GY3X_SCL_PIN    GPIO_Pin_7

#define  GY3x_SCL_OUTPUT()  GPIO_Init(GY3X_SCL_PORT,GY3X_SCL_PIN, GPIO_Mode_Out_PP_Low_Fast)
#define  GY3x_SDA_OUTPUT()  GPIO_Init(GY3X_SDA_PORT,GY3X_SDA_PIN, GPIO_Mode_Out_PP_Low_Fast)
#define  GY3x_SCL_INPUT()   GPIO_Init(GY3X_SCL_PORT,GY3X_SCL_PIN, GPIO_Mode_In_FL_No_IT)
#define  GY3x_SDA_INPUT()   GPIO_Init(GY3X_SDA_PORT,GY3X_SDA_PIN, GPIO_Mode_In_FL_No_IT)
#define  GY3x_SCL_HIGH()    GPIO_SetBits(GY3X_SCL_PORT,GY3X_SCL_PIN)
#define  GY3x_SCL_LOW()     GPIO_ResetBits(GY3X_SCL_PORT,GY3X_SCL_PIN)
#define  GY3x_SDA_HIGH()    GPIO_SetBits(GY3X_SDA_PORT,GY3X_SDA_PIN)
#define  GY3x_SDA_LOW()     GPIO_ResetBits(GY3X_SDA_PORT,GY3X_SDA_PIN)
#define  GY3x_SCL_STATE()   GPIO_ReadInputDataBit(GY3X_SCL_PORT, GY3X_SCL_PIN)
#define  GY3x_SDA_STATE()   GPIO_ReadInputDataBit(GY3X_SDA_PORT, GY3X_SDA_PIN)

#define	  SlaveAddress   0x46 //定义器件的从地址
                              //ALT ADDRESS引脚接地时地址为0x46,接电源时地址为0x3A

void GY3x_Init(void);
void GY3x_I2cStartCondition(void);
void GY3x_I2cStopCondition(void);
void Single_Write_GY3x(uint8_t REG_Address);
uint8_t Single_Read_GY3x(uint8_t REG_Address);
void Multiple_Read_GY3x(void);
void GY3x_SendByte(uint8_t dat);
uint8_t GY3x_RecvByte(void);
uint8_t GY3x_RecvACK(void);
void GY3x_SendACK(uint8_t ack);
void GY3x_ReadLightdata(int ms);
float Get_GY3x_Lightdata(void);

#endif