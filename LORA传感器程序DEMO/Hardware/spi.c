/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * 文件名称：spi.c
 * 文件标识：
 * 摘    要：SPI驱动程序
 * 
 * 当前版本：V1.0.0
 * 作    者：yjd
 * 完成日期：2017-09-30
**************************************************/
#include "led.h"
#include "exti.h"
#include "stm8l15x.h"
#include "stdio.h"
#include "spi.h"

/*******************************************************************************
 * 名称: void SPI1_DeInit(void) 
 * 功能: 禁用SPI口
 * 形参: 无
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/  

void SPI1_DeInit(void)   //禁用SPI 并设置引脚为浮动无中断
{
  SPI_Cmd(SPI1, DISABLE); //禁用SPI

  CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, DISABLE); //关闭时钟

  GPIO_Init(SPI_SCK_GPIO_PORT, SPI_SCK_PIN, GPIO_Mode_In_FL_No_IT); //SCK pin

  GPIO_Init(SPI_MISO_GPIO_PORT, SPI_MISO_PIN, GPIO_Mode_In_FL_No_IT); //MOSIpin

  GPIO_Init(SPI_MOSI_GPIO_PORT, SPI_MOSI_PIN, GPIO_Mode_In_FL_No_IT); //MISO pin

  GPIO_Init(CS_GPIO_PORT, CS_PIN, GPIO_Mode_In_FL_No_IT); //CS pin
}

/*******************************************************************************
 * 名称: void SPI1_Init_Gpio(void)
 * 功能: SPI1引脚初始化
 * 形参: 无
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/  
void SPI1_Init_Gpio(void)
{
  CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, ENABLE);   //开启时钟

  GPIO_ExternalPullUpConfig(SPI_SCK_GPIO_PORT, SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN, ENABLE); //启用引脚内部上拉

  GPIO_Init(CS_GPIO_PORT, SPI_CS_PIN , GPIO_Mode_Out_PP_High_Slow);  //CS pin 设为输出
}

/*******************************************************************************
 * 名称: void SPI1_Init(void)    
 * 功能: SPI1初始化
 * 形参: 无
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/  
void SPI1_Init(void)    //SPI1初始化
{

  SPI1_Init_Gpio();

  SPI_CS_HIGH();  //拉高CS

  SPI_Init(SPI1, SPI_FirstBit_MSB, SPI_BaudRatePrescaler_4, SPI_Mode_Master,
           SPI_CPOL_High, SPI_CPHA_2Edge, SPI_Direction_2Lines_FullDuplex,
           SPI_NSS_Soft, 0x07); //SPI1 设置 先传高位,4分频,主模式,模式1,2线双工,软件控制

  SPI_Cmd(SPI1, ENABLE);  //开启spi
}


/*******************************************************************************
 * 名称: uint8_t SPI1_SendByte(uint8_t byte)   
 * 功能: SPI1发送字节
 * 形参: uint8_t byte 
 * 返回: 接收到的字节
 * 说明: 无 
 ******************************************************************************/  
uint8_t SPI1_SendByte(uint8_t byte) //发送字节  不是写一个字节 只是发送,没有CS脚的控制
{
  while (SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE) == RESET);  //DR寄存器为空

  SPI_SendData(SPI1, byte); //发送字节

  while (SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE) == RESET); //等待可读取

  return SPI_ReceiveData(SPI1); //返回RX数据
}
