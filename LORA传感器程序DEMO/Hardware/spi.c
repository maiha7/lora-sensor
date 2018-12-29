/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * �ļ����ƣ�spi.c
 * �ļ���ʶ��
 * ժ    Ҫ��SPI��������
 * 
 * ��ǰ�汾��V1.0.0
 * ��    �ߣ�yjd
 * ������ڣ�2017-09-30
**************************************************/
#include "led.h"
#include "exti.h"
#include "stm8l15x.h"
#include "stdio.h"
#include "spi.h"

/*******************************************************************************
 * ����: void SPI1_DeInit(void) 
 * ����: ����SPI��
 * �β�: ��
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/  

void SPI1_DeInit(void)   //����SPI ����������Ϊ�������ж�
{
  SPI_Cmd(SPI1, DISABLE); //����SPI

  CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, DISABLE); //�ر�ʱ��

  GPIO_Init(SPI_SCK_GPIO_PORT, SPI_SCK_PIN, GPIO_Mode_In_FL_No_IT); //SCK pin

  GPIO_Init(SPI_MISO_GPIO_PORT, SPI_MISO_PIN, GPIO_Mode_In_FL_No_IT); //MOSIpin

  GPIO_Init(SPI_MOSI_GPIO_PORT, SPI_MOSI_PIN, GPIO_Mode_In_FL_No_IT); //MISO pin

  GPIO_Init(CS_GPIO_PORT, CS_PIN, GPIO_Mode_In_FL_No_IT); //CS pin
}

/*******************************************************************************
 * ����: void SPI1_Init_Gpio(void)
 * ����: SPI1���ų�ʼ��
 * �β�: ��
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/  
void SPI1_Init_Gpio(void)
{
  CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, ENABLE);   //����ʱ��

  GPIO_ExternalPullUpConfig(SPI_SCK_GPIO_PORT, SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN, ENABLE); //���������ڲ�����

  GPIO_Init(CS_GPIO_PORT, SPI_CS_PIN , GPIO_Mode_Out_PP_High_Slow);  //CS pin ��Ϊ���
}

/*******************************************************************************
 * ����: void SPI1_Init(void)    
 * ����: SPI1��ʼ��
 * �β�: ��
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/  
void SPI1_Init(void)    //SPI1��ʼ��
{

  SPI1_Init_Gpio();

  SPI_CS_HIGH();  //����CS

  SPI_Init(SPI1, SPI_FirstBit_MSB, SPI_BaudRatePrescaler_4, SPI_Mode_Master,
           SPI_CPOL_High, SPI_CPHA_2Edge, SPI_Direction_2Lines_FullDuplex,
           SPI_NSS_Soft, 0x07); //SPI1 ���� �ȴ���λ,4��Ƶ,��ģʽ,ģʽ1,2��˫��,�������

  SPI_Cmd(SPI1, ENABLE);  //����spi
}


/*******************************************************************************
 * ����: uint8_t SPI1_SendByte(uint8_t byte)   
 * ����: SPI1�����ֽ�
 * �β�: uint8_t byte 
 * ����: ���յ����ֽ�
 * ˵��: �� 
 ******************************************************************************/  
uint8_t SPI1_SendByte(uint8_t byte) //�����ֽ�  ����дһ���ֽ� ֻ�Ƿ���,û��CS�ŵĿ���
{
  while (SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE) == RESET);  //DR�Ĵ���Ϊ��

  SPI_SendData(SPI1, byte); //�����ֽ�

  while (SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE) == RESET); //�ȴ��ɶ�ȡ

  return SPI_ReceiveData(SPI1); //����RX����
}
