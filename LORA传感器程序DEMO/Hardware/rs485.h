#ifndef __RS485_H
#define __RS485_H

/* ����ϵͳͷ�ļ� */
#include <stdio.h>
#include <stdarg.h>
#include "stm8l15x.h"
#include "stm8l15x_syscfg.h"
#include "stm8l15x_itc.h"
/* Private define ------------------------------------------------------------*/
#define RS485_PORT  GPIOC
#define RS485_EN    GPIO_Pin_4
#define RS485_TX    GPIO_Pin_3
#define RS485_RX    GPIO_Pin_2
#define RS485_BAUDRATE 19200

#ifdef _RAISONANCE_
#define PUTCHAR_PROTOTYPE int putchar (char c)
#define GETCHAR_PROTOTYPE int getchar (void)
#elif defined (_COSMIC_)
#define PUTCHAR_PROTOTYPE char putchar (char c)
#define GETCHAR_PROTOTYPE char getchar (void)
#else /* _IAR_ */
#define PUTCHAR_PROTOTYPE int putchar (int c)
#define GETCHAR_PROTOTYPE int getchar (void)
#endif /* _RAISONANCE_ */

extern u8 RS485_RX_BUF[64];  //���ջ���,���64���ֽ�.
extern u8 RS485_RX_CNT ;  //�����ֽڼ���
extern u8 RS485_RX_END ; //����������
extern u8 RS485_RX_LEN ; //���յ������ݳ���
extern char g_ID[4]; //���õ��豸ID

void rs485_init(void);
u8 UART1_ReceiveByte(void);
void UART1_SendByte(u8 data);
void UART1_SendStr(u8 *str);
void RS485_Send_Data(u8 *buf,u8 len);
u8 RS485_Receive_Data(u8 *data,u8 *buf,u8 len);
void Rs485_SendStr(u8 *str);
void rs485_communication(void);
#ifdef _IAR_
int fputc(int ch, FILE *f);
#endif
#endif

