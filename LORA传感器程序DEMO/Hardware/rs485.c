/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * �ļ����ƣ�rs485.c
 * �ļ���ʶ��
 * ժ    Ҫ��485ͨѶ
 * 
 * ��ǰ�汾��V1.0.0
 * ��    �ߣ�yjd
 * ������ڣ�2017-03-23
**************************************************/

#include "rs485.h"
#include "led.h"
#include "string.h"  
#include "flash.h" 
#include "apply.h" 

	
u8 RS485_RX_BUF[64];  //���ջ���,���64���ֽ�.

u8 RS485_RX_CNT = 0;  //�����ֽڼ���

u8 RS485_RX_END = 0; //����������

u8 RS485_RX_LEN = 0;//���յ������ݳ���

char g_ID[4] = {0}; //���õ��豸ID

/*******************************************************************************
 * ����: void rs485_init(void)
 * ����: RS485��ʼ��
 * �β�: ��
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/
void rs485_init(void)
{
  CLK_PeripheralClockConfig(CLK_Peripheral_USART1,ENABLE);

  //  SYSCFG_REMAPDeInit();
  // SYSCFG_REMAPPinConfig(REMAP_Pin_USART1TxRxPortC,ENABLE);
  GPIO_Init(RS485_PORT, RS485_EN, GPIO_Mode_Out_PP_High_Fast);//EN
  GPIO_Init(RS485_PORT, RS485_TX, GPIO_Mode_Out_PP_High_Fast);//TXD
  GPIO_Init(RS485_PORT, RS485_RX, GPIO_Mode_In_PU_No_IT);//RXD
  USART_DeInit(USART1);		//��λUSART1 
  USART_Init(USART1,(u32)RS485_BAUDRATE, USART_WordLength_8b, USART_StopBits_1,
                            USART_Parity_No, USART_Mode_Tx_And_Rx);
  USART_ClearITPendingBit(USART1, USART_IT_RXNE);//��������жϱ�־λ
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//���������ж�
  //USART_ITConfig(USART1, USART_IT_TC, ENABLE);//���������ж�
  ITC_SetSoftwarePriority(USART1_RX_IRQn, ITC_PriorityLevel_2);//�������ȼ�
  USART_Cmd(USART1, ENABLE);	//ʹ��USART1
  GPIO_ResetBits(RS485_PORT, RS485_EN);//ʹ������
}

/*******************************************************************************
 * ����: INTERRUPT_HANDLER(USART1_RX_TIM5_CC_IRQHandler,28)
 * ����: USART1�жϺ���
 * �β�: USART1_RX_TIM5_CC_IRQHandler
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/
INTERRUPT_HANDLER(USART1_RX_TIM5_CC_IRQHandler,28)
{
  u8 res;	    
  if(USART_GetITStatus(USART1, USART_IT_RXNE)) //���յ�����
  {	                     
    res = UART1_ReceiveByte(); 	//��ȡ���յ�������
    if(RS485_RX_CNT < sizeof(RS485_RX_BUF))
    {
      RS485_RX_BUF[RS485_RX_CNT] = res;		//��¼���յ���ֵ
      RS485_RX_CNT++;		//������������1 
    } 
  }  											 
}

/*******************************************************************************
 * ����: void RS485_Send_Data(u8 *buf,u8 len)
 * ����: RS485�����ֽ�
 * �β�: buf-> Ҫ���͵��ֽ��׵�ַ len-> ���͵��ֽڳ���
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/

void RS485_Send_Data(u8 *buf,u8 len)
{
  if(buf == NULL)
  {
    return;
  }
  u8 t;
  u8 *p = buf;
  GPIO_SetBits(RS485_PORT, RS485_EN);	//����Ϊ����ģʽ
  for(t = 0;t < len;t++)		//ѭ����������
  {		   
    while( RESET == USART_GetFlagStatus(USART1, USART_FLAG_TC));	  
    USART_SendData8(USART1,p[t]);
  }	 

  while(RESET == USART_GetFlagStatus(USART1, USART_FLAG_TC));		
  RS485_RX_CNT = 0;	  
  GPIO_ResetBits(RS485_PORT, RS485_EN); //����Ϊ����ģʽ	
}

/*******************************************************************************
 * ����: u8 RS485_Receive_Data(u8 *data,u8 *buf,u8 len)
 * ����: RS485��������
 * �β�: u8 *data ��ȡ����  u8 *buf �������ݻ���  u8 len �������ݳ���
 * ����: �������ֽڳ���
 * ˵��: �� 
 ******************************************************************************/

u8 RS485_Receive_Data(u8 *data,u8 *buf,u8 len)
{
  if((data == NULL) || (buf == NULL) )
  {
    return 0;
  }
  u8 rxlen = len;
  u8 i = 0; 
  u8 *bufp = buf;
  u8 *datap = data;
  for(i = 0;i < rxlen;i++)
  {
    datap[i]=bufp[i];	
  }		
  len = 0; //����
  memset(bufp,0,sizeof(bufp));
  return rxlen;
}
/*******************************************************************************
 * ����: void Rs485_SendStr(u8 *str)
 * ����: �����ַ���
 * �β�: str
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/
void Rs485_SendStr(u8 *str)
{
  GPIO_SetBits(RS485_PORT, RS485_EN);	
  while(*str != '\0')
  {
    UART1_SendByte(*str++);	/* ѭ�����÷���һ���ַ����� */
  }
  GPIO_ResetBits(RS485_PORT, RS485_EN); 	
}


/**************************************************
 * �������ƣ�void rs485_communication(void)
 * �������ܣ�RS485ͨѶ
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
void rs485_communication(void)
{
  u8 i = 0;
  u8 j = 0;
  u8 setlenth = 0; //���õ��ַ�������
  char* strp = NULL; //��=�������ַ�����ַ
  u8 strlenth = 0; //��=�������ַ�������
  u8 rs485_rx_buf[64]; 
  char rs485_send_str[64];
  if(RS485_RX_END == TRUE)//��ʱ����
  {
    RS485_RX_END = FALSE;
    memset(rs485_rx_buf,0,sizeof(rs485_rx_buf));
    RS485_Receive_Data(rs485_rx_buf,RS485_RX_BUF,RS485_RX_LEN );
//    if(strcmp((char*)rs485_rx_buf,devID_str) == ZERO)
//    {  
//     memset(rs485_send_str,ZERO,sizeof(rs485_send_str));
//     sprintf(rs485_send_str,"DeviceID:%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X,Distance:%.2fm\r\n",\
//          chipID[0],chipID[1],chipID[2],chipID[3],chipID[4],chipID[5],chipID[6],chipID[7],\
//          chipID[8],chipID[9],chipID[10],chipID[11],g_distance/100);
//     Rs485_SendStr((u8*)rs485_send_str);
//     
//    }
    if(strstr((char*)rs485_rx_buf,"SET+ID="))
    {
      memset(rs485_send_str,0,sizeof(rs485_send_str));
      strp = strstr((char*)rs485_rx_buf,"SET+ID=");
      setlenth = strlen("SET+ID=");
      strlenth = strlen((strp+setlenth));
      if((*(strp+setlenth) == '?') && (strlenth == 1))
      {
        for(i = 0; i < 4; i++)
        {
          g_ID[i] = FLASH_ReadByte(BASE_ADDR+i);     
        }
//        sprintf(rs485_send_str,"DeviceID:%c%c%c%c,Distance:%.2fm\r\n",g_ID[0],g_ID[1],g_ID[2],g_ID[3],g_distance/100);
        Rs485_SendStr((u8*)rs485_send_str);
      }
      else if(strlenth == 4) //�ĸ�ID�ַ�
            {
              for(j = 0; j < 4; j++)
              {
                FLASH_EraseByte(BASE_ADDR+j);         //�ٲ���
                FLASH_ProgramByte(BASE_ADDR+j,*(strp+j+setlenth));
              }
                Rs485_SendStr("OK\r\n");
            }
           else
              { 
                Rs485_SendStr("EEROR\r\n");
              }
      
      }

  }

}

/*******************************************************************************
 * ����: UART1_SendByte
 * ����: UART1����һ���ֽ�
 * �β�: data -> Ҫ���͵��ֽ�
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/
void UART1_SendByte(u8 data)
{
  USART_SendData8(USART1, data);
  /* �ȴ�������� */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

/*******************************************************************************
 * ����: UART1_SendStr
 * ����: UART1����len���ַ�
 * �β�: data -> ָ��Ҫ���͵��ַ���
 *       len -> Ҫ���͵��ֽ���
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/
void UART1_SendStr(u8 *str)
{
  while(*str != '\0')
  {
    UART1_SendByte(*str++);	/* ѭ�����÷���һ���ַ����� */
  }	
}

/*******************************************************************************
 * ����: UART1_ReceiveByte
 * ����: UART1����һ���ַ�
 * �β�: ��
 * ����: ���յ����ַ�
 * ˵��: �� 
 ******************************************************************************/
u8 UART1_ReceiveByte(void)
{
  u8 UART1_RX_BUF; 
  /* �ȴ�������� */
  while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
  UART1_RX_BUF = USART_ReceiveData8(USART1);
  return  UART1_RX_BUF;
}

/*******************************************************************************
 * ����: fputc
 * ����: �ض���c�⺯��printf��UART1
 * �β�: ��
 * ����: Ҫ��ӡ���ַ�
 * ˵��: ��printf���� 
 ******************************************************************************/
#ifdef _IAR_
int fputc(int ch, FILE *f)
{  
  /* ��Printf���ݷ������� */
  UART1_SendByte(ch);
  return (ch);
}
#else
PUTCHAR_PROTOTYPE
{
/* Write a character to the UART1 */
  UART1_SendByte(c);
  return (c);
}
#endif

GETCHAR_PROTOTYPE
{
#ifdef _COSMIC_
  char c = 0;
#else
  int c = 0;
#endif
  /* Loop until the Read data register flag is SET */
  while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
  c = USART_ReceiveData8(USART1);
  return (c);
}

