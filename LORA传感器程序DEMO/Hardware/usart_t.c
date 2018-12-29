 /**************************************************
 * Copyright (c) 2013,���ݺ���������Ϣ�������޹�˾
 * All rights reserved.
 * 
 * �ļ����ƣ�USART.c
 * �ļ���ʶ��
 * ժ    Ҫ��USART����
 * 
 * ��ǰ�汾��1.0
 * ��    �ߣ�chaiyimin
 * ������ڣ�2013-11-01
 **************************************************/

//#include "USART.h"
#include "usart_t.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int m_iPort; // �˿ں�
// ��д��������ǵĴ�С
unsigned char* m_pWriteBuff;
unsigned int m_iWriteBuffSize;
unsigned char* m_pReadBuff;
unsigned int m_iReadBuffSize;
// ����ʱ(��λΪ�������ݽ���ʱ��)
unsigned int m_iTimeout;
unsigned int m_iBaud;

unsigned char DebugComRxbuffer[DebugComBufferSize];
unsigned char DebugComTxbuffer[DebugComBufferSize];
//Ϊÿ�����ڿ��ٵĻ�����
CCOMBUFFER g_CComBuffer[1];


void UARTInit(void)
{
    GPIO_Init(GPIOC, GPIO_Pin_3, GPIO_Mode_Out_PP_High_Fast);//TXD
    GPIO_Init(GPIOC, GPIO_Pin_2, GPIO_Mode_In_PU_No_IT);//RXD
	CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE);
    USART_DeInit(USART1);       //��λUART1 
//	USART_Init(USART1, 
//               (u32)9600,
//               USART_WordLength_8b, 
//               USART_StopBits_1,
//               USART_Parity_No, 
//               USART_Mode_Tx_And_Rx);
    Set_BaudRate(9600);
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	USART_ITConfig( USART1,  USART_IT_RXNE, ENABLE ); //���������ж�
//	USART_ITConfig( USART1,  USART_IT_TC, DISABLE ); //�رմ�������ж�
	USART_Cmd(USART1, ENABLE );	
}

/*
* �������ܣ� ����һ���ֽ�
* ��������� ��Ҫ���͵�����
* ����ֵ��   ��
*/
void UARTSendData(u8 data){
  
	USART_SendData8(USART1, data);
 	/* �ȴ�������� */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}


void UARTSendBuf(u8* Data, u16 len){
	
	for(u16 i = 0; i<len ;i++)
    {
		UARTSendData(Data[i]);
    }
}

/*
* �������ܣ� ����һ���ֽ�
* ��������� ��
* ����ֵ��   USART1_RX_BUF�����յ�����
*/
u8 UARTReceiveByte(void)
{
  
    u8 USART1_RX_BUF; 
    
    /* �ȴ�������� */
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
	
    USART1_RX_BUF = USART_ReceiveData8(USART1);
    
    return  USART1_RX_BUF;
    
}

/*
* �������ܣ� ���������ַ���
* ��������� ���
* ����ֵ��   ��
*/
void UARTSendString(char *format, ...)
{
//	char strBuf[64];
//	va_list ap;   //��ʼ��ָ��ɱ�����б��ָ��  
//	va_start(ap, format);  //����һ���ɱ�����ĵ�ַ����ap����apָ��ɱ�����б�Ŀ�ʼ
//	vsprintf(strBuf, format, ap); //������ap��format����ת���γɸ�ʽ���ַ�������������ʾ���ַ���
//	strBuf[63] = 0;
//	UARTSendBuf(strBuf, strlen(strBuf));
//	va_end(ap);
}


/*
 * ��������itoa
 * ����  ������������ת�����ַ���
 * ����  ��-radix =10 ��ʾ10���ƣ��������Ϊ0
 *         -value Ҫת����������
 *         -buf ת������ַ���
 *         -radix = 10
 * ���  ����
 * ����  ����
 * ����  ����USART1_printf()����
*/
//static char *itoa(int value, char *string, int radix)
//{
//    int     i, d;
//    int     flag = 0;
//    char    *ptr = string;
// 
//    /* This implementation only works for decimal numbers. */
//    if (radix != 10)
//    {
//        *ptr = 0;
//        return string;
//    }
// 
//    if (!value)
//    {
//        *ptr++ = 0x30;
//        *ptr = 0;
//        return string;
//    }
// 
//    /* if this is a negative value insert the minus sign. */
//    if (value < 0)
//    {
//        *ptr++ = '-';
//        /* Make the value positive. */
//        value *= -1;
//    }
// 
//    for (i = 10000; i > 0; i /= 10)
//    {
//        d = value / i;
// 
//        if (d || flag)
//        {
//            *ptr++ = (char)(d + 0x30);
//            value -= (d * i);
//            flag = 1;
//        }
//    }
// 
//    /* Null terminate the string. */
//    *ptr = 0;
// 
//    return string;
// 
//} /* NCL_Itoa */



/**
  * @brief  ����ͨ��������.
  * @param IN  COM_TypeDef COM	��   ͨ��ѡ��
  * @param IN  Cuint32_t USART_Baud	������Ҫ�Ĳ�����
  * @retval None
  */
    
void USART2_CONFIG(void)
{
//   //m_iPort = 2;
//   UART2_DeInit();
//   CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART2, ENABLE);//����2ʱ�ӿ�
//   UART_IOConfig();
//   Set_BaudRate((u32)9600);
////   UART2_Init((u32)57600, UART2_WORDLENGTH_8D,
////              UART2_STOPBITS_1, UART2_PARITY_NO,
////              UART2_SYNCMODE_CLOCK_DISABLE, UART2_MODE_RX_ENABLE);
//   UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);
//   UART2_Cmd(ENABLE);
//   
}

/**********************************************
*��������void USARTx_Init(u8 channel)
*����  ����IOд��
*���  ��u8 channel
*����  �� 
*��ע  ����
***********************************************/
void USARTx_Init(u8 channel)
{
  if(channel == 1)
  {
    
  }
  else if(channel == 2)
  {
    CdyUARTRAM();
    CdyUARTSetBuff(DebugComTxbuffer, sizeof(DebugComTxbuffer), DebugComRxbuffer, sizeof(DebugComRxbuffer));
    //USART2_CONFIG();

    UARTInit();
  }
  else if(channel == 3)
  {
    
  }
  else
  {
    return;
  }
}

/***********************************
�������ܣ�UART IO�ڳ�ʼ��
�����������
�����������
��   ע���ɣ������ģʽ�£���ͨ���ãң��Ĵ���
         �����������
***********************************/
void UART_IOConfig(void)
{
    TXPort->DDR |= TXPin;//���ģʽ
    TXPort->CR1 |= TXPin;//�������   
    
    RXPort->DDR &=~RXPin;//����ģʽ
    RXPort->CR1 &=~RXPin;//��������
}

/***********************************************
��������: �ض���fputc����
��   ע��ʹ��printf���ض���fputc����������
        �޸���General Options �е�Library Configuration
        ��Library Options
***********************************************/
//int fputc(int ch, FILE *f)
//{
//    while((UART2->SR&0X40)==0);   
//    UART2->DR = (u8) ch;   
//    
//    return ch;
//}

void CdyUARTRAM(void)
{
  m_iPort = 0;
  m_pWriteBuff = 0;
  m_iWriteBuffSize = 0;
  m_pReadBuff = 0;
  m_iReadBuffSize = 0;
  m_iTimeout = 100;
}

// ���û���
void CdyUARTSetBuff(void* pWriteBuff, unsigned int iWriteBuffSize, void* pReadBuff, unsigned int iReadBuffSize)
{
   // CdyStream::SetBuff(pWriteBuff,iWriteBuffSize,pReadBuff,iReadBuffSize);
    PCCOMBUFFER pCComBuffer;
   // if((m_iPort<0)||(m_iPort>4)) return;//�˿ںŲ��ڷ�Χ�ڣ�����
    pCComBuffer = &g_CComBuffer[0];
    
    m_pWriteBuff = (unsigned char*)pWriteBuff;
    m_iWriteBuffSize = iWriteBuffSize;
    m_pReadBuff = (unsigned char*)pReadBuff;
    m_iReadBuffSize = iReadBuffSize;
    
    pCComBuffer->pTxdBuff = m_pWriteBuff;//���÷��ͻ�����
    pCComBuffer->pRxdBuff = m_pReadBuff; //���÷����ջ�����
    pCComBuffer->TxdBuffSize = m_iWriteBuffSize;//���÷��ͻ���������
    pCComBuffer->RxdBuffSize = m_iReadBuffSize; //���ý��ջ���������
}


/**********************************************
*��������void Set_BaudRate(uint32_t BaudRate)
*����  :���ڲ���������
*���  ��uint32_t BaudRate
*����  �� 
*��ע  ����
***********************************************/
void Set_BaudRate(uint32_t BaudRate)
{
  PCCOMBUFFER pCComBuffer;
  pCComBuffer = &g_CComBuffer[0];
  m_iBaud =  BaudRate;
  pCComBuffer->RxdDelayCntMax = m_iTimeout*10000/m_iBaud;
//  CdyUARTSetReadTimeout(100);
  if(pCComBuffer->RxdDelayCntMax < 5)
    pCComBuffer->RxdDelayCntMax = 5; //���������Ϊ0����ֵ5ms
  if(pCComBuffer->RxdDelayCntMax > 500) pCComBuffer->RxdDelayCntMax = 500; //������㳬��Ϊ500����ֵ500ms
  
  pCComBuffer->pTxdBuff = m_pWriteBuff;//���÷��ͻ�����
  pCComBuffer->pRxdBuff = m_pReadBuff; //���÷����ջ�����
  pCComBuffer->TxdBuffSize = m_iWriteBuffSize;//���÷��ͻ���������
  pCComBuffer->RxdBuffSize = m_iReadBuffSize; //���ý��ջ���������
  
  pCComBuffer->RxdByteCnt = 0;//����ָ�븴λ
  pCComBuffer->pTxHead = 0;//����ͷָ�븴λ
  pCComBuffer->pTxTail = 0;//����βָ�븴λ
  
  USART_Init(USART1, 
             (u32)BaudRate, 
             USART_WordLength_8b, 
             USART_StopBits_1, 
             USART_Parity_No,
             USART_Mode_Tx_And_Rx);//USART_Mode_Rx|USART_Mode_Tx
//  UART2_Init((u32)BaudRate,
//             UART2_WORDLENGTH_8D,
//             UART2_STOPBITS_1,
//             UART2_PARITY_NO,
//             UART2_SYNCMODE_CLOCK_DISABLE,
//             UART2_MODE_TXRX_ENABLE);
}


void CdyUARTSetReadTimeout(unsigned int iTimeout)
{
//    if((m_iPort<0)||(m_iPort>4)) return;//�˿ںŲ��ڷ�Χ�ڣ�����
    m_iTimeout = iTimeout;
    PCCOMBUFFER pCComBuffer;
    pCComBuffer = &g_CComBuffer[0];
    pCComBuffer->RxdDelayCntMax = m_iTimeout*10000/m_iBaud;
}

void CdyUARTWrite(void *pData, unsigned int iLen)
{
  //  USART_TypeDef*  USARTNo;
    unsigned int WriteBufferFreeSize = 0;
   // if((m_iPort<0)||(m_iPort>4)) return;//�˿ںŲ��ڷ�Χ�ڣ�����
    if((iLen == 0)||(pData == NULL))   return; //�������Ϊ0��������Ϊ����ȡ������
  //  USARTNo = COM_USART_CHANNEL[m_iPort];    //���Ҵ���
    //����ʣ���д�ռ�ĳ��� WriteBufferFreeSize
    PCCOMBUFFER pCComBuffer;
    pCComBuffer = &g_CComBuffer[0];

   // USART_ITConfig(USARTNo, USART_IT_TXE, DISABLE);     //��ֹ�����ж�
    USART_ITConfig(USART1,USART_IT_TXE, DISABLE); //��ֹ�����ж�
    if(pCComBuffer->pTxdBuff == NULL) return;

    WriteBufferFreeSize = pCComBuffer->TxdBuffSize -  (pCComBuffer->pTxTail - pCComBuffer->pTxHead);
    if(iLen > WriteBufferFreeSize) iLen = WriteBufferFreeSize;//������͵ĳ��ȴ��ڻ�������ȡ�������ĳ���

    for(unsigned int i = 0;i < iLen; i++)
    {
        *(m_pWriteBuff + (pCComBuffer->pTxTail%pCComBuffer->TxdBuffSize)) =  *((unsigned char*)pData + i);
        pCComBuffer->pTxTail++;
    }
    pCComBuffer->CommStatus.COMM_SENDEND = 0; //���㷢�����־
    pCComBuffer->CommStatus.COMM_SENDING = 1; //�������ڷ���־λ
    
//    UART2_ITConfig(UART2_IT_TXE, ENABLE);     //�������ͷ���
//    UART2_ITConfig(UART2_IT_TC, ENABLE);     
    USART_ITConfig( USART1,  USART_IT_RXNE, ENABLE ); //���������ж�
	USART_ITConfig( USART1,  USART_IT_TC, DISABLE ); //�رմ�������ж�
//    USART_ITConfig(USARTNo, USART_IT_TXE, ENABLE);     //�������ͷ���
//    USART_ITConfig(USARTNo, USART_IT_TC, ENABLE);
}


unsigned int CdyUARTRead(void *pData,unsigned int iLen)
{
    unsigned int t;
    //USART_TypeDef*  USARTNo;

    //if((m_iPort<0)||(m_iPort>4)) return 0;  //�˿ںŲ��ڷ�Χ�ڣ�����
   // USARTNo = COM_USART_CHANNEL[m_iPort];   //��ѯ����

    PCCOMBUFFER pCComBuffer;
    pCComBuffer = &g_CComBuffer[0];

    if(pCComBuffer->pRxdBuff == NULL)  return 0;

    if(pCComBuffer->RxdDelayCnt > 0)   return  0;
      //if( (pCComBuffer->CommStatus.COMM_RECVING == 1) || (pCComBuffer->CommStatus.COMM_RECVEND == 0) || ( pCComBuffer->RxdByteCnt == 0))  //����δ��ɻ��߽�������Ϊ0��ֱ���˳�
    if( pCComBuffer->RxdByteCnt == 0)  //����δ��ɻ��߽�������Ϊ0��ֱ���˳�
    {
      t = 0;
    }
    else
    {
      //USART_ITConfig(USARTNo, USART_IT_RXNE, DISABLE);  //�Ƚ�ֹ����
//      UART2_ITConfig(UART2_IT_RXNE, DISABLE);    //�Ƚ�ֹ����
      USART_ITConfig( USART1,  USART_IT_RXNE, DISABLE ); //�Ƚ�ֹ����
      t = pCComBuffer->RxdByteCnt;
      if(t > iLen)   t = iLen;                          //����Ҫ��ȡ�ĳ���
      m_pReadBuff = pCComBuffer->pRxdBuff;              //��ȡ���ջ�����ָ��
      memcpy(pData,m_pReadBuff,t);                      //��ȡ���յ�����
      pCComBuffer->RxdByteCnt = 0;                      //�����ָ��
      pCComBuffer->CommStatus.COMM_RECVEND = 0;         //���������ɱ�־
      // USART_ITConfig(USARTNo, USART_IT_RXNE, ENABLE); //�򿪽����ж�
//      UART2_ITConfig(UART2_IT_RXNE, ENABLE); 
      USART_ITConfig( USART1,  USART_IT_RXNE, ENABLE ); //�򿪽����ж�
    }
    return t;
}


void CdyUARTRun(unsigned int ms)   //ms�Ǵ��ݽ�����ms��
{
    PCCOMBUFFER pCComBuffer;
 //   if((m_iPort<0)||(m_iPort>4)) return;//�˿ںŲ��ڷ�Χ�ڣ�����
    pCComBuffer = &g_CComBuffer[0];

    if( pCComBuffer->RxdDelayCnt > 0)   //���ճ�ʱ�ж�
    {
        pCComBuffer->RxdDelayCnt -= ms ;
        if( pCComBuffer->RxdDelayCnt <= 0)
        {
            pCComBuffer->CommStatus.COMM_RECVING = 0;			 	
            pCComBuffer->CommStatus.COMM_RECVEND = 1;
            pCComBuffer->RxdDelayCnt = 0;
        }
    }
    
    if( (pCComBuffer->CommStatus.COMM_SENDING != 1 ))
    {
      pCComBuffer->CommStatus.COMM_IDLE = 1;
    }
    else
    {
      pCComBuffer->CommStatus.COMM_IDLE = 0;
    }
}


//���ڹ���״̬
bool CdyUARTIsBusy(void)
{
    PCCOMBUFFER pCComBuffer;
   // if((m_iPort<0)||(m_iPort>4)) return false;//�˿ںŲ��ڷ�Χ�ڣ�����
    pCComBuffer = &g_CComBuffer[0];

    if(pCComBuffer->CommStatus.COMM_IDLE == 1)
        return FALSE;
    else
        return TRUE;
}


//���ں�����ͨ�ô�����
void Usartx_IrqRXHandler(void)
{
  unsigned char RxData;
  PCCOMBUFFER  pCComBuffer;
  pCComBuffer = &g_CComBuffer[0];
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    if(pCComBuffer->pRxdBuff == NULL) return;              //������ջ�����ָ��Ϊ�գ�ֱ�ӷ���
    /* Read one byte from the receive data register */
    RxData = USART_ReceiveData8(USART1);                     //����һ���ֽ�����
    if(pCComBuffer->RxdByteCnt < pCComBuffer->RxdBuffSize) //С�ڻ���������//�������ٽ���
    {
        pCComBuffer->pRxdBuff[pCComBuffer->RxdByteCnt] =  RxData;
        pCComBuffer->CommStatus.COMM_RECVEND = 0;
        pCComBuffer->CommStatus.COMM_RECVING = 1;
        
        pCComBuffer->RxdDelayCnt = pCComBuffer->RxdDelayCntMax; //�������ʱ����ʱ
        pCComBuffer->RxdByteCnt++;
      }
  }
//  USART_ClearITPendingBit(USART1, USART_IT_TC);

}

void Usartx_IrqTXHandler(void)
{
  unsigned char c;
  PCCOMBUFFER  pCComBuffer;
  pCComBuffer = &g_CComBuffer[0];
  //CdyUARTRun(200);
  //pCComBuffer->RxdDelayCnt = (u16)pCComBuffer->RxdDelayCntMax;    //300-400ms
  if(USART_GetITStatus(USART1,USART_IT_TXE) != RESET)  //USART_GetITStatus(USART1, USART_IT_IDLE)
  {   	  	
    if(pCComBuffer->pTxdBuff == NULL) return;//������ͻ�����ָ��Ϊ�գ�ֱ�ӷ���
    if(pCComBuffer->pTxHead >= pCComBuffer->pTxTail)
    {			
        USART_ITConfig( USART1,  USART_IT_TXE, DISABLE ); //�Ƚ�ֹ����
//      UART2_ITConfig(UART2_IT_TXE, DISABLE);
      pCComBuffer->pTxHead = 0;
      pCComBuffer->pTxTail = 0;	
    }
    else
    {
      /* Write one byte to the transmit data register */
      c =  *( pCComBuffer->pTxdBuff + pCComBuffer->pTxHead%pCComBuffer->TxdBuffSize );
      pCComBuffer->pTxHead ++;
      USART_SendData8(USART1, c);
      pCComBuffer->CommStatus.COMM_SENDING = 1;			 	
      pCComBuffer->CommStatus.COMM_SENDEND = 0;	
    }
  }    
    //���һ���ֽڵ�����bit������
  if(USART_GetITStatus(USART1,USART_IT_TC) != RESET)
  {  
    USART_ITConfig(USART1,USART_IT_TC, DISABLE);
    pCComBuffer->CommStatus.COMM_SENDING = 0;			 	
    pCComBuffer->CommStatus.COMM_SENDEND = 1;
    
  }
}

void DebugCom_test(void)
{
  //���ý��շ��ͻ���ָ��ʹ�С
 // USARTx_Init(2);
  static unsigned char DebugComRxBuffer[10];
  static unsigned int Len;
  CdyUARTWrite((void*)"The car left a trail of exhaust fumes. Two from eight leaves six.\r\n",strlen("The car left a trail of exhaust fumes. Two from eight leaves six.\r\n"));
  //unsigned int t1 = CRTC_DateTime.GetMSec();	
  while (1)
  {
    memset(DebugComRxBuffer,0,sizeof(DebugComRxBuffer));	
    
    if((Len = CdyUARTRead(DebugComRxBuffer,10)) > 0)
    {
      CdyUARTWrite((void*)"\r\nThe Receive Data:\r\n",strlen("\r\nThe Receive Data:\r\n"));
      CdyUARTWrite(DebugComRxBuffer,Len);
    }	
  }
}

