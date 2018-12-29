 /**************************************************
 * Copyright (c) 2013,杭州海康雷鸟信息技术有限公司
 * All rights reserved.
 * 
 * 文件名称：USART.c
 * 文件标识：
 * 摘    要：USART函数
 * 
 * 当前版本：1.0
 * 作    者：chaiyimin
 * 完成日期：2013-11-01
 **************************************************/

//#include "USART.h"
#include "usart_t.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int m_iPort; // 端口号
// 读写缓存和他们的大小
unsigned char* m_pWriteBuff;
unsigned int m_iWriteBuffSize;
unsigned char* m_pReadBuff;
unsigned int m_iReadBuffSize;
// 读超时(单位为单个数据接收时间)
unsigned int m_iTimeout;
unsigned int m_iBaud;

unsigned char DebugComRxbuffer[DebugComBufferSize];
unsigned char DebugComTxbuffer[DebugComBufferSize];
//为每个串口开辟的缓冲区
CCOMBUFFER g_CComBuffer[1];


void UARTInit(void)
{
    GPIO_Init(GPIOC, GPIO_Pin_3, GPIO_Mode_Out_PP_High_Fast);//TXD
    GPIO_Init(GPIOC, GPIO_Pin_2, GPIO_Mode_In_PU_No_IT);//RXD
	CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE);
    USART_DeInit(USART1);       //复位UART1 
//	USART_Init(USART1, 
//               (u32)9600,
//               USART_WordLength_8b, 
//               USART_StopBits_1,
//               USART_Parity_No, 
//               USART_Mode_Tx_And_Rx);
    Set_BaudRate(9600);
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	USART_ITConfig( USART1,  USART_IT_RXNE, ENABLE ); //开启接收中断
//	USART_ITConfig( USART1,  USART_IT_TC, DISABLE ); //关闭传输完成中断
	USART_Cmd(USART1, ENABLE );	
}

/*
* 函数介绍： 发送一个字节
* 输入参数： 需要发送的数据
* 返回值：   无
*/
void UARTSendData(u8 data){
  
	USART_SendData8(USART1, data);
 	/* 等待传输结束 */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}


void UARTSendBuf(u8* Data, u16 len){
	
	for(u16 i = 0; i<len ;i++)
    {
		UARTSendData(Data[i]);
    }
}

/*
* 函数介绍： 接收一个字节
* 输入参数： 无
* 返回值：   USART1_RX_BUF，接收的数据
*/
u8 UARTReceiveByte(void)
{
  
    u8 USART1_RX_BUF; 
    
    /* 等待接收完成 */
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
	
    USART1_RX_BUF = USART_ReceiveData8(USART1);
    
    return  USART1_RX_BUF;
    
}

/*
* 函数介绍： 发送任意字符串
* 输入参数： 变参
* 返回值：   无
*/
void UARTSendString(char *format, ...)
{
//	char strBuf[64];
//	va_list ap;   //初始化指向可变参数列表的指针  
//	va_start(ap, format);  //将第一个可变参数的地址赋给ap，即ap指向可变参数列表的开始
//	vsprintf(strBuf, format, ap); //将参数ap和format进行转化形成格式化字符串，即可以显示的字符串
//	strBuf[63] = 0;
//	UARTSendBuf(strBuf, strlen(strBuf));
//	va_end(ap);
}


/*
 * 函数名：itoa
 * 描述  ：将整形数据转换成字符串
 * 输入  ：-radix =10 表示10进制，其他结果为0
 *         -value 要转换的整形数
 *         -buf 转换后的字符串
 *         -radix = 10
 * 输出  ：无
 * 返回  ：无
 * 调用  ：被USART1_printf()调用
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
  * @brief  串口通道的设置.
  * @param IN  COM_TypeDef COM	：   通道选择
  * @param IN  Cuint32_t USART_Baud	：所需要的波特率
  * @retval None
  */
    
void USART2_CONFIG(void)
{
//   //m_iPort = 2;
//   UART2_DeInit();
//   CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART2, ENABLE);//串口2时钟开
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
*程序名：void USARTx_Init(u8 channel)
*功能  ：对IO写高
*入口  ：u8 channel
*出口  ： 
*备注  ：无
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
函数功能：UART IO口初始化
输入参数：无
输出参数：无
备   注：ＩＯ在输出模式下，可通过ＣＲ２寄存器
         控制输出速率
***********************************/
void UART_IOConfig(void)
{
    TXPort->DDR |= TXPin;//输出模式
    TXPort->CR1 |= TXPin;//推挽输出   
    
    RXPort->DDR &=~RXPin;//输入模式
    RXPort->CR1 &=~RXPin;//浮空输入
}

/***********************************************
函数功能: 重定义fputc函数
备   注：使用printf需重定义fputc函数，并且
        修改在General Options 中的Library Configuration
        和Library Options
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

// 设置缓存
void CdyUARTSetBuff(void* pWriteBuff, unsigned int iWriteBuffSize, void* pReadBuff, unsigned int iReadBuffSize)
{
   // CdyStream::SetBuff(pWriteBuff,iWriteBuffSize,pReadBuff,iReadBuffSize);
    PCCOMBUFFER pCComBuffer;
   // if((m_iPort<0)||(m_iPort>4)) return;//端口号不在范围内，返回
    pCComBuffer = &g_CComBuffer[0];
    
    m_pWriteBuff = (unsigned char*)pWriteBuff;
    m_iWriteBuffSize = iWriteBuffSize;
    m_pReadBuff = (unsigned char*)pReadBuff;
    m_iReadBuffSize = iReadBuffSize;
    
    pCComBuffer->pTxdBuff = m_pWriteBuff;//设置发送缓冲区
    pCComBuffer->pRxdBuff = m_pReadBuff; //设置发接收缓冲区
    pCComBuffer->TxdBuffSize = m_iWriteBuffSize;//设置发送缓冲区长度
    pCComBuffer->RxdBuffSize = m_iReadBuffSize; //设置接收缓冲区长度
}


/**********************************************
*程序名：void Set_BaudRate(uint32_t BaudRate)
*功能  :串口波特率设置
*入口  ：uint32_t BaudRate
*出口  ： 
*备注  ：无
***********************************************/
void Set_BaudRate(uint32_t BaudRate)
{
  PCCOMBUFFER pCComBuffer;
  pCComBuffer = &g_CComBuffer[0];
  m_iBaud =  BaudRate;
  pCComBuffer->RxdDelayCntMax = m_iTimeout*10000/m_iBaud;
//  CdyUARTSetReadTimeout(100);
  if(pCComBuffer->RxdDelayCntMax < 5)
    pCComBuffer->RxdDelayCntMax = 5; //如果计算结果为0，则付值5ms
  if(pCComBuffer->RxdDelayCntMax > 500) pCComBuffer->RxdDelayCntMax = 500; //如果计算超过为500，则付值500ms
  
  pCComBuffer->pTxdBuff = m_pWriteBuff;//设置发送缓冲区
  pCComBuffer->pRxdBuff = m_pReadBuff; //设置发接收缓冲区
  pCComBuffer->TxdBuffSize = m_iWriteBuffSize;//设置发送缓冲区长度
  pCComBuffer->RxdBuffSize = m_iReadBuffSize; //设置接收缓冲区长度
  
  pCComBuffer->RxdByteCnt = 0;//接收指针复位
  pCComBuffer->pTxHead = 0;//发送头指针复位
  pCComBuffer->pTxTail = 0;//发送尾指针复位
  
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
//    if((m_iPort<0)||(m_iPort>4)) return;//端口号不在范围内，返回
    m_iTimeout = iTimeout;
    PCCOMBUFFER pCComBuffer;
    pCComBuffer = &g_CComBuffer[0];
    pCComBuffer->RxdDelayCntMax = m_iTimeout*10000/m_iBaud;
}

void CdyUARTWrite(void *pData, unsigned int iLen)
{
  //  USART_TypeDef*  USARTNo;
    unsigned int WriteBufferFreeSize = 0;
   // if((m_iPort<0)||(m_iPort>4)) return;//端口号不在范围内，返回
    if((iLen == 0)||(pData == NULL))   return; //如果长度为0或者数据为空则取消发送
  //  USARTNo = COM_USART_CHANNEL[m_iPort];    //查找串口
    //计算剩余的写空间的长度 WriteBufferFreeSize
    PCCOMBUFFER pCComBuffer;
    pCComBuffer = &g_CComBuffer[0];

   // USART_ITConfig(USARTNo, USART_IT_TXE, DISABLE);     //禁止发送中断
    USART_ITConfig(USART1,USART_IT_TXE, DISABLE); //禁止发送中断
    if(pCComBuffer->pTxdBuff == NULL) return;

    WriteBufferFreeSize = pCComBuffer->TxdBuffSize -  (pCComBuffer->pTxTail - pCComBuffer->pTxHead);
    if(iLen > WriteBufferFreeSize) iLen = WriteBufferFreeSize;//如果发送的长度大于缓冲区就取缓冲区的长度

    for(unsigned int i = 0;i < iLen; i++)
    {
        *(m_pWriteBuff + (pCComBuffer->pTxTail%pCComBuffer->TxdBuffSize)) =  *((unsigned char*)pData + i);
        pCComBuffer->pTxTail++;
    }
    pCComBuffer->CommStatus.COMM_SENDEND = 0; //清零发送完标志
    pCComBuffer->CommStatus.COMM_SENDING = 1; //设置正在发送志位
    
//    UART2_ITConfig(UART2_IT_TXE, ENABLE);     //启动发送发送
//    UART2_ITConfig(UART2_IT_TC, ENABLE);     
    USART_ITConfig( USART1,  USART_IT_RXNE, ENABLE ); //开启接收中断
	USART_ITConfig( USART1,  USART_IT_TC, DISABLE ); //关闭传输完成中断
//    USART_ITConfig(USARTNo, USART_IT_TXE, ENABLE);     //启动发送发送
//    USART_ITConfig(USARTNo, USART_IT_TC, ENABLE);
}


unsigned int CdyUARTRead(void *pData,unsigned int iLen)
{
    unsigned int t;
    //USART_TypeDef*  USARTNo;

    //if((m_iPort<0)||(m_iPort>4)) return 0;  //端口号不在范围内，返回
   // USARTNo = COM_USART_CHANNEL[m_iPort];   //查询串口

    PCCOMBUFFER pCComBuffer;
    pCComBuffer = &g_CComBuffer[0];

    if(pCComBuffer->pRxdBuff == NULL)  return 0;

    if(pCComBuffer->RxdDelayCnt > 0)   return  0;
      //if( (pCComBuffer->CommStatus.COMM_RECVING == 1) || (pCComBuffer->CommStatus.COMM_RECVEND == 0) || ( pCComBuffer->RxdByteCnt == 0))  //接收未完成或者接收数据为0，直接退出
    if( pCComBuffer->RxdByteCnt == 0)  //接收未完成或者接收数据为0，直接退出
    {
      t = 0;
    }
    else
    {
      //USART_ITConfig(USARTNo, USART_IT_RXNE, DISABLE);  //先禁止接收
//      UART2_ITConfig(UART2_IT_RXNE, DISABLE);    //先禁止接收
      USART_ITConfig( USART1,  USART_IT_RXNE, DISABLE ); //先禁止接收
      t = pCComBuffer->RxdByteCnt;
      if(t > iLen)   t = iLen;                          //计算要读取的长度
      m_pReadBuff = pCComBuffer->pRxdBuff;              //读取接收缓存区指针
      memcpy(pData,m_pReadBuff,t);                      //读取接收的数据
      pCComBuffer->RxdByteCnt = 0;                      //清接收指针
      pCComBuffer->CommStatus.COMM_RECVEND = 0;         //清零接收完成标志
      // USART_ITConfig(USARTNo, USART_IT_RXNE, ENABLE); //打开接收中断
//      UART2_ITConfig(UART2_IT_RXNE, ENABLE); 
      USART_ITConfig( USART1,  USART_IT_RXNE, ENABLE ); //打开接收中断
    }
    return t;
}


void CdyUARTRun(unsigned int ms)   //ms是传递进来的ms数
{
    PCCOMBUFFER pCComBuffer;
 //   if((m_iPort<0)||(m_iPort>4)) return;//端口号不在范围内，返回
    pCComBuffer = &g_CComBuffer[0];

    if( pCComBuffer->RxdDelayCnt > 0)   //接收超时判断
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


//串口工作状态
bool CdyUARTIsBusy(void)
{
    PCCOMBUFFER pCComBuffer;
   // if((m_iPort<0)||(m_iPort>4)) return false;//端口号不在范围内，返回
    pCComBuffer = &g_CComBuffer[0];

    if(pCComBuffer->CommStatus.COMM_IDLE == 1)
        return FALSE;
    else
        return TRUE;
}


//串口函数的通用处理函数
void Usartx_IrqRXHandler(void)
{
  unsigned char RxData;
  PCCOMBUFFER  pCComBuffer;
  pCComBuffer = &g_CComBuffer[0];
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    if(pCComBuffer->pRxdBuff == NULL) return;              //如果接收缓冲区指针为空，直接返回
    /* Read one byte from the receive data register */
    RxData = USART_ReceiveData8(USART1);                     //接收一个字节数据
    if(pCComBuffer->RxdByteCnt < pCComBuffer->RxdBuffSize) //小于缓冲区长度//满了则不再接收
    {
        pCComBuffer->pRxdBuff[pCComBuffer->RxdByteCnt] =  RxData;
        pCComBuffer->CommStatus.COMM_RECVEND = 0;
        pCComBuffer->CommStatus.COMM_RECVING = 1;
        
        pCComBuffer->RxdDelayCnt = pCComBuffer->RxdDelayCntMax; //接受完成时间延时
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
    if(pCComBuffer->pTxdBuff == NULL) return;//如果发送缓冲区指针为空，直接返回
    if(pCComBuffer->pTxHead >= pCComBuffer->pTxTail)
    {			
        USART_ITConfig( USART1,  USART_IT_TXE, DISABLE ); //先禁止接收
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
    //最后一个字节的所有bit都发完
  if(USART_GetITStatus(USART1,USART_IT_TC) != RESET)
  {  
    USART_ITConfig(USART1,USART_IT_TC, DISABLE);
    pCComBuffer->CommStatus.COMM_SENDING = 0;			 	
    pCComBuffer->CommStatus.COMM_SENDEND = 1;
    
  }
}

void DebugCom_test(void)
{
  //设置接收发送缓存指针和大小
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

