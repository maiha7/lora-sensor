/*************************************
                今明电子
**************************************/

#ifndef  _USART_H
#define  _USART_H
#include "stm8l15x_usart.h"

//串口状态表示
//extern u8   m_pWriteBuff[50];
//extern u8   m_pReadBuff[50];
typedef struct _COMSTATUSBIT
{
    u8 COMM_SENDING:1;  //1表示正在发送
    u8 COMM_RECVING:1;  //1表示正在接收
    u8 COMM_SENDEND:1;  //1表示发送完毕
    u8 COMM_RECVEND:1;  //1表示接收完毕
    u8 COMM_IDLE:1;     //1表示空闲
    u8 Reservebit5:1;   //3个保留位
    u8 Reservebit6:1;
    u8 Reservebit7:1;
}COMSTATUS;

typedef struct _CCOMBUFFER
{
    int  RxdDelayCnt;            //收包延时，用于确认收到数据包完成
    int  RxdDelayCntMax;         //收包结束判断延时阀值

    unsigned int  RxdByteCnt;   //接收字节记数，每接收一个字节，该值加1
    unsigned int  pTxHead;      //发送缓冲区头指针
    unsigned int  pTxTail;      //发送缓冲区尾指针

    unsigned char*  pRxdBuff;   // 接收缓冲区指针
    unsigned char*  pTxdBuff;   //发送缓冲区指针

    unsigned int  RxdBuffSize;  //接收缓冲区长度
    unsigned int  TxdBuffSize;  //发送缓冲区长度 
    COMSTATUS   CommStatus;     //串口状态
    
}CCOMBUFFER,*PCCOMBUFFER;

#define  DebugComBufferSize 40
extern unsigned char DebugComRxbuffer[DebugComBufferSize];
extern unsigned char DebugComTxbuffer[DebugComBufferSize];

extern CCOMBUFFER g_CComBuffer[1];
extern void CdyUARTRAM(void);
//extern void Usartx_IrqHandler(void);
extern void Usartx_IrqTXHandler(void);
extern void Usartx_IrqRXHandler(void);
extern bool CdyUARTIsBusy(void);
extern void CdyUARTRun(unsigned int ms);
extern unsigned int CdyUARTRead(void *pData,unsigned int iLen);
extern void CdyUARTWrite(void *pData, unsigned int iLen);
extern void CdyUARTSetReadTimeout(unsigned int iTimeout);
extern void Set_BaudRate(uint32_t BaudRate);
extern void CdyUARTSetBuff(void* pWriteBuff, unsigned int iWriteBuffSize, void* pReadBuff, unsigned int iReadBuffSize);
extern void DebugCom_test(void);

//extern unsigned int m_iPort ,m_iTimeout;

void USART2_CONFIG(void);
void USARTx_Init(u8 channel);
void UART_IOConfig(void);
//extern void NW_COMInit2(COM_TypeDef COM_CHANNEL,uint32_t USART_Baud);


//定义UART的TX、RX引脚
#define  TXPort  GPIOD
#define  TXPin   (1 << 5) 
#define  RXPort  GPIOD
#define  RXPin   (1 << 6)

void UARTInit(void);
void UARTSendData(u8 data);
void UARTSendBuf(u8* Data, u16 len);
void UARTSendString(char *format, ...);
u8 UARTReceiveByte(void);
static char *itoa(int value, char *string, int radix);
void UART1Printf(u8 *Data,...);

#endif





