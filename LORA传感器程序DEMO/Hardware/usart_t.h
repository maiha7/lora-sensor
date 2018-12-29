/*************************************
                ��������
**************************************/

#ifndef  _USART_H
#define  _USART_H
#include "stm8l15x_usart.h"

//����״̬��ʾ
//extern u8   m_pWriteBuff[50];
//extern u8   m_pReadBuff[50];
typedef struct _COMSTATUSBIT
{
    u8 COMM_SENDING:1;  //1��ʾ���ڷ���
    u8 COMM_RECVING:1;  //1��ʾ���ڽ���
    u8 COMM_SENDEND:1;  //1��ʾ�������
    u8 COMM_RECVEND:1;  //1��ʾ�������
    u8 COMM_IDLE:1;     //1��ʾ����
    u8 Reservebit5:1;   //3������λ
    u8 Reservebit6:1;
    u8 Reservebit7:1;
}COMSTATUS;

typedef struct _CCOMBUFFER
{
    int  RxdDelayCnt;            //�հ���ʱ������ȷ���յ����ݰ����
    int  RxdDelayCntMax;         //�հ������ж���ʱ��ֵ

    unsigned int  RxdByteCnt;   //�����ֽڼ�����ÿ����һ���ֽڣ���ֵ��1
    unsigned int  pTxHead;      //���ͻ�����ͷָ��
    unsigned int  pTxTail;      //���ͻ�����βָ��

    unsigned char*  pRxdBuff;   // ���ջ�����ָ��
    unsigned char*  pTxdBuff;   //���ͻ�����ָ��

    unsigned int  RxdBuffSize;  //���ջ���������
    unsigned int  TxdBuffSize;  //���ͻ��������� 
    COMSTATUS   CommStatus;     //����״̬
    
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


//����UART��TX��RX����
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





