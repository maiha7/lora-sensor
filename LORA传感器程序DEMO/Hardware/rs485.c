/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * 文件名称：rs485.c
 * 文件标识：
 * 摘    要：485通讯
 * 
 * 当前版本：V1.0.0
 * 作    者：yjd
 * 完成日期：2017-03-23
**************************************************/

#include "rs485.h"
#include "led.h"
#include "string.h"  
#include "flash.h" 
#include "apply.h" 

	
u8 RS485_RX_BUF[64];  //接收缓冲,最大64个字节.

u8 RS485_RX_CNT = 0;  //接收字节计数

u8 RS485_RX_END = 0; //接收完数据

u8 RS485_RX_LEN = 0;//接收到的数据长度

char g_ID[4] = {0}; //配置的设备ID

/*******************************************************************************
 * 名称: void rs485_init(void)
 * 功能: RS485初始化
 * 形参: 无
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/
void rs485_init(void)
{
  CLK_PeripheralClockConfig(CLK_Peripheral_USART1,ENABLE);

  //  SYSCFG_REMAPDeInit();
  // SYSCFG_REMAPPinConfig(REMAP_Pin_USART1TxRxPortC,ENABLE);
  GPIO_Init(RS485_PORT, RS485_EN, GPIO_Mode_Out_PP_High_Fast);//EN
  GPIO_Init(RS485_PORT, RS485_TX, GPIO_Mode_Out_PP_High_Fast);//TXD
  GPIO_Init(RS485_PORT, RS485_RX, GPIO_Mode_In_PU_No_IT);//RXD
  USART_DeInit(USART1);		//复位USART1 
  USART_Init(USART1,(u32)RS485_BAUDRATE, USART_WordLength_8b, USART_StopBits_1,
                            USART_Parity_No, USART_Mode_Tx_And_Rx);
  USART_ClearITPendingBit(USART1, USART_IT_RXNE);//清除串口中断标志位
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启接收中断
  //USART_ITConfig(USART1, USART_IT_TC, ENABLE);//开启接收中断
  ITC_SetSoftwarePriority(USART1_RX_IRQn, ITC_PriorityLevel_2);//设置优先级
  USART_Cmd(USART1, ENABLE);	//使能USART1
  GPIO_ResetBits(RS485_PORT, RS485_EN);//使能拉低
}

/*******************************************************************************
 * 名称: INTERRUPT_HANDLER(USART1_RX_TIM5_CC_IRQHandler,28)
 * 功能: USART1中断函数
 * 形参: USART1_RX_TIM5_CC_IRQHandler
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/
INTERRUPT_HANDLER(USART1_RX_TIM5_CC_IRQHandler,28)
{
  u8 res;	    
  if(USART_GetITStatus(USART1, USART_IT_RXNE)) //接收到数据
  {	                     
    res = UART1_ReceiveByte(); 	//读取接收到的数据
    if(RS485_RX_CNT < sizeof(RS485_RX_BUF))
    {
      RS485_RX_BUF[RS485_RX_CNT] = res;		//记录接收到的值
      RS485_RX_CNT++;		//接收数据增加1 
    } 
  }  											 
}

/*******************************************************************************
 * 名称: void RS485_Send_Data(u8 *buf,u8 len)
 * 功能: RS485发送字节
 * 形参: buf-> 要发送的字节首地址 len-> 发送的字节长度
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/

void RS485_Send_Data(u8 *buf,u8 len)
{
  if(buf == NULL)
  {
    return;
  }
  u8 t;
  u8 *p = buf;
  GPIO_SetBits(RS485_PORT, RS485_EN);	//设置为发送模式
  for(t = 0;t < len;t++)		//循环发送数据
  {		   
    while( RESET == USART_GetFlagStatus(USART1, USART_FLAG_TC));	  
    USART_SendData8(USART1,p[t]);
  }	 

  while(RESET == USART_GetFlagStatus(USART1, USART_FLAG_TC));		
  RS485_RX_CNT = 0;	  
  GPIO_ResetBits(RS485_PORT, RS485_EN); //设置为接收模式	
}

/*******************************************************************************
 * 名称: u8 RS485_Receive_Data(u8 *data,u8 *buf,u8 len)
 * 功能: RS485接收数据
 * 形参: u8 *data 读取数据  u8 *buf 接收数据缓存  u8 len 接收数据长度
 * 返回: 读到的字节长度
 * 说明: 无 
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
  len = 0; //清零
  memset(bufp,0,sizeof(bufp));
  return rxlen;
}
/*******************************************************************************
 * 名称: void Rs485_SendStr(u8 *str)
 * 功能: 发送字符串
 * 形参: str
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/
void Rs485_SendStr(u8 *str)
{
  GPIO_SetBits(RS485_PORT, RS485_EN);	
  while(*str != '\0')
  {
    UART1_SendByte(*str++);	/* 循环调用发送一个字符函数 */
  }
  GPIO_ResetBits(RS485_PORT, RS485_EN); 	
}


/**************************************************
 * 函数名称：void rs485_communication(void)
 * 函数介绍：RS485通讯
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/
void rs485_communication(void)
{
  u8 i = 0;
  u8 j = 0;
  u8 setlenth = 0; //设置的字符串长度
  char* strp = NULL; //‘=’后面字符串地址
  u8 strlenth = 0; //‘=’后面字符串长度
  u8 rs485_rx_buf[64]; 
  char rs485_send_str[64];
  if(RS485_RX_END == TRUE)//超时结束
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
      else if(strlenth == 4) //四个ID字符
            {
              for(j = 0; j < 4; j++)
              {
                FLASH_EraseByte(BASE_ADDR+j);         //再擦除
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
 * 名称: UART1_SendByte
 * 功能: UART1发送一个字节
 * 形参: data -> 要发送的字节
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/
void UART1_SendByte(u8 data)
{
  USART_SendData8(USART1, data);
  /* 等待传输结束 */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

/*******************************************************************************
 * 名称: UART1_SendStr
 * 功能: UART1发送len个字符
 * 形参: data -> 指向要发送的字符串
 *       len -> 要发送的字节数
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/
void UART1_SendStr(u8 *str)
{
  while(*str != '\0')
  {
    UART1_SendByte(*str++);	/* 循环调用发送一个字符函数 */
  }	
}

/*******************************************************************************
 * 名称: UART1_ReceiveByte
 * 功能: UART1接收一个字符
 * 形参: 无
 * 返回: 接收到的字符
 * 说明: 无 
 ******************************************************************************/
u8 UART1_ReceiveByte(void)
{
  u8 UART1_RX_BUF; 
  /* 等待接收完成 */
  while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
  UART1_RX_BUF = USART_ReceiveData8(USART1);
  return  UART1_RX_BUF;
}

/*******************************************************************************
 * 名称: fputc
 * 功能: 重定向c库函数printf到UART1
 * 形参: 无
 * 返回: 要打印的字符
 * 说明: 由printf调用 
 ******************************************************************************/
#ifdef _IAR_
int fputc(int ch, FILE *f)
{  
  /* 将Printf内容发往串口 */
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

