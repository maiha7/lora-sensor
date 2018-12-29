/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * 文件名称：sx1278.c
 * 文件标识：
 * 摘    要：LORA芯片sx1278驱动
 * 
 * 当前版本：V1.0.0
 * 作    者：yjd
 * 完成日期：2017-10-12
**************************************************/
#include "led.h"
#include "sx1278.h"
#include "stm8l15x.h"
#include "stm8l15x_gpio.h"
#include "ds18b20.h"
#include "gy3x.h"
#include "sht2x.h"
//#include "mq137.h"
#include "Crc8.h"
#include "DriverConfig.h"


uint8_t   Frequency[3] = {0x6c,0x80,0x00};

uint8_t   SpreadingFactor = 11;    //7-12,扩频因子选小，发射时间会快一些

uint8_t   CodingRate = 2;        //1-4

uint8_t   Bw_Frequency = 7;      //6-9

uint8_t   powerValue = 7;   //功率设置

uint8_t   power_data[8]={0X80,0X80,0X80,0X83,0X86,0x89,0x8c,0x8f};

uint8_t   RF_EX0_STATUS; //读取到的状态值

uint8_t   CRC_Value;  

uint8_t   SX1278_RLEN; //接收到的字节数

uint8_t   lora_slen; //LORA发送字节数

uint8_t   Lora_recvdata[RECVLEN] = {0}; //lora接收数据

uint8_t   Lora_senddata[SENDLEN] = {0};//lora发送数据

uint8_t   device_addr_msb = 0; //子机地址高字节

uint8_t   device_addr_lsb = 0; //子机地址低字节

uint8_t   vbat = 0; //子机电池电压值

uint8_t   device_type = DEVICE_TYPE ; //定义设备类型
/**************************************************
 * 函数名称：void SX1278_GPIO(void)
 * 函数介绍：控制SX1278的相关引脚初始化
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/
void SX1278_GPIO(void)
{
   GPIO_Init(TXEN_PORT ,TXEN_PIN, GPIO_Mode_Out_PP_Low_Slow);//TXD
   
   GPIO_Init(RXEN_PORT ,RXEN_PIN, GPIO_Mode_Out_PP_Low_Slow);//RXD
   
   GPIO_Init(RESET_PORT,RESET_PIN,GPIO_Mode_Out_PP_Low_Slow);//RESET
   
   GPIO_Init(CCGDO0_PORT,CCGDO0_PIN,GPIO_Mode_In_PU_No_IT);//GIO0
   
//   GPIO_Init(GPIOC,GPIO_PIN_5,GPIO_Mode_In_PU_No_IT);//GIO1
   
   GPIO_Init(CCGDO2_PORT,CCGDO2_PIN, GPIO_Mode_In_PU_No_IT);//GIO2
   
//   GPIO_Init(GPIOE,GPIO_PIN_5,GPIO_Mode_In_PU_No_IT);//GIO3
   
   GPIO_Init(RFCLK_PORT,RFCLK_PIN,GPIO_Mode_Out_PP_Low_Slow);//SCK
   
   GPIO_Init(RFMISO_PORT,RFMISO_PIN,GPIO_Mode_In_PU_No_IT);//SDO MISO
   
   GPIO_Init(RFMOSI_PORT,RFMOSI_PIN,GPIO_Mode_Out_PP_Low_Slow);//SDI MOSI
   
   GPIO_Init(RFCE_PORT,RFCE_PIN,GPIO_Mode_Out_PP_Low_Slow);//CS
   
   GPIO_Init(RFPWCTRL_PORT,RFPWCTRL_PIN,GPIO_Mode_Out_PP_Low_Slow);//RFPW
}

/**************************************************
 * 函数名称：void FSK_SEND_PACKET(void)
 * 函数介绍：FSK发送数据包
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/
void FSK_SEND_PACKET(void)
{
  SX1278LoRaSetOpMode( Stdby_mode );
  SX1278WriteBuffer(REG_LR_DIOMAPPING1,0x01);
  SX1278WriteBuffer(REG_LR_DIOMAPPING2,0x20);
}
/**************************************************
 * 函数名称：void SX1278Reset(void)
 * 函数介绍：SX1278复位
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/	
void SX1278Reset(void)
{
   RF_REST_L;	
   delay_ms(200);
   RF_REST_H;
   delay_ms(500);
}
/**************************************************
 * 函数名称：void SX1278_INIT(void)
 * 函数介绍：SX1278的SPI相关引脚初始化
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/	
void SX1278_INIT(void)
{
   RF_CKL_L;
   RF_CE_H;
   RF_SDI_H;
}
/**************************************************
 * 函数名称：void RF_SPI_MasterIO(uint8_t out)
 * 函数介绍：向SX1278写字节
 * 输入参数：uint8_t out ，写入字节
 * 输出参数：
 * 返回值  ：
 **************************************************/		
void RF_SPI_MasterIO(uint8_t out)
{
  uint8_t i;
  for (i = 0;i < 8;i++)
  {   
    if (out & 0x80)			/* check if MSB is high */
    {
      RF_SDI_H;
    }
    else 
    {
      RF_SDI_L;							/* if not, set to low */
    }	 
    RF_CKL_H;						  /* toggle clock high */
    delay_us(40);
    out = (out << 1);					/* shift 1 place for next bit */
    RF_CKL_L;							/* toggle clock low */
    delay_us(40);
  }
}
/**************************************************
 * 函数名称：uint8_t RF_SPI_READ_BYTE()
 * 函数介绍：从SX1278读字节
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/	
uint8_t RF_SPI_READ_BYTE()
{	 
  uint8_t j;
  uint8_t i;
  j = 0;
  for (i = 0; i < 8; i++)
  {	 
    RF_CKL_H; 
    delay_us(40);
    j = (j << 1);						 // shift 1 place to the left or shift in 0 //
    if( SX1278_SDO )							 // check to see if bit is high //
    {
      j = j | 0x01; 					   // if high, make bit high //
    }												  // toggle clock high // 
    RF_CKL_L; 							 // toggle clock low //  
    delay_us(40);
  }
  return j;								// toggle clock low //
}

/**************************************************
 * 函数名称：void SX1278WriteBuffer( uint8_t addr, uint8_t buffer)
 * 函数介绍：向SX1278某个寄存器地址写字节
 * 输入参数：uint8_t addr ，寄存器地址 ； uint8_t buffer，写入字节
 * 输出参数：
 * 返回值  ：
 **************************************************/	  
void SX1278WriteBuffer( uint8_t addr, uint8_t buffer)
{ 
  RF_CE_L; //NSS = 0;
  RF_SPI_MasterIO( addr | 0x80 );
  RF_SPI_MasterIO( buffer);
  RF_CE_H; //NSS = 1;
}
/**************************************************
 * 函数名称：uint8_t SX1278ReadBuffer(uint8_t addr)
 * 函数介绍：从SX1278某个寄存器地址读字节
 * 输入参数：uint8_t addr ，寄存器地址
 * 输出参数：uint8_t Value，读出的字节
 * 返回值  ：
 **************************************************/		
uint8_t SX1278ReadBuffer(uint8_t addr)
{
  uint8_t Value;
  RF_CE_L; //NSS = 0;
  RF_SPI_MasterIO( addr & 0x7f  );
  Value = RF_SPI_READ_BYTE();
  RF_CE_H; //NSS = 1;

  return Value; 
}

/**************************************************
 * 函数名称：void SX1278LoRaSetOpMode( RFMode_SET opMode )
 * 函数介绍：设置SX1278的操作模式
 * 输入参数：RFMode_SET opMode ，操作模式
 * 输出参数：
 * 返回值  ：
 **************************************************/
void SX1278LoRaSetOpMode( RFMode_SET opMode )
{
  uint8_t opModePrev;
  opModePrev = SX1278ReadBuffer(REG_LR_OPMODE);
  opModePrev &= 0xf8;
  opModePrev |= (uint8_t)opMode;
  SX1278WriteBuffer(REG_LR_OPMODE, opModePrev);		
}
/**************************************************
 * 函数名称：void SX1278LoRaFsk( Debugging_fsk_ook opMode )
 * 函数介绍：设置SX1278的调制方式
 * 输入参数：Debugging_fsk_ook opMode  ，调制方式
 * 输出参数：
 * 返回值  ：
 **************************************************/
void SX1278LoRaFsk( Debugging_fsk_ook opMode )
{
  uint8_t opModePrev;
  opModePrev = SX1278ReadBuffer(REG_LR_OPMODE);
  opModePrev &= 0x7F;
  opModePrev |= (uint8_t)opMode;
  SX1278WriteBuffer( REG_LR_OPMODE, opModePrev);		
}
/**************************************************
 * 函数名称：void SX1278LoRaSetRFFrequency(void)
 * 函数介绍：设置SX1278的频率
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/	
void SX1278LoRaSetRFFrequency(void)
{
  SX1278WriteBuffer( REG_LR_FRFMSB, Frequency[0]);
  SX1278WriteBuffer( REG_LR_FRFMID, Frequency[1]);
  SX1278WriteBuffer( REG_LR_FRFLSB, Frequency[2]);
}
/**************************************************
 * 函数名称：void SX1278LoRaSetRFPower( uint8_t power )
 * 函数介绍：设置SX1278的RF功率
 * 输入参数：uint8_t power，功率值
 * 输出参数：
 * 返回值  ：
 **************************************************/		
void SX1278LoRaSetRFPower( uint8_t power )
{
  SX1278WriteBuffer( REG_LR_PADAC, 0x87);
  SX1278WriteBuffer( REG_LR_PACONFIG,  power_data[power] );
}
/**************************************************
 * 函数名称：void SX1278LoRaSetSpreadingFactor( uint8_t factor )
 * 函数介绍：设置SX1278的扩频因子
 * 输入参数：uint8_t factor ，扩频因子
 * 输出参数：
 * 返回值  ：
 **************************************************/		
void SX1278LoRaSetSpreadingFactor( uint8_t factor )
{
  uint8_t RECVER_DAT;
  SX1278LoRaSetNbTrigPeaks( 3 );
  RECVER_DAT=SX1278ReadBuffer( REG_LR_MODEMCONFIG2);	  
  RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG2_SF_MASK ) | ( factor << 4 );
  SX1278WriteBuffer( REG_LR_MODEMCONFIG2, RECVER_DAT );	 
}
/**************************************************
 * 函数名称：void SX1278LoRaSetNbTrigPeaks( uint8_t value )
 * 函数介绍：设置SX1278的扩频触发尖峰
 * 输入参数： uint8_t value，设置值
 * 输出参数：
 * 返回值  ：
 **************************************************/	
void SX1278LoRaSetNbTrigPeaks( uint8_t value )
{
  uint8_t RECVER_DAT;
  RECVER_DAT = SX1278ReadBuffer( 0x31);
  RECVER_DAT = ( RECVER_DAT & 0xF8 ) | value;
  SX1278WriteBuffer( 0x31, RECVER_DAT );
}
/**************************************************
 * 函数名称：void SX1278LoRaSetErrorCoding( uint8_t value )
 * 函数介绍：设置SX1278的有效数据比
 * 输入参数： uint8_t value，设置值
 * 输出参数：
 * 返回值  ：
 **************************************************/		
void SX1278LoRaSetErrorCoding( uint8_t value )
{	
  uint8_t RECVER_DAT;
  RECVER_DAT=SX1278ReadBuffer( REG_LR_MODEMCONFIG1);
  RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG1_CODINGRATE_MASK ) | ( value << 1 );
  SX1278WriteBuffer( REG_LR_MODEMCONFIG1, RECVER_DAT);
  // LoRaSettings.ErrorCoding = value;
}
/**************************************************
 * 函数名称：void SX1278LoRaSetPacketCrcOn( bool enable )
 * 函数介绍：设置SX1278的CRC校验
 * 输入参数：bool enable，使能
 * 输出参数：
 * 返回值  ：
 **************************************************/	
void SX1278LoRaSetPacketCrcOn( bool enable )
{	
  uint8_t RECVER_DAT;
  RECVER_DAT= SX1278ReadBuffer( REG_LR_MODEMCONFIG2);
  RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) | ( enable << 2 );
  SX1278WriteBuffer( REG_LR_MODEMCONFIG2, RECVER_DAT );
}
/**************************************************
 * 函数名称：void SX1278LoRaSetSignalBandwidth( uint8_t bw )
 * 函数介绍：设置SX1278的扩频带宽
 * 输入参数： uint8_t bw，带宽值
 * 输出参数：
 * 返回值  ：
 **************************************************/	
void SX1278LoRaSetSignalBandwidth( uint8_t bw )
{
  uint8_t RECVER_DAT;
  RECVER_DAT=SX1278ReadBuffer( REG_LR_MODEMCONFIG1);
  RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG1_BW_MASK ) | ( bw << 4 );
  SX1278WriteBuffer( REG_LR_MODEMCONFIG1, RECVER_DAT );
  // LoRaSettings.SignalBw = bw;
}
/**************************************************
 * 函数名称：void SX1278LoRaSetImplicitHeaderOn( bool enable )
 * 函数介绍：设置SX1278的同步头模式
 * 输入参数：bool enable ，使能
 * 输出参数：
 * 返回值  ：
 **************************************************/		
void SX1278LoRaSetImplicitHeaderOn( bool enable )
{
  uint8_t RECVER_DAT;
  RECVER_DAT=SX1278ReadBuffer( REG_LR_MODEMCONFIG1 );
  RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) | ( enable );
  SX1278WriteBuffer( REG_LR_MODEMCONFIG1, RECVER_DAT );
}
/**************************************************
 * 函数名称：void SX1278LoRaSetSymbTimeout( uint16_t value )
 * 函数介绍：设置SX1278的标志超时值
 * 输入参数：uint16_t value ，设置值
 * 输出参数：
 * 返回值  ：
 **************************************************/		
void SX1278LoRaSetSymbTimeout( uint16_t value )
{
  uint8_t RECVER_DAT[2];
  RECVER_DAT[0]=SX1278ReadBuffer( REG_LR_MODEMCONFIG2 );
  RECVER_DAT[1]=SX1278ReadBuffer( REG_LR_SYMBTIMEOUTLSB );
  RECVER_DAT[0] = ( RECVER_DAT[0] & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) | ( ( value >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK );
  RECVER_DAT[1] = value & 0xFF;
  SX1278WriteBuffer( REG_LR_MODEMCONFIG2, RECVER_DAT[0]);
  SX1278WriteBuffer( REG_LR_SYMBTIMEOUTLSB, RECVER_DAT[1]);
}
/**************************************************
 * 函数名称：void SX1278LoRaSetPayloadLength( uint8_t value )
 * 函数介绍：设置SX1278的数据有效长度
 * 输入参数：uint8_t value ，设置值
 * 输出参数：
 * 返回值  ：
 **************************************************/		
void SX1278LoRaSetPayloadLength( uint8_t value )
{
  SX1278WriteBuffer( REG_LR_PAYLOADLENGTH, value );
} 
/**************************************************
 * 函数名称：void SX1278LoRaSetPreamLength( uint16_t value )
 * 函数介绍：设置SX1278的数据序头长度
 * 输入参数：uint16_t value ，设置值
 * 输出参数：
 * 返回值  ：
 **************************************************/	
void SX1278LoRaSetPreamLength( uint16_t value )
{
  uint8_t a[2];
  a[0]=(value&0xff00)>>8;
  a[1]=value&0xff;
  SX1278WriteBuffer( REG_LR_PREAMBLEMSB, a[0] );
  SX1278WriteBuffer( REG_LR_PREAMBLELSB, a[1] );
}
/**************************************************
 * 函数名称：void SX1278LoRaSetMobileNode( bool enable )
 * 函数介绍：设置SX1278的低数据优化
 * 输入参数：bool enable ，使能
 * 输出参数：
 * 返回值  ：
 **************************************************/		
void SX1278LoRaSetMobileNode( bool enable )
{	 
  uint8_t RECVER_DAT;
  RECVER_DAT=SX1278ReadBuffer( REG_LR_MODEMCONFIG3 );
  RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG3_MOBILE_NODE_MASK ) | ( enable << 3 );
  SX1278WriteBuffer( REG_LR_MODEMCONFIG3, RECVER_DAT );
}

/**************************************************
 * 函数名称：void SX1278LORA_INT(void)
 * 函数介绍：SX1278各模式初始化
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/	
void SX1278LORA_INT(void)
{
  RFPOWER_EN;     //电源开启
  SX1278Reset();
  SX1278_INIT();
  SX1278LoRaSetOpMode(Sleep_mode);  //设置睡眠模式
  SX1278LoRaFsk(LORA_mode);	   // 设置扩频模式
  SX1278LoRaSetOpMode(Stdby_mode);  // 设置为普通模式
  SX1278WriteBuffer( REG_LR_DIOMAPPING1,GPIO_VARE_1);
  SX1278WriteBuffer( REG_LR_DIOMAPPING2,GPIO_VARE_2);
  SX1278LoRaSetRFFrequency();
  SX1278LoRaSetRFPower(powerValue);
  SX1278LoRaSetSpreadingFactor(SpreadingFactor);			 // 扩频因子设置
  SX1278LoRaSetErrorCoding(CodingRate);				//有效数据比
  SX1278LoRaSetPacketCrcOn( TRUE);			 //CRC 校验打开
  SX1278LoRaSetSignalBandwidth( Bw_Frequency );			//设置扩频带宽
  SX1278LoRaSetImplicitHeaderOn( FALSE);		 //同步头是显性模式
  SX1278LoRaSetPayloadLength( 0xff);
  SX1278LoRaSetSymbTimeout( 0x3FF );
  SX1278LoRaSetMobileNode(TRUE ); 			// 低数据的优化
}
/**************************************************
 * 函数名称：void FUN_RF_SENDPACKET(uint8_t *RF_TRAN_P,uint8_t LEN)
 * 函数介绍：SX1278开启发送模式，并发送数据包
 * 输入参数：uint8_t *RF_TRAN_P，发送数据起始地址；uint8_t LEN，数据长度
 * 输出参数：
 * 返回值  ：
 **************************************************/		
void FUN_RF_SENDPACKET(uint8_t *RF_TRAN_P,uint8_t LEN)
{
  uint8_t ASM_i;
  PA_TXD_OUT();
  SX1278LoRaSetOpMode( Stdby_mode );
  SX1278WriteBuffer( REG_LR_HOPPERIOD, 0 );	//不做频率跳变
  SX1278WriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_TXD_Value);	//打开发送中断
  SX1278WriteBuffer( REG_LR_PAYLOADLENGTH, LEN);	 //最大数据包
  SX1278WriteBuffer( REG_LR_FIFOTXBASEADDR, 0);
  SX1278WriteBuffer( REG_LR_FIFOADDRPTR, 0 );
  RF_CE_L;
  RF_SPI_MasterIO( 0x80 );
  for( ASM_i = 0; ASM_i < LEN; ASM_i++ )
  {
    RF_SPI_MasterIO( *RF_TRAN_P );
    RF_TRAN_P++;
  }
  RF_CE_H;
  SX1278WriteBuffer(REG_LR_DIOMAPPING1,0x40);
  SX1278WriteBuffer(REG_LR_DIOMAPPING2,0x00);
  SX1278LoRaSetOpMode( Transmitter_mode );
}
/**************************************************
 * 函数名称：void RF_RECEIVE (void)
 * 函数介绍：开启接收模式
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/
void RF_RECEIVE (void)
{
  SX1278LoRaSetOpMode( Stdby_mode );
  SX1278WriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);  //打开发送中断
  SX1278WriteBuffer(REG_LR_HOPPERIOD, PACKET_MIAX_Value );
  SX1278WriteBuffer( REG_LR_DIOMAPPING1, 0X00 );
  SX1278WriteBuffer( REG_LR_DIOMAPPING2, 0X00 );	
  SX1278LoRaSetOpMode( Receiver_mode );
  PA_RXD_OUT();
}
/**************************************************
 * 函数名称：void RF_CAD_RECEIVE (void)
 * 函数介绍：开启CAD模式
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/	
void RF_CAD_RECEIVE (void)
{
  SX1278LoRaSetOpMode( Stdby_mode );
  SX1278WriteBuffer(REG_LR_IRQFLAGSMASK,  IRQN_CAD_Value);	//打开CAD中断
  SX1278WriteBuffer( REG_LR_DIOMAPPING1, 0X80 );
  SX1278WriteBuffer( REG_LR_DIOMAPPING2, 0X00 );	
  SX1278LoRaSetOpMode( CAD_mode );
  PA_RXD_OUT();
}
/**************************************************
 * 函数名称：void RF_SEELP(void)
 * 函数介绍：开启睡眠模式
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/		
void RF_SEELP(void)
{
  SX1278LoRaSetOpMode( Stdby_mode );
  SX1278WriteBuffer(REG_LR_IRQFLAGSMASK,  IRQN_SEELP_Value);  //打开SEELP中断
  SX1278WriteBuffer( REG_LR_DIOMAPPING1, 0X00 );
  SX1278WriteBuffer( REG_LR_DIOMAPPING2, 0X00 );	
  SX1278LoRaSetOpMode( Sleep_mode );
  //PA_SEELP_OUT();
}

/**************************************************
 * 函数名称：void SX1278_Interupt(void)
 * 函数介绍：SX1278中断处理
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/
void SX1278_Interupt(void)
{
  RF_EX0_STATUS=SX1278ReadBuffer( REG_LR_IRQFLAGS ); 
  if((RF_EX0_STATUS&0x40)==0x40) //接收完成
  {
    CRC_Value=SX1278ReadBuffer( REG_LR_MODEMCONFIG2 );
    if((CRC_Value&0x04)==0x04)
    {
      SX1278WriteBuffer (REG_LR_FIFOADDRPTR,0x00);
      SX1278_RLEN = SX1278ReadBuffer(REG_LR_NBRXBYTES);
      RF_CE_L;
      RF_SPI_MasterIO( 0x00 );
      for(uint8_t RF_REC_RLEN_i = 0;RF_REC_RLEN_i < SX1278_RLEN;RF_REC_RLEN_i++)
      {
        Lora_recvdata[RF_REC_RLEN_i]=RF_SPI_READ_BYTE();	    
      }
      RF_CE_H;
    }
    SX1278LoRaSetOpMode( Stdby_mode );
    SX1278WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value);  //打开发送中断
    SX1278WriteBuffer(REG_LR_HOPPERIOD,    PACKET_MIAX_Value);
    SX1278WriteBuffer( REG_LR_DIOMAPPING1, 0X00 );
    SX1278WriteBuffer( REG_LR_DIOMAPPING2, 0x00 );	
    SX1278LoRaSetOpMode( Receiver_mode );
    PA_RXD_OUT();
//    flag.Abit.rfModule_ReceiveOK = 1;
  }
  else 
  {
    if((RF_EX0_STATUS&0x08)==0x08) //发送完成
    {
      SX1278LoRaSetOpMode( Stdby_mode );
      SX1278WriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);  //打开接收中断
      SX1278WriteBuffer(REG_LR_HOPPERIOD,   PACKET_MIAX_Value );
      SX1278WriteBuffer( REG_LR_DIOMAPPING1, 0X00 );
      SX1278WriteBuffer( REG_LR_DIOMAPPING2, 0x00 );	
      SX1278LoRaSetOpMode( Receiver_mode );
      PA_RXD_OUT();
    }
    else 
    {
      if((RF_EX0_STATUS&0x04)==0x04) //CAD完成
      {  
        if((RF_EX0_STATUS&0x01)==0x01)
        {     //表示CAD 检测到扩频信号 模块进入了接收状态来接收数据
          SX1278LoRaSetOpMode( Stdby_mode );
          SX1278WriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);  //打开发送中断
          SX1278WriteBuffer(REG_LR_HOPPERIOD,   PACKET_MIAX_Value );
          SX1278WriteBuffer( REG_LR_DIOMAPPING1, 0X02 );
          SX1278WriteBuffer( REG_LR_DIOMAPPING2, 0x00 );	
          SX1278LoRaSetOpMode( Receiver_mode );
          PA_RXD_OUT();
        }
        else
        {                           // 没检测到
          SX1278LoRaSetOpMode( Stdby_mode );
          SX1278WriteBuffer(REG_LR_IRQFLAGSMASK,  IRQN_SEELP_Value);  //打开发送中断
          SX1278WriteBuffer( REG_LR_DIOMAPPING1, 0X00 );
          SX1278WriteBuffer( REG_LR_DIOMAPPING2, 0X00 );	
          SX1278LoRaSetOpMode( Sleep_mode );
          //PA_SEELP_OUT();
        }
      }
      else
      {
      SX1278LoRaSetOpMode( Stdby_mode );
      SX1278WriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);  //打开发送中断
      SX1278WriteBuffer(REG_LR_HOPPERIOD,   PACKET_MIAX_Value );
      SX1278WriteBuffer( REG_LR_DIOMAPPING1, 0X02 );
      SX1278WriteBuffer( REG_LR_DIOMAPPING2, 0x00 );	
      SX1278LoRaSetOpMode( Receiver_mode );
      PA_RXD_OUT();
     }
    }
  }
  SX1278WriteBuffer( REG_LR_IRQFLAGS, 0xff  );
}
/**************************************************
 * 函数名称：void Lora_Init(void)
 * 函数介绍：LORA模块整体初始化
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/
void Lora_Init(void)
{
   SX1278_GPIO();
   SX1278LORA_INT();
}
/**************************************************
 * 函数名称：void Process_Lora_Send(uint8_t* data ,uint8_t datalen)
 * 函数介绍：LORA模块发送数据包处理
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/
void Process_Lora_Send(uint8_t* data ,uint8_t datalen)
{
    static uint8_t send_cnt = 0;
    uint16_t sendi = 0;
    FUN_RF_SENDPACKET(data,datalen); //发射数据
    send_cnt++; //发射次数加1
    while((SX1278ReadBuffer(REG_LR_IRQFLAGS) & 0x08) != 0x08 && (sendi < 1250))//限时等待发送完成
    {
       sendi++;
       delay_ms(1);
    }
    if(sendi < 1250)
    {
       send_cnt = 0;
       eDriverState = GotoHalt_State;
    }
    if(send_cnt > 3)  //超过3次失败
    {
       send_cnt = 0;
       eDriverState = GotoHalt_State;
    }
}
/**************************************************
 * 函数名称：void Process_Lora_Recv(void)
 * 函数介绍：LORA模块接收数据包处理
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/
void Process_Lora_Recv(void)
{
  RF_EX0_STATUS=SX1278ReadBuffer( REG_LR_IRQFLAGS ); //读0x12状态寄存器  
  if(RF_EX0_STATUS)
  {
    SX1278_Interupt();//数据处理
  }
  if((RF_EX0_STATUS&0x40)==0x40)  //接收完成
  {
  
  }
}
/**************************************************
 * 函数名称：void Lora_Interaction_Protocol(void)
 * 函数介绍：数据协议封装
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/
void Lora_Interaction_Protocol(void)
{
   uint8_t datalen = 0;
   uint8_t device_addr_msb = DEVICE_ADDR_MSB;
   uint8_t device_addr_lsb = DEVICE_ADDR_LSB;
   int16_t data = 0;
   uint8_t crc8 = 0;
   Lora_senddata[0] = 0x56;
   Lora_senddata[1] = 0; //0表示传感器设备发送
   Lora_senddata[2] = device_type;
   Lora_senddata[3] = device_addr_msb;
   Lora_senddata[4] = device_addr_lsb;
   Lora_senddata[5] = vbat;
   if(device_type == 0x10) //DS18B20温度值
   { 
      datalen = 2;
      
      Lora_senddata[6] = datalen;
      
      data = (int) (g_ds18b20_temperature * 10);
      
      Lora_senddata[7] = data >> 8; 
      
      Lora_senddata[8] = data & 0x00ff;
      
   }
   if(device_type == 0x20) //SHT2X温湿度值
   {
      datalen = 4;
      
      Lora_senddata[6] = datalen;
      
      data = (int) (g_sht2x_temperature * 10);
      
      Lora_senddata[7] = data >> 8; 
      
      Lora_senddata[8] = data & 0x00ff;
   
      data = (int) (g_sht2x_humidity * 10);
      
      Lora_senddata[9] = data >> 8; 
      
      Lora_senddata[10] = data & 0x00ff;
   
   }
   if(device_type == 0x30) //GY3X光照度值
   {
      datalen = 2;
      
      Lora_senddata[6] = datalen;
      
      Lora_senddata[7] = (uint16_t) g_gy3x_light >> 8; 
      
      Lora_senddata[8] = (uint16_t) g_gy3x_light & 0x00ff;
   
   } 
   if(device_type == 0x60) //光照温湿度
   {
      datalen = 6;
      
      Lora_senddata[6] = datalen;
      
      data = (int) (g_sht2x_temperature * 10);
      
      Lora_senddata[7] = data >> 8; 
      
      Lora_senddata[8] = data & 0x00ff;
   
      data = (int) (g_sht2x_humidity * 10);
      
      Lora_senddata[9] = data >> 8; 
      
      Lora_senddata[10] = data & 0x00ff;
      
      Lora_senddata[11] = (uint16_t) g_gy3x_light >> 8; 
      
      Lora_senddata[12] = (uint16_t) g_gy3x_light & 0x00ff;
   }
   if(device_type == 0x70) //氨气浓度
   {
      datalen = 2;
    
      Lora_senddata[6] = datalen;
      
      Lora_senddata[7] = acData>> 8; 
      
      Lora_senddata[8] = acData & 0x00ff;
   
   }
   if(device_type == 0x80) //温湿度氨气
   {
      datalen = 6;
      
      Lora_senddata[6] = datalen;
      
      data = (int) (g_sht2x_temperature * 10);
      
      Lora_senddata[7] = data >> 8; 
      
      Lora_senddata[8] = data & 0x00ff;
   
      data = (int) (g_sht2x_humidity * 10);
      
      Lora_senddata[9] = data >> 8; 
      
      Lora_senddata[10] = data & 0x00ff;
      
      Lora_senddata[11] = (uint16_t) acData >> 8; 
      
      Lora_senddata[12] = (uint16_t) acData & 0x00ff;
   }
   lora_slen = datalen + 8;
//   crc8 = calc_crc8_x8_x2_x_1(crc8,Lora_senddata,lora_slen);
   crc8 = calc_check_sum8(Lora_senddata, (lora_slen - 1));
   Lora_senddata[lora_slen - 1] = crc8;
 
}
 