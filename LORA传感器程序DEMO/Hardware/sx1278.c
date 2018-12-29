/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * �ļ����ƣ�sx1278.c
 * �ļ���ʶ��
 * ժ    Ҫ��LORAоƬsx1278����
 * 
 * ��ǰ�汾��V1.0.0
 * ��    �ߣ�yjd
 * ������ڣ�2017-10-12
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

uint8_t   SpreadingFactor = 11;    //7-12,��Ƶ����ѡС������ʱ����һЩ

uint8_t   CodingRate = 2;        //1-4

uint8_t   Bw_Frequency = 7;      //6-9

uint8_t   powerValue = 7;   //��������

uint8_t   power_data[8]={0X80,0X80,0X80,0X83,0X86,0x89,0x8c,0x8f};

uint8_t   RF_EX0_STATUS; //��ȡ����״ֵ̬

uint8_t   CRC_Value;  

uint8_t   SX1278_RLEN; //���յ����ֽ���

uint8_t   lora_slen; //LORA�����ֽ���

uint8_t   Lora_recvdata[RECVLEN] = {0}; //lora��������

uint8_t   Lora_senddata[SENDLEN] = {0};//lora��������

uint8_t   device_addr_msb = 0; //�ӻ���ַ���ֽ�

uint8_t   device_addr_lsb = 0; //�ӻ���ַ���ֽ�

uint8_t   vbat = 0; //�ӻ���ص�ѹֵ

uint8_t   device_type = DEVICE_TYPE ; //�����豸����
/**************************************************
 * �������ƣ�void SX1278_GPIO(void)
 * �������ܣ�����SX1278��������ų�ʼ��
 * ���������
 * ���������
 * ����ֵ  ��
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
 * �������ƣ�void FSK_SEND_PACKET(void)
 * �������ܣ�FSK�������ݰ�
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
void FSK_SEND_PACKET(void)
{
  SX1278LoRaSetOpMode( Stdby_mode );
  SX1278WriteBuffer(REG_LR_DIOMAPPING1,0x01);
  SX1278WriteBuffer(REG_LR_DIOMAPPING2,0x20);
}
/**************************************************
 * �������ƣ�void SX1278Reset(void)
 * �������ܣ�SX1278��λ
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/	
void SX1278Reset(void)
{
   RF_REST_L;	
   delay_ms(200);
   RF_REST_H;
   delay_ms(500);
}
/**************************************************
 * �������ƣ�void SX1278_INIT(void)
 * �������ܣ�SX1278��SPI������ų�ʼ��
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/	
void SX1278_INIT(void)
{
   RF_CKL_L;
   RF_CE_H;
   RF_SDI_H;
}
/**************************************************
 * �������ƣ�void RF_SPI_MasterIO(uint8_t out)
 * �������ܣ���SX1278д�ֽ�
 * ���������uint8_t out ��д���ֽ�
 * ���������
 * ����ֵ  ��
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
 * �������ƣ�uint8_t RF_SPI_READ_BYTE()
 * �������ܣ���SX1278���ֽ�
 * ���������
 * ���������
 * ����ֵ  ��
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
 * �������ƣ�void SX1278WriteBuffer( uint8_t addr, uint8_t buffer)
 * �������ܣ���SX1278ĳ���Ĵ�����ַд�ֽ�
 * ���������uint8_t addr ���Ĵ�����ַ �� uint8_t buffer��д���ֽ�
 * ���������
 * ����ֵ  ��
 **************************************************/	  
void SX1278WriteBuffer( uint8_t addr, uint8_t buffer)
{ 
  RF_CE_L; //NSS = 0;
  RF_SPI_MasterIO( addr | 0x80 );
  RF_SPI_MasterIO( buffer);
  RF_CE_H; //NSS = 1;
}
/**************************************************
 * �������ƣ�uint8_t SX1278ReadBuffer(uint8_t addr)
 * �������ܣ���SX1278ĳ���Ĵ�����ַ���ֽ�
 * ���������uint8_t addr ���Ĵ�����ַ
 * ���������uint8_t Value���������ֽ�
 * ����ֵ  ��
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
 * �������ƣ�void SX1278LoRaSetOpMode( RFMode_SET opMode )
 * �������ܣ�����SX1278�Ĳ���ģʽ
 * ���������RFMode_SET opMode ������ģʽ
 * ���������
 * ����ֵ  ��
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
 * �������ƣ�void SX1278LoRaFsk( Debugging_fsk_ook opMode )
 * �������ܣ�����SX1278�ĵ��Ʒ�ʽ
 * ���������Debugging_fsk_ook opMode  �����Ʒ�ʽ
 * ���������
 * ����ֵ  ��
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
 * �������ƣ�void SX1278LoRaSetRFFrequency(void)
 * �������ܣ�����SX1278��Ƶ��
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/	
void SX1278LoRaSetRFFrequency(void)
{
  SX1278WriteBuffer( REG_LR_FRFMSB, Frequency[0]);
  SX1278WriteBuffer( REG_LR_FRFMID, Frequency[1]);
  SX1278WriteBuffer( REG_LR_FRFLSB, Frequency[2]);
}
/**************************************************
 * �������ƣ�void SX1278LoRaSetRFPower( uint8_t power )
 * �������ܣ�����SX1278��RF����
 * ���������uint8_t power������ֵ
 * ���������
 * ����ֵ  ��
 **************************************************/		
void SX1278LoRaSetRFPower( uint8_t power )
{
  SX1278WriteBuffer( REG_LR_PADAC, 0x87);
  SX1278WriteBuffer( REG_LR_PACONFIG,  power_data[power] );
}
/**************************************************
 * �������ƣ�void SX1278LoRaSetSpreadingFactor( uint8_t factor )
 * �������ܣ�����SX1278����Ƶ����
 * ���������uint8_t factor ����Ƶ����
 * ���������
 * ����ֵ  ��
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
 * �������ƣ�void SX1278LoRaSetNbTrigPeaks( uint8_t value )
 * �������ܣ�����SX1278����Ƶ�������
 * ��������� uint8_t value������ֵ
 * ���������
 * ����ֵ  ��
 **************************************************/	
void SX1278LoRaSetNbTrigPeaks( uint8_t value )
{
  uint8_t RECVER_DAT;
  RECVER_DAT = SX1278ReadBuffer( 0x31);
  RECVER_DAT = ( RECVER_DAT & 0xF8 ) | value;
  SX1278WriteBuffer( 0x31, RECVER_DAT );
}
/**************************************************
 * �������ƣ�void SX1278LoRaSetErrorCoding( uint8_t value )
 * �������ܣ�����SX1278����Ч���ݱ�
 * ��������� uint8_t value������ֵ
 * ���������
 * ����ֵ  ��
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
 * �������ƣ�void SX1278LoRaSetPacketCrcOn( bool enable )
 * �������ܣ�����SX1278��CRCУ��
 * ���������bool enable��ʹ��
 * ���������
 * ����ֵ  ��
 **************************************************/	
void SX1278LoRaSetPacketCrcOn( bool enable )
{	
  uint8_t RECVER_DAT;
  RECVER_DAT= SX1278ReadBuffer( REG_LR_MODEMCONFIG2);
  RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) | ( enable << 2 );
  SX1278WriteBuffer( REG_LR_MODEMCONFIG2, RECVER_DAT );
}
/**************************************************
 * �������ƣ�void SX1278LoRaSetSignalBandwidth( uint8_t bw )
 * �������ܣ�����SX1278����Ƶ����
 * ��������� uint8_t bw������ֵ
 * ���������
 * ����ֵ  ��
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
 * �������ƣ�void SX1278LoRaSetImplicitHeaderOn( bool enable )
 * �������ܣ�����SX1278��ͬ��ͷģʽ
 * ���������bool enable ��ʹ��
 * ���������
 * ����ֵ  ��
 **************************************************/		
void SX1278LoRaSetImplicitHeaderOn( bool enable )
{
  uint8_t RECVER_DAT;
  RECVER_DAT=SX1278ReadBuffer( REG_LR_MODEMCONFIG1 );
  RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) | ( enable );
  SX1278WriteBuffer( REG_LR_MODEMCONFIG1, RECVER_DAT );
}
/**************************************************
 * �������ƣ�void SX1278LoRaSetSymbTimeout( uint16_t value )
 * �������ܣ�����SX1278�ı�־��ʱֵ
 * ���������uint16_t value ������ֵ
 * ���������
 * ����ֵ  ��
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
 * �������ƣ�void SX1278LoRaSetPayloadLength( uint8_t value )
 * �������ܣ�����SX1278��������Ч����
 * ���������uint8_t value ������ֵ
 * ���������
 * ����ֵ  ��
 **************************************************/		
void SX1278LoRaSetPayloadLength( uint8_t value )
{
  SX1278WriteBuffer( REG_LR_PAYLOADLENGTH, value );
} 
/**************************************************
 * �������ƣ�void SX1278LoRaSetPreamLength( uint16_t value )
 * �������ܣ�����SX1278��������ͷ����
 * ���������uint16_t value ������ֵ
 * ���������
 * ����ֵ  ��
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
 * �������ƣ�void SX1278LoRaSetMobileNode( bool enable )
 * �������ܣ�����SX1278�ĵ������Ż�
 * ���������bool enable ��ʹ��
 * ���������
 * ����ֵ  ��
 **************************************************/		
void SX1278LoRaSetMobileNode( bool enable )
{	 
  uint8_t RECVER_DAT;
  RECVER_DAT=SX1278ReadBuffer( REG_LR_MODEMCONFIG3 );
  RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG3_MOBILE_NODE_MASK ) | ( enable << 3 );
  SX1278WriteBuffer( REG_LR_MODEMCONFIG3, RECVER_DAT );
}

/**************************************************
 * �������ƣ�void SX1278LORA_INT(void)
 * �������ܣ�SX1278��ģʽ��ʼ��
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/	
void SX1278LORA_INT(void)
{
  RFPOWER_EN;     //��Դ����
  SX1278Reset();
  SX1278_INIT();
  SX1278LoRaSetOpMode(Sleep_mode);  //����˯��ģʽ
  SX1278LoRaFsk(LORA_mode);	   // ������Ƶģʽ
  SX1278LoRaSetOpMode(Stdby_mode);  // ����Ϊ��ͨģʽ
  SX1278WriteBuffer( REG_LR_DIOMAPPING1,GPIO_VARE_1);
  SX1278WriteBuffer( REG_LR_DIOMAPPING2,GPIO_VARE_2);
  SX1278LoRaSetRFFrequency();
  SX1278LoRaSetRFPower(powerValue);
  SX1278LoRaSetSpreadingFactor(SpreadingFactor);			 // ��Ƶ��������
  SX1278LoRaSetErrorCoding(CodingRate);				//��Ч���ݱ�
  SX1278LoRaSetPacketCrcOn( TRUE);			 //CRC У���
  SX1278LoRaSetSignalBandwidth( Bw_Frequency );			//������Ƶ����
  SX1278LoRaSetImplicitHeaderOn( FALSE);		 //ͬ��ͷ������ģʽ
  SX1278LoRaSetPayloadLength( 0xff);
  SX1278LoRaSetSymbTimeout( 0x3FF );
  SX1278LoRaSetMobileNode(TRUE ); 			// �����ݵ��Ż�
}
/**************************************************
 * �������ƣ�void FUN_RF_SENDPACKET(uint8_t *RF_TRAN_P,uint8_t LEN)
 * �������ܣ�SX1278��������ģʽ�����������ݰ�
 * ���������uint8_t *RF_TRAN_P������������ʼ��ַ��uint8_t LEN�����ݳ���
 * ���������
 * ����ֵ  ��
 **************************************************/		
void FUN_RF_SENDPACKET(uint8_t *RF_TRAN_P,uint8_t LEN)
{
  uint8_t ASM_i;
  PA_TXD_OUT();
  SX1278LoRaSetOpMode( Stdby_mode );
  SX1278WriteBuffer( REG_LR_HOPPERIOD, 0 );	//����Ƶ������
  SX1278WriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_TXD_Value);	//�򿪷����ж�
  SX1278WriteBuffer( REG_LR_PAYLOADLENGTH, LEN);	 //������ݰ�
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
 * �������ƣ�void RF_RECEIVE (void)
 * �������ܣ���������ģʽ
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
void RF_RECEIVE (void)
{
  SX1278LoRaSetOpMode( Stdby_mode );
  SX1278WriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);  //�򿪷����ж�
  SX1278WriteBuffer(REG_LR_HOPPERIOD, PACKET_MIAX_Value );
  SX1278WriteBuffer( REG_LR_DIOMAPPING1, 0X00 );
  SX1278WriteBuffer( REG_LR_DIOMAPPING2, 0X00 );	
  SX1278LoRaSetOpMode( Receiver_mode );
  PA_RXD_OUT();
}
/**************************************************
 * �������ƣ�void RF_CAD_RECEIVE (void)
 * �������ܣ�����CADģʽ
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/	
void RF_CAD_RECEIVE (void)
{
  SX1278LoRaSetOpMode( Stdby_mode );
  SX1278WriteBuffer(REG_LR_IRQFLAGSMASK,  IRQN_CAD_Value);	//��CAD�ж�
  SX1278WriteBuffer( REG_LR_DIOMAPPING1, 0X80 );
  SX1278WriteBuffer( REG_LR_DIOMAPPING2, 0X00 );	
  SX1278LoRaSetOpMode( CAD_mode );
  PA_RXD_OUT();
}
/**************************************************
 * �������ƣ�void RF_SEELP(void)
 * �������ܣ�����˯��ģʽ
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/		
void RF_SEELP(void)
{
  SX1278LoRaSetOpMode( Stdby_mode );
  SX1278WriteBuffer(REG_LR_IRQFLAGSMASK,  IRQN_SEELP_Value);  //��SEELP�ж�
  SX1278WriteBuffer( REG_LR_DIOMAPPING1, 0X00 );
  SX1278WriteBuffer( REG_LR_DIOMAPPING2, 0X00 );	
  SX1278LoRaSetOpMode( Sleep_mode );
  //PA_SEELP_OUT();
}

/**************************************************
 * �������ƣ�void SX1278_Interupt(void)
 * �������ܣ�SX1278�жϴ���
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
void SX1278_Interupt(void)
{
  RF_EX0_STATUS=SX1278ReadBuffer( REG_LR_IRQFLAGS ); 
  if((RF_EX0_STATUS&0x40)==0x40) //�������
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
    SX1278WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value);  //�򿪷����ж�
    SX1278WriteBuffer(REG_LR_HOPPERIOD,    PACKET_MIAX_Value);
    SX1278WriteBuffer( REG_LR_DIOMAPPING1, 0X00 );
    SX1278WriteBuffer( REG_LR_DIOMAPPING2, 0x00 );	
    SX1278LoRaSetOpMode( Receiver_mode );
    PA_RXD_OUT();
//    flag.Abit.rfModule_ReceiveOK = 1;
  }
  else 
  {
    if((RF_EX0_STATUS&0x08)==0x08) //�������
    {
      SX1278LoRaSetOpMode( Stdby_mode );
      SX1278WriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);  //�򿪽����ж�
      SX1278WriteBuffer(REG_LR_HOPPERIOD,   PACKET_MIAX_Value );
      SX1278WriteBuffer( REG_LR_DIOMAPPING1, 0X00 );
      SX1278WriteBuffer( REG_LR_DIOMAPPING2, 0x00 );	
      SX1278LoRaSetOpMode( Receiver_mode );
      PA_RXD_OUT();
    }
    else 
    {
      if((RF_EX0_STATUS&0x04)==0x04) //CAD���
      {  
        if((RF_EX0_STATUS&0x01)==0x01)
        {     //��ʾCAD ��⵽��Ƶ�ź� ģ������˽���״̬����������
          SX1278LoRaSetOpMode( Stdby_mode );
          SX1278WriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);  //�򿪷����ж�
          SX1278WriteBuffer(REG_LR_HOPPERIOD,   PACKET_MIAX_Value );
          SX1278WriteBuffer( REG_LR_DIOMAPPING1, 0X02 );
          SX1278WriteBuffer( REG_LR_DIOMAPPING2, 0x00 );	
          SX1278LoRaSetOpMode( Receiver_mode );
          PA_RXD_OUT();
        }
        else
        {                           // û��⵽
          SX1278LoRaSetOpMode( Stdby_mode );
          SX1278WriteBuffer(REG_LR_IRQFLAGSMASK,  IRQN_SEELP_Value);  //�򿪷����ж�
          SX1278WriteBuffer( REG_LR_DIOMAPPING1, 0X00 );
          SX1278WriteBuffer( REG_LR_DIOMAPPING2, 0X00 );	
          SX1278LoRaSetOpMode( Sleep_mode );
          //PA_SEELP_OUT();
        }
      }
      else
      {
      SX1278LoRaSetOpMode( Stdby_mode );
      SX1278WriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);  //�򿪷����ж�
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
 * �������ƣ�void Lora_Init(void)
 * �������ܣ�LORAģ�������ʼ��
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
void Lora_Init(void)
{
   SX1278_GPIO();
   SX1278LORA_INT();
}
/**************************************************
 * �������ƣ�void Process_Lora_Send(uint8_t* data ,uint8_t datalen)
 * �������ܣ�LORAģ�鷢�����ݰ�����
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
void Process_Lora_Send(uint8_t* data ,uint8_t datalen)
{
    static uint8_t send_cnt = 0;
    uint16_t sendi = 0;
    FUN_RF_SENDPACKET(data,datalen); //��������
    send_cnt++; //���������1
    while((SX1278ReadBuffer(REG_LR_IRQFLAGS) & 0x08) != 0x08 && (sendi < 1250))//��ʱ�ȴ��������
    {
       sendi++;
       delay_ms(1);
    }
    if(sendi < 1250)
    {
       send_cnt = 0;
       eDriverState = GotoHalt_State;
    }
    if(send_cnt > 3)  //����3��ʧ��
    {
       send_cnt = 0;
       eDriverState = GotoHalt_State;
    }
}
/**************************************************
 * �������ƣ�void Process_Lora_Recv(void)
 * �������ܣ�LORAģ��������ݰ�����
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
void Process_Lora_Recv(void)
{
  RF_EX0_STATUS=SX1278ReadBuffer( REG_LR_IRQFLAGS ); //��0x12״̬�Ĵ���  
  if(RF_EX0_STATUS)
  {
    SX1278_Interupt();//���ݴ���
  }
  if((RF_EX0_STATUS&0x40)==0x40)  //�������
  {
  
  }
}
/**************************************************
 * �������ƣ�void Lora_Interaction_Protocol(void)
 * �������ܣ�����Э���װ
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
void Lora_Interaction_Protocol(void)
{
   uint8_t datalen = 0;
   uint8_t device_addr_msb = DEVICE_ADDR_MSB;
   uint8_t device_addr_lsb = DEVICE_ADDR_LSB;
   int16_t data = 0;
   uint8_t crc8 = 0;
   Lora_senddata[0] = 0x56;
   Lora_senddata[1] = 0; //0��ʾ�������豸����
   Lora_senddata[2] = device_type;
   Lora_senddata[3] = device_addr_msb;
   Lora_senddata[4] = device_addr_lsb;
   Lora_senddata[5] = vbat;
   if(device_type == 0x10) //DS18B20�¶�ֵ
   { 
      datalen = 2;
      
      Lora_senddata[6] = datalen;
      
      data = (int) (g_ds18b20_temperature * 10);
      
      Lora_senddata[7] = data >> 8; 
      
      Lora_senddata[8] = data & 0x00ff;
      
   }
   if(device_type == 0x20) //SHT2X��ʪ��ֵ
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
   if(device_type == 0x30) //GY3X���ն�ֵ
   {
      datalen = 2;
      
      Lora_senddata[6] = datalen;
      
      Lora_senddata[7] = (uint16_t) g_gy3x_light >> 8; 
      
      Lora_senddata[8] = (uint16_t) g_gy3x_light & 0x00ff;
   
   } 
   if(device_type == 0x60) //������ʪ��
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
   if(device_type == 0x70) //����Ũ��
   {
      datalen = 2;
    
      Lora_senddata[6] = datalen;
      
      Lora_senddata[7] = acData>> 8; 
      
      Lora_senddata[8] = acData & 0x00ff;
   
   }
   if(device_type == 0x80) //��ʪ�Ȱ���
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
 