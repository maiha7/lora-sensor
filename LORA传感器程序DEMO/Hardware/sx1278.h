
#ifndef __SX1278__H__
#define __SX1278__H__

#include "stm8l15x.h"
#include "stm8l15x_gpio.h"

#define  DS18B20       0x10 
#define  SHT2X         0x20 
#define  GY3X          0x30
#define  LIQUID        0x40 
#define  FLOW          0x50
#define  SHT2XANDGY3X  0x60 
#define  GAS           0x70
#define  GASSHT2X      0x80  //温湿度、氨气
//发送模块的设备类型，温度10H、温湿度20H、光照度30H、液位40H、流量50H、光照温湿度60H、气体70H、
#define  DEVICE_TYPE      SHT2X    //定义设备类型
#define  DEVICE_ADDR_MSB  0x00  //定义设备ID高位
#define  DEVICE_ADDR_LSB  0x02  //定义设备ID低位
extern uint8_t   lora_slen;
extern uint8_t   device_type;
#define  RECVLEN   64
#define  SENDLEN   20

extern uint8_t   Lora_senddata[SENDLEN];
extern uint8_t   Lora_recvdata[RECVLEN];

#define  TXEN_PORT                          GPIOB                                           
#define  TXEN_PIN                           GPIO_Pin_2
#define  RXEN_PORT                          GPIOB
#define  RXEN_PIN                           GPIO_Pin_3
#define  RESET_PORT                         GPIOD
#define  RESET_PIN                          GPIO_Pin_4
#define  CCGDO0_PORT                        GPIOB
#define  CCGDO0_PIN                         GPIO_Pin_1
#define  CCGDO2_PORT                        GPIOB
#define  CCGDO2_PIN                         GPIO_Pin_0
#define  RFCE_PORT                          GPIOB
#define  RFCE_PIN                           GPIO_Pin_4
#define  RFCLK_PORT                         GPIOB
#define  RFCLK_PIN                          GPIO_Pin_5
#define  RFMOSI_PORT                        GPIOB
#define  RFMOSI_PIN                         GPIO_Pin_6
#define  RFMISO_PORT                        GPIOB
#define  RFMISO_PIN                         GPIO_Pin_7
#define  RFPWCTRL_PORT                      GPIOD
#define  RFPWCTRL_PIN                       GPIO_Pin_5


#define  SX1278_SDO                         GPIO_ReadInputDataBit(RFMISO_PORT,RFMISO_PIN)

#define  RF_REST_L			    GPIO_ResetBits(RESET_PORT, RESET_PIN)	  /* MMC CE = L */
#define  RF_REST_H			    GPIO_SetBits(RESET_PORT, RESET_PIN)	  /* MMC CE = H */

#define  RF_CE_L                            GPIO_ResetBits(RFCE_PORT, RFCE_PIN)      /* MMC CE = L */
#define  RF_CE_H                            GPIO_SetBits(RFCE_PORT, RFCE_PIN)         /* MMC CE = H */

#define  RF_CKL_L                           GPIO_ResetBits(RFCLK_PORT, RFCLK_PIN)     /* MMC CKL = L */
#define  RF_CKL_H                           GPIO_SetBits(RFCLK_PORT, RFCLK_PIN)      /* MMC CKL = H */

#define  RF_SDI_L                           GPIO_ResetBits(RFMOSI_PORT, RFMOSI_PIN)      /* MMC CKL = L */
#define  RF_SDI_H                           GPIO_SetBits(RFMOSI_PORT, RFMOSI_PIN)         /* MMC CKL = H */

#define  PA_TXD_OUT()                       GPIO_SetBits(TXEN_PORT, TXEN_PIN);\
                                            GPIO_ResetBits(RXEN_PORT, RXEN_PIN)

#define  PA_RXD_OUT()                       GPIO_SetBits(RXEN_PORT, RXEN_PIN );\
                                            GPIO_ResetBits(TXEN_PORT, TXEN_PIN)  
                                              
#define  RFPOWER_EN                         GPIO_SetBits(RFPWCTRL_PORT, RFPWCTRL_PIN) 
#define  RFPOWER_DIS                        GPIO_ResetBits(RFPWCTRL_PORT, RFPWCTRL_PIN)  


/*!
  SX1278 Internal registers Address
  */
#define REG_LR_FIFO                                 0x00 
 // Common settings
#define REG_LR_OPMODE                               0x01 
#define REG_LR_BANDSETTING                          0x04
#define REG_LR_FRFMSB                               0x06 
#define REG_LR_FRFMID                               0x07
#define REG_LR_FRFLSB                               0x08 
 // Tx settings
#define REG_LR_PACONFIG                             0x09 
#define REG_LR_PARAMP                               0x0A 
#define REG_LR_OCP                                  0x0B 
 // Rx settings
#define REG_LR_LNA                                  0x0C 
 // LoRa registers
#define REG_LR_FIFOADDRPTR                          0x0D 
#define REG_LR_FIFOTXBASEADDR                       0x0E 
#define REG_LR_FIFORXBASEADDR                       0x0F 
#define REG_LR_FIFORXCURRENTADDR                    0x10 
#define REG_LR_IRQFLAGSMASK                         0x11 
#define REG_LR_IRQFLAGS                             0x12 
#define REG_LR_NBRXBYTES                            0x13 
#define REG_LR_RXHEADERCNTVALUEMSB                  0x14 
#define REG_LR_RXHEADERCNTVALUELSB                  0x15 
#define REG_LR_RXPACKETCNTVALUEMSB                  0x16 
#define REG_LR_RXPACKETCNTVALUELSB                  0x17 
#define REG_LR_MODEMSTAT                            0x18 
#define REG_LR_PKTSNRVALUE                          0x19 
#define REG_LR_PKTRSSIVALUE                         0x1A 
#define REG_LR_RSSIVALUE                            0x1B 
#define REG_LR_HOPCHANNEL                           0x1C 
#define REG_LR_MODEMCONFIG1                         0x1D 
#define REG_LR_MODEMCONFIG2                         0x1E 
#define REG_LR_SYMBTIMEOUTLSB                       0x1F 
#define REG_LR_PREAMBLEMSB                          0x20 
#define REG_LR_PREAMBLELSB                          0x21 
#define REG_LR_PAYLOADLENGTH                        0x22 
#define REG_LR_PAYLOADMAXLENGTH                     0x23 
#define REG_LR_HOPPERIOD                            0x24 
#define REG_LR_FIFORXBYTEADDR                       0x25
#define REG_LR_MODEMCONFIG3                         0x26
 // end of documented register in datasheet
 // I/O settings
#define REG_LR_DIOMAPPING1                          0x40
#define REG_LR_DIOMAPPING2                          0x41
 // Version
#define REG_LR_VERSION                              0x42
 // Additional settings
#define REG_LR_PLLHOP                               0x44
#define REG_LR_TCXO                                 0x4B
#define REG_LR_PADAC                                0x4D
#define REG_LR_FORMERTEMP                           0x5B
#define REG_LR_BITRATEFRAC                          0x5D
#define REG_LR_AGCREF                               0x61
#define REG_LR_AGCTHRESH1                           0x62
#define REG_LR_AGCTHRESH2                           0x63
#define REG_LR_AGCTHRESH3                           0x64


#define GPIO_VARE_1                                  0x00
#define GPIO_VARE_2                                  0x00
#define RFLR_MODEMCONFIG2_SF_MASK                    0x0f
#define RFLR_MODEMCONFIG1_CODINGRATE_MASK            0xF1 
#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK          0xFB 
#define RFLR_MODEMCONFIG1_BW_MASK                    0x0F 
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK        0xFE 
#define RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK        0xfc
#define RFLR_MODEMCONFIG3_MOBILE_NODE_MASK           0xF7 

#define TIME_OUT_INT                                 0x80 
#define PACKET_RECVER_INT                            0x40 
#define CRC_ERROR_INT                                0x20 
#define RECVER_HEAR_INT                              0x10 
#define FIFO_SEND_OVER                               0x08 
#define RFLR_IRQFLAGS_CAD                            0x04 
#define RFLR_IRQFLAGS_FHSS                           0x02 
#define RFLR_IRQFLAGS_CADD                           0x01 

#define IRQN_TXD_Value                               0xF7
#define IRQN_RXD_Value                               0x9F
#define IRQN_CAD_Value                               0xFA
#define IRQN_SEELP_Value                             0xFF

#define PACKET_MIAX_Value                            0xff

                                              
typedef enum 
{
  Sleep_mode				= (uint8_t)0x00, 
  Stdby_mode				= (uint8_t)0x01, 
  TX_mode 				= (uint8_t)0x02,
  Transmitter_mode		        = (uint8_t)0x03,
  RF_mode 				= (uint8_t)0x04,
  Receiver_mode			        = (uint8_t)0x05,
  receive_single			= (uint8_t)0x06,
  CAD_mode				= (uint8_t)0x07,
} RFMode_SET;

typedef enum 
{
  FSK_mode               = (uint8_t)0x00, 
  LORA_mode              = (uint8_t)0x80, 
} Debugging_fsk_ook;


void    PA_SEELP_OUT(void);
void    Delay1s(uint16_t ii);
void    SX1278Reset( void );
void    SX1278_INIT(void);
void    RF_SPI_MasterIO(uint8_t out);
uint8_t RF_SPI_READ_BYTE();
void    SX1278WriteBuffer( uint8_t addr, uint8_t buffer);
uint8_t SX1278ReadBuffer( uint8_t addr);
void SX1278LoRaSetOpMode( RFMode_SET opMode );
void SX1278LoRaFsk( Debugging_fsk_ook opMode );
void SX1278LoRaSetRFFrequency(void);
void SX1278LoRaSetRFPower( uint8_t power );
void SX1278LoRaSetSpreadingFactor( uint8_t factor );
void SX1278LoRaSetNbTrigPeaks( uint8_t value );
void SX1278LoRaSetErrorCoding( uint8_t value );
void SX1278LoRaSetPacketCrcOn( bool enable );
void SX1278LoRaSetSignalBandwidth( uint8_t bw );
void SX1278LoRaSetImplicitHeaderOn( bool enable );
void SX1278LoRaSetSymbTimeout( uint16_t value );
void SX1278LoRaSetPayloadLength( uint8_t value );
void SX1278LoRaSetPreamLength( uint16_t value );
void SX1278LoRaSetMobileNode( bool enable );
void SLEEPLORA_INT(void);
void FSK_SEND_PACKET(void);
void FUN_RF_SENDPACKET(uint8_t *RF_TRAN_P,uint8_t LEN);
void RF_RECEIVE (void);
void RF_CAD_RECEIVE (void);
void RF_SEELP(void);
void SX1278LORA_INT(void);
void SX1278_Interupt(void);
void SX1278_GPIO(void);
void Lora_Init(void);
void Process_Lora_Send(uint8_t* data ,uint8_t datalen);
void Process_Lora_Recv(void);
void Lora_Interaction_Protocol(void);

#endif