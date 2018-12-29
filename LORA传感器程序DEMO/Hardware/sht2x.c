/**************************************************
* Copyright (c) 
* All rights reserved.
* 
* �ļ����ƣ�sht2x.c
* �ļ���ʶ��
* ժ    Ҫ����ʪ�ȴ�������������
* 
* ��ǰ�汾��V1.0.0
* ��    �ߣ�yjd
* ������ڣ�2017-09-30
**************************************************/
#include "led.h"
#include "exti.h"
#include "stm8l15x.h"
#include "stdio.h"
#include "sht2x.h"
#include "apply.h"

float g_sht2x_temperature = 0;  //sht2x �¶�ֵ
float g_sht2x_humidity = 0; //sht2x ʪ��ֵ
ESht2xsampleState  eSht2xsampleState;
/**************************************************
* �������ƣ�void SHT2x_Init(void)
* �������ܣ�SHT2X��ʼ��
* ���������
* ���������
* ����ֵ  ��
**************************************************/
void SHT2x_Init(void)
{
    GPIO_Init(SHT2X_SCL_PORT,SHT2X_SCL_PIN, GPIO_Mode_Out_PP_High_Slow);//��ʼ��SCL�˿�
    GPIO_Init(SHT2X_SDA_PORT,SHT2X_SDA_PIN, GPIO_Mode_Out_PP_High_Slow);//��ʼ��SDA�˿�
    //    delay_us(80);
    //    SHT2x_SoftReset();
}

/**************************************************
* �������ƣ�void SHT2x_SoftReset(void)
* �������ܣ�SHT2X��λ
* ���������
* ���������
* ����ֵ  ��
**************************************************/
void SHT2x_SoftReset(void)
{
    SHT2x_I2cStartCondition();
    SHT2x_I2cWriteByte(I2C_ADR_W);
    SHT2x_I2cWriteByte(SOFT_RESET);
    SHT2x_I2cStopCondition();
    delay_us(80);
}
/**************************************************
* �������ƣ�void SHT2x_I2cStartCondition(void)
* �������ܣ�SHT2X IIC��ʼ����
* ���������
* ���������
* ����ֵ  ��
**************************************************/
void SHT2x_I2cStartCondition(void)
{
    SHT2x_SCL_OUTPUT();
    SHT2x_SDA_OUTPUT();
    
    SHT2x_SDA_HIGH();
    SHT2x_SCL_HIGH();
    delay_us(500);
    SHT2x_SDA_LOW();
    delay_us(100);
    SHT2x_SCL_LOW();
    delay_us(100);
}
/**************************************************
* �������ƣ�void SHT2x_I2cStopCondition(void)
* �������ܣ�SHT2X IIC��������
* ���������
* ���������
* ����ֵ  ��
**************************************************/
void SHT2x_I2cStopCondition(void)
{
    SHT2x_SCL_OUTPUT();
    SHT2x_SDA_OUTPUT();
    
    SHT2x_SDA_LOW();
    SHT2x_SCL_LOW();
    delay_us(100);
    SHT2x_SCL_HIGH();
    delay_us(100);
    SHT2x_SDA_HIGH();
    delay_us(100);
}
/**************************************************
* �������ƣ�void SHT2x_I2cAcknowledge(void)
* �������ܣ�SHT2X IICӦ��
* ���������
* ���������
* ����ֵ  ��
**************************************************/
void SHT2x_I2cAcknowledge(void)
{
    SHT2x_SCL_OUTPUT();
    SHT2x_SDA_OUTPUT();
    
    SHT2x_SDA_LOW();
    
    SHT2x_SCL_HIGH();
    delay_us(80);
    SHT2x_SCL_LOW();   
    delay_us(80);
}
/**************************************************
* �������ƣ�void SHT2x_I2cNoAcknowledge(void)
* �������ܣ�SHT2X IIC��Ӧ��
* ���������
* ���������
* ����ֵ  ��
**************************************************/
void SHT2x_I2cNoAcknowledge(void)
{
    SHT2x_SCL_OUTPUT();
    SHT2x_SDA_OUTPUT();
    
    SHT2x_SDA_HIGH();
    
    SHT2x_SCL_HIGH();
    delay_us(80);
    SHT2x_SCL_LOW();   
    delay_us(80);
}
/**************************************************
* �������ƣ�uint8_t SHT2x_I2cReadByte(void)
* �������ܣ�SHT2X IIC���ֽ�
* ���������
* ���������
* ����ֵ  ��
**************************************************/
uint8_t SHT2x_I2cReadByte(void)
{
    uint8_t  val = 0;
    
    SHT2x_SCL_OUTPUT();
    SHT2x_SDA_OUTPUT();
    
    SHT2x_SDA_HIGH();
    
    SHT2x_SDA_INPUT();
    
    for(uint8_t i = 0; i < 8; i++)
    {
        val <<= 1; 
        
        SHT2x_SCL_HIGH();   
        delay_us(2);
        if(0x01 == SHT2x_SDA_STATE()) 
        {
            val |= 0x01;
        }
        
        SHT2x_SCL_LOW();  
        delay_us(2); 
    }
    delay_us(2);
    SHT2x_SCL_HIGH();
    delay_us(10);
    SHT2x_SCL_LOW();
    SHT2x_SDA_OUTPUT();
    
    SHT2x_SDA_HIGH();
    delay_us(10);
    return (val);
}
/**************************************************
* �������ƣ�uint8_t SHT2x_I2cWriteByte(uint8_t byte)
* �������ܣ�SHT2X IICд�ֽ�
* ���������
* ���������
* ����ֵ  ��
**************************************************/
uint8_t SHT2x_I2cWriteByte(uint8_t byte)
{
    uint8_t ack;
    uint8_t i;
    SHT2x_SCL_OUTPUT();
    SHT2x_SDA_OUTPUT();
    
    for(i = 0; i < 8; i++)
    {
        if(byte & 0x80) 
        {
            SHT2x_SDA_HIGH();
        }
        else 
        {
            SHT2x_SDA_LOW();
        }
        delay_us(1);
        SHT2x_SCL_HIGH();
        delay_us(5);
        SHT2x_SCL_LOW();   
        delay_us(1);
        
        byte <<= 1;
    }
    
    SHT2x_SDA_INPUT();
    
    SHT2x_SCL_HIGH();
    
    delay_us(5);
    
    if(0x01 == SHT2x_SDA_STATE()) 
    {
        ack = ACK_ERROR;
    }
    else
    {
        ack = ACK_OK;
    }
    
    SHT2x_SCL_LOW();  
    
    delay_us(50);
    
    SHT2x_SDA_OUTPUT();
    
    SHT2x_SDA_HIGH();
    
    return (ack);
}
/**************************************************
* �������ƣ�float SHT2x_MeasureTempHM(void)
* �������ܣ�SHT2X ���¶ȸ�λ
* ���������
* ���������
* ����ֵ  ��
**************************************************/
float SHT2x_MeasureTempHM(void)
{
    float    TEMP;
    uint8_t  tmp1;
    uint8_t  tmp2;
    uint16_t ST;
    
    SHT2x_SCL_OUTPUT();
    
    SHT2x_I2cStartCondition();                            
    SHT2x_I2cWriteByte(I2C_ADR_W);
    SHT2x_I2cWriteByte(TRIG_TEMP_MEASUREMENT_HM);
    
    SHT2x_I2cStartCondition();
    SHT2x_I2cWriteByte(I2C_ADR_R);
    
    SHT2x_SCL_HIGH();
    
    SHT2x_SCL_INPUT();
    
    while(0 == SHT2x_SCL_STATE())
    {
        delay_us(20);
    }
    
    tmp1 = SHT2x_I2cReadByte();
    SHT2x_I2cAcknowledge();
    tmp2 = SHT2x_I2cReadByte();
    SHT2x_I2cNoAcknowledge();
    SHT2x_I2cStopCondition();
    
    ST = (tmp1 << 8) | (tmp2 << 0);
    ST &= ~0x0003;
    TEMP = ((float)ST * 0.00268127) - 46.85;
    
    SHT2x_SCL_OUTPUT();
    
    return (TEMP);	  
}
/**************************************************
* �������ƣ�float SHT2x_MeasureHumiHM(void)
* �������ܣ�SHT2X ��ʪ�ȸ�λ
* ���������
* ���������
* ����ֵ  ��
**************************************************/
//��ȡʪ������
float SHT2x_MeasureHumiHM(void)
{
    float HUMI;
    uint8_t tmp1, tmp2;    
    uint16_t SRH;
    
    
    SHT2x_SCL_OUTPUT();
    
    SHT2x_I2cStartCondition();                               
    SHT2x_I2cWriteByte(I2C_ADR_W);
    SHT2x_I2cWriteByte(TRIG_HUMI_MEASUREMENT_HM);
    
    SHT2x_I2cStartCondition();
    SHT2x_I2cWriteByte(I2C_ADR_R);
    
    SHT2x_SCL_HIGH();
    
    SHT2x_SCL_INPUT();
    
    while(0 == SHT2x_SCL_STATE())
    {
        delay_us(20);
    }
    
    tmp1 = SHT2x_I2cReadByte();
    SHT2x_I2cAcknowledge();
    tmp2 = SHT2x_I2cReadByte();
    SHT2x_I2cNoAcknowledge();
    SHT2x_I2cStopCondition();
    
    SRH = (tmp1 << 8) | (tmp2 << 0);
    SRH &= ~0x0003;
    HUMI = ((float)SRH * 0.00190735) - 6;
    
    SHT2x_SCL_OUTPUT();
    
    return (HUMI);
}

/**************************************************
* �������ƣ�void SHT2x_MeasureTempPoll(int ms)
* �������ܣ�SHT2X ���¶ȵ�λ
* ���������
* ���������
* ����ֵ  ��
**************************************************/
//��ȡ�¶�����
void SHT2x_MeasureTempPoll(int ms)
{
    uint8_t  ack,tmp1,tmp2;
    uint16_t ST;
    static uint8_t ack_err = 0;
    static uint16_t sht2xtemp_sample_time = 0;
    switch(eSht2xsampleState)  
    {
    case SHT2X_TEMPI2C_WR:
        SHT2x_I2cStartCondition();                            
        SHT2x_I2cWriteByte(I2C_ADR_W);
        SHT2x_I2cWriteByte(TRIG_TEMP_MEASUREMENT_POLL);
        eSht2xsampleState = SHT2X_TEMPI2C_RDACK;
        break;
    case SHT2X_TEMPI2C_RDACK:
        sht2xtemp_sample_time += ms;
        if(sht2xtemp_sample_time > 6)
        {
            sht2xtemp_sample_time = 0;
            SHT2x_I2cStartCondition();
            ack = SHT2x_I2cWriteByte(I2C_ADR_R);
            if(ACK_ERROR == ack)
            {
                ack_err++;
                if(ack_err > 20)
                {
                    ack_err = 0;
                    g_sht2x_temperature = 128.5;
                    eSht2xsampleState = SHT2X_HUMII2C_WR;
                }
            }
            else
            {
                eSht2xsampleState = SHT2X_TEMPI2C_RD;
                ack_err = 0;
            }
        }
        break;
    case SHT2X_TEMPI2C_RD:
        tmp1 = SHT2x_I2cReadByte();
        SHT2x_I2cAcknowledge();
        tmp2 = SHT2x_I2cReadByte();
        SHT2x_I2cNoAcknowledge();
        SHT2x_I2cStopCondition();
        
        ST = (tmp1 << 8) | (tmp2 << 0);
        ST &= ~0x0003;
        g_sht2x_temperature = ((float)ST * 0.00268127) - 46.85; 
        if((g_sht2x_temperature >= -40)&&(g_sht2x_temperature <= 125))
        {
            eSht2xsampleState = SHT2X_HUMII2C_WR;
        }
        else
        {
            g_sht2x_temperature = 128.5;//������ȷ��Χ�ڣ��õ��涨�����¶�ֵ
            eSht2xsampleState = SHT2X_HUMII2C_WR;
        }
        break;
    default:
        break;
    }
}
/**************************************************
* �������ƣ�float SHT2x_MeasureHumiPoll(int ms)
* �������ܣ�SHT2X ��ʪ�ȵ�λ
* ���������
* ���������
* ����ֵ  ��
**************************************************/
void SHT2x_MeasureHumiPoll(int ms)
{
    uint8_t ack;
    uint8_t tmp1;
    uint8_t tmp2;    
    uint16_t SRH;
    static uint8_t i_err = 0;
    static uint16_t sht2xhumi_sample_time = 0;
    switch(eSht2xsampleState)
    {
    case SHT2X_HUMII2C_WR:
        {
            SHT2x_I2cStartCondition();                               
            SHT2x_I2cWriteByte(I2C_ADR_W);
            SHT2x_I2cWriteByte(TRIG_HUMI_MEASUREMENT_POLL);
            eSht2xsampleState = SHT2X_HUMII2C_RDACK;
        }
        break;
    case SHT2X_HUMII2C_RDACK:
        {
            sht2xhumi_sample_time += ms;
            if(sht2xhumi_sample_time > 6)
            {
                sht2xhumi_sample_time = 0;
                SHT2x_I2cStartCondition();
                ack = SHT2x_I2cWriteByte(I2C_ADR_R);
                if(ACK_ERROR == ack)
                {
                    i_err++;
                    if(i_err > 20)
                    {
                        i_err = 0;
                        g_sht2x_humidity = 100.5;
                        eSht2xsampleState = SHT2X_SAMPLE_COMPLETE;
                    }
                }
                else
                {
                    eSht2xsampleState = SHT2X_HUMII2C_RD;
                    i_err = 0;
                }
            }
        }
        break;
    case SHT2X_HUMII2C_RD:
        {
            tmp1 = SHT2x_I2cReadByte();
            SHT2x_I2cAcknowledge();
            tmp2 = SHT2x_I2cReadByte();
            SHT2x_I2cNoAcknowledge();
            SHT2x_I2cStopCondition();
            
            SRH = (tmp1 << 8) | (tmp2 << 0);
            SRH &= ~0x0003;
            g_sht2x_humidity = ((float)SRH * 0.00190735) - 6;
            if((g_sht2x_humidity >= 0)&&(g_sht2x_humidity <=100))
            {
                eSht2xsampleState = SHT2X_SAMPLE_COMPLETE;
            }
            else
            {
                g_sht2x_humidity = 100.5; //������ȷ��Χ�ڣ��õ��涨����ʪ��ֵ
                eSht2xsampleState = SHT2X_SAMPLE_COMPLETE;
            }
        }
        break;
    default:
        break;
    }
}

/**************************************************
* �������ƣ�float SHT2x_CalcRH(uint16_t u16sRH)
* �������ܣ�SHT2X ��ʪ�Ȱٷֱ�
* ���������
* ���������
* ����ֵ  ��
**************************************************/
//==============================================================================
float SHT2x_CalcRH(uint16_t u16sRH)
//==============================================================================
{
    float humidityRH;              // variable for result
    
    u16sRH &= ~0x0003;          // clear bits [1..0] (status bits)
    //-- calculate relative humidity [%RH] --
    
    humidityRH = -6.0 + 125.0/65536 * (float)u16sRH; // RH= -6 + 125 * SRH/2^16
    return humidityRH;
}

/**************************************************
* �������ƣ�float SHT2x_CalcTemperatureC(uint16_t u16sT)
* �������ܣ�SHT2X �����¶�ֵ
* ���������
* ���������
* ����ֵ  ��
**************************************************/
//==============================================================================
float SHT2x_CalcTemperatureC(uint16_t u16sT)
//==============================================================================
{
    float temperatureC;            // variable for result
    
    u16sT &= ~0x0003;           // clear bits [1..0] (status bits)
    
    //-- calculate temperature--
    temperatureC= -46.85 + 175.72/65536 *(float)u16sT; //T= -46.85 + 175.72 * ST/2^16
    return temperatureC;
}

/**************************************************
* �������ƣ�uint8_t SHT2x_ReadUserReg(void) 
* �������ܣ���ȡ�Ĵ���״̬
* ���������
* ���������
* ����ֵ  ��
**************************************************/

uint8_t SHT2x_ReadUserReg(void)    
{
    uint8_t reg;
    SHT2x_I2cStartCondition();                 
    SHT2x_I2cWriteByte(I2C_ADR_W);
    SHT2x_I2cWriteByte(USER_REG_R);
    SHT2x_I2cStartCondition(); 
    SHT2x_I2cWriteByte(I2C_ADR_R);
    reg = SHT2x_I2cReadByte();
    SHT2x_I2cNoAcknowledge();
    SHT2x_I2cStopCondition();
    
    return (reg); 
}

/**************************************************
* �������ƣ�uint8_t SHT2x_WriteUserReg(uint8_t reg) 
* �������ܣ�д�Ĵ�������
* ���������
* ���������
* ����ֵ  ��
**************************************************/

uint8_t SHT2x_WriteUserReg(uint8_t reg)
{
    uint8_t ack;
    
    SHT2x_I2cStartCondition();
    SHT2x_I2cWriteByte(I2C_ADR_W);
    SHT2x_I2cWriteByte(USER_REG_W);
    ack = SHT2x_I2cWriteByte(reg);
    SHT2x_I2cStopCondition();  
    
    return (ack);
}

/**************************************************
* �������ƣ�void SHT2x_GetSerialNumber(uint8_t *buf) 
* �������ܣ��õ����к�
* ���������
* ���������
* ����ֵ  ��
**************************************************/
void SHT2x_GetSerialNumber(uint8_t *buf)
{
    /* Read from memory location 1 */
    SHT2x_I2cStartCondition();
    SHT2x_I2cWriteByte(I2C_ADR_W); //I2C address
    SHT2x_I2cWriteByte(0xFA);      //Command for readout on-chip memory
    SHT2x_I2cWriteByte(0x0F);      //on-chip memory address
    SHT2x_I2cStartCondition();
    SHT2x_I2cWriteByte(I2C_ADR_R); //I2C address
    buf[5] = SHT2x_I2cReadByte();  //Read SNB_3
    SHT2x_I2cAcknowledge();
    SHT2x_I2cReadByte();           //Read CRC SNB_3 (CRC is not analyzed)
    SHT2x_I2cAcknowledge();
    buf[4] = SHT2x_I2cReadByte();  //Read SNB_2
    SHT2x_I2cAcknowledge();
    SHT2x_I2cReadByte();           //Read CRC SNB_2 (CRC is not analyzed)
    SHT2x_I2cAcknowledge();
    buf[3] = SHT2x_I2cReadByte();  //Read SNB_1
    SHT2x_I2cAcknowledge();
    SHT2x_I2cReadByte();           //Read CRC SNB_1 (CRC is not analyzed)
    SHT2x_I2cAcknowledge();
    buf[2] = SHT2x_I2cReadByte();  //Read SNB_0
    SHT2x_I2cAcknowledge();
    SHT2x_I2cReadByte();           //Read CRC SNB_0 (CRC is not analyzed)
    SHT2x_I2cNoAcknowledge();
    SHT2x_I2cStopCondition();
    
    /* Read from memory location 2 */
    SHT2x_I2cStartCondition();
    SHT2x_I2cWriteByte(I2C_ADR_W); //I2C address
    SHT2x_I2cWriteByte(0xFC);      //Command for readout on-chip memory
    SHT2x_I2cWriteByte(0xC9);      //on-chip memory address
    SHT2x_I2cStartCondition();
    SHT2x_I2cWriteByte(I2C_ADR_R); //I2C address
    buf[1] = SHT2x_I2cReadByte();  //Read SNC_1
    SHT2x_I2cAcknowledge();
    buf[0] = SHT2x_I2cReadByte(); //Read SNC_0
    SHT2x_I2cAcknowledge();
    SHT2x_I2cReadByte();          //Read CRC SNC0/1 (CRC is not analyzed)
    SHT2x_I2cAcknowledge();
    buf[7] = SHT2x_I2cReadByte(); //Read SNA_1
    SHT2x_I2cAcknowledge();
    buf[6] = SHT2x_I2cReadByte(); //Read SNA_0
    SHT2x_I2cAcknowledge();
    SHT2x_I2cReadByte();          //Read CRC SNA0/1 (CRC is not analyzed)
    SHT2x_I2cNoAcknowledge();
    SHT2x_I2cStopCondition();
}

/*******************************************************************************
* ����: float Get_SHT2x_MeasureTempPoll(void)
* ����: SHT2X��ȡ�¶�ֵȡƽ��
* �β�: ��
* ����: avg_temperature��ƽ���¶�ֵ
* ˵��: �� 
******************************************************************************/  
//float Get_SHT2x_MeasureTempPoll(void)
//{
//    float avg_temperature = 0;
//    float temperature[SAMPLE_NUM] = {0};
//    uint8_t i = 0;
//    for(i = 0; i < SAMPLE_NUM; i++)
//    {
//     temperature[i] = SHT2x_MeasureTempPoll();
//     delay_ms(INTERVAL);
//    }
//    avg_temperature = get_averagedata(temperature,SAMPLE_NUM);
//    return avg_temperature;
//}

/*******************************************************************************
* ����: float Get_SHT2x_MeasureHumiPoll(void)
* ����: SHT2X��ȡʪ��ֵȡƽ��
* �β�: ��
* ����: avg_humidity��ƽ���¶�ֵ
* ˵��: �� 
******************************************************************************/  
//float Get_SHT2x_MeasureHumiPoll(void)
//{
//    float avg_humidity = 0;
//    float humidity[SAMPLE_NUM] = {0};
//    uint8_t i = 0;
//    for(i = 0; i < SAMPLE_NUM; i++)
//    {
//     humidity[i] = SHT2x_MeasureHumiPoll();
//     delay_ms(INTERVAL);
//    }
//    avg_humidity = get_averagedata(humidity,SAMPLE_NUM);
//    return avg_humidity;
//}
