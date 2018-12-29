#include "DriverConfig.h"
#include "tim2.h"
#include "apply.h"
#include "usart_t.h"
#include "stm8l15x_adc.h"
//�͹���ģʽ״̬��־
uint8_t g_LowPowerCntState = 0; 

EDriverState  eDriverState;
//��ʼ��AD��IO��
void DrADC_Init(void)
{
    //ADC_IOConfig();
    //ADC_CR1  = 0x41;       // 8bit���� ����ת��ģʽ ���ϵ绽��ADCת��       
    //ADC_CR3  = 0x00;       //���ⲿ���� ����ת��ͨ��0 
    CLK_PeripheralClockConfig(CLK_Peripheral_ADC1, ENABLE);
    ADC_DeInit(ADC1);
    /* Initialize and configure ADC1 */
    ADC_Init(ADC1, ADC_ConversionMode_Single,
    ADC_Resolution_12Bit, ADC_Prescaler_1);
    ADC_SamplingTimeConfig(ADC1, ADC_Group_SlowChannels, 
    ADC_SamplingTime_384Cycles);
    /* Enable ADC1 Channel 10*/
    ADC_ChannelCmd(ADC1,ADC_Channel_0, ENABLE); 
                //ADC1->SQR[regindex] |= (uint8_t)(ADC_Channels);
                //ADC_ITConfig(ADC1, ADC_IT_EOC, DISABLE);
    ADC_VrefintCmd(ENABLE);
    ADC_Cmd(ADC1, ENABLE);//����AD�ĳ�ʼ��
}

//AD��ʽ��MP-702�豸���������ݲɼ�
float ADC_NH3_Read(void)
{
        char i = 0;
	unsigned long numtemp = 0;						 
        float Max_Value = 0;
        float Min_Value = 0;
        float temp_l = 0;
	//DrADC_Init();					           //���³�ʼ��AD
        ADC_SoftwareStartConv(ADC1);
        //delay(1000);
        while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC) == 0 );
        temp_l=ADC_GetConversionValue(ADC1);
	Min_Value = temp_l;				   //ȡһ�βο�ֵ
	Max_Value = temp_l;
 
	for(i = 0; i < 7; i++)
	{	
	    ADC_SoftwareStartConv(ADC1);
            //delay(1000);
            while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC) == 0 );
            temp_l=ADC_GetConversionValue(ADC1);            //�ȴ�ת�����
					
		if(temp_l >= Max_Value)			//ȡ���ֵ
		{
			Max_Value = temp_l;			
		}
		if(temp_l <= Min_Value)			//ȡ��Сֵ
		{
			Min_Value = temp_l;
		}								
		numtemp += temp_l;	               //�ۼ�
	}
	numtemp -= Max_Value;				//��ȥ������С����ֵ
	numtemp -= Min_Value;
 
	if(numtemp > 0xFFFF)				//����Ϊ���󣬷�����ƽ����������ֵ
	{
		temp_l = 0xFF; 
	}
	else
	{
		temp_l = numtemp/5;
	}
        const  uint16_t VS_V0[10]={0,500,750,1000,1150,1600,1850,2100,2200,2450};
        const  uint16_t slo[9]={200,100,83,75,45,40,25,10,15};
        float ValeTemp = 0;
        //uint8_t i;
        static  float Vale_ppm = 0;
        float Vale0 = 700;
        ValeTemp = (temp_l/4096)*3774;
        for(uint8_t i=0;i<9;i++)
        {
          if(VS_V0[i]<(ValeTemp-Vale0)&&VS_V0[i+1]>(ValeTemp-Vale0))
          {
            Vale_ppm = (ValeTemp-Vale0)/slo[i];
          }
          else
          {
            Vale_ppm = (ValeTemp-Vale0)/slo[8];
          }
        }
          
//        f =ValeTemp;
//        f /=(1024-ValeTemp);
//        f *= ADXIS[pcom];
//        num =(uint8_t)f;
//        if(num > 9)
//        {
//            return 101;
//        }
    //    f0 = R0_RS[num+1]-R0_RS[num];
     //   f0 *=(f-num);
      //  f0 +=R0_RS[num];
      //  return f0/10;
        
        
	return Vale_ppm;   //����Ũ�� ppm
}
/*/MP503
//1-RS/R0��Ӧ0-9����Ӧ��ȩŨ��ppmx10
//A(4095-A0)/A0(4095-A)
const  uint16_t R0_RS[10]={10,15,25,40,75,150,260,450,800,1000};
uint16_t  pm0ad=PM0;
uint16_t  mp503;
//��ȡָ���ڵ�ADֵ MP503 1X,PM25 10X
float GetADCValue(ADCUSE pcom)
{  
    uint32_t ValeTemp;
    uint8_t num;
    float f,f0;  
    ValeTemp = 0;
    for(uint8_t i=0;i<8;i++)
    {
        ValeTemp += ADC_ValueTable[pcom+i*ADCCHANNEL];
    }
    ValeTemp >>= 3;
    if(pcom==MP503) 
    {
        f =ValeTemp;
        f /=(4095-ValeTemp);
        f *= ADXIS[pcom];  
        num =(uint8_t)f;
        if(num > 9)
        {
            return 101;
        }
        f0 = R0_RS[num+1]-R0_RS[num];
        f0 *=(f-num);
        f0 +=R0_RS[num];
        return f0/10;
    }
    else
    {
        if(ValeTemp<pm0ad)
        {
            pm0ad = ValeTemp;
            return 0;
        }
        f = ValeTemp-pm0ad;    //VREF 3.3
        f *= ADXIS[pcom];	
        return f; 
    }
    
}*/


//��ʼ��IO
void DrGPIO_Init(void)
{
  //ST8_GPIO_Init();
}


//д��ӦIO״̬
void DrGPIO_Write(int index, int value)
{
    
}

//��ȡ��ӦIO��״̬
int GPIO_Read(int index)
{
    return 1;
}

//д��������
int UART_Write(void* p, int len)
{
  CdyUARTWrite(p,len);
  return len;
}

//��ȡ��������
int UART_Read(void* p, int len)
{
  return CdyUARTRead(p,len);
}


//��ʼ������
void ROM_Init(void)
{
    
}

//��EEPROM����
int ROM_Write(int addr, void* p, int len)
{
	//emb_Eeprom_WriteData(0, addr, p, len);
  return len;
}

//��ȡEEPROM����
int dyROM_Read(int addr, void* p, int len)
{
	//emb_Eeprom_ReadData(0, addr, p, len);
  return len;
}

// ��������
void DriverConfig(void)
{
    //ϵͳʱ�ӳ�ʼ��
    Sysclk_Init();
    //��ʱ��2��ʼ��  
    tim2_init();
    //�ɼ���ʼ��
    Sample_Init();
    //��ʼ������
    USARTx_Init(2);
    //UARTInit();
    //��ȫ���ж�   
    enableInterrupts();
}

//�͹���ģʽ��־
void LowPowerCntSet(uint8_t LowPowerCntState)
{  
   g_LowPowerCntState = LowPowerCntState;  
}  
//��ȡϵͳ�͹���״̬
uint8_t LowPowerCntGet(void)
{
    return g_LowPowerCntState;
}

uint8_t g_iSendData = 0;
// ��������
void DriverTask(int ms)
{
    static int mms = 0;
    static int mss = 0;
    mms += ms;
    if(mms > 100)
    {
        mss ++;
        mms = 0;
        if(mss > 1000)
        {
            mss = 0;
            g_iSendData = 1;
        }
    }
   switch(eDriverState)
   {
   case SampleRead_State:
    //�ɼ�����
      Sampledata_Read(ms);
    break;
   case DataSend_State:
    //��������
       if(g_iSendData)
       {
           g_iSendData = 0;
           Sampledata_Send();
       }
    break;
   case GotoHalt_State:
      //���ý���͹���ģʽ��־
      //LowPowerCntSet(LOW_POWER_CNT_STATE_HALT);
       //����͹���״̬
     // SimSleep(g_LowPowerCntState);
       //����״̬�ı�Ϊ������ȡ
       
       eDriverState = SampleRead_State;
    break;
   default:
    break;
   }
}

//���Ź�
void  Clear_IWDG_Flag(void)
{
  IWDG_ReloadCounter();
}


// �õ�ϵͳ���к�����ֵ
uint32_t SysTimeGetms(void)
{
    //ϵͳ����ʱ�ӱ��� 
    static uint32_t prev_time_ms = 0;
    //����ʱ���ֵ
    uint32_t run_time_ms = 0;
    //��ǰ����ֵ
    uint32_t present_time_ms  = 0;
    //ʱ�Ӹ���
    present_time_ms = g_MsTimeCnt;
    //��ȡ���ϵͳ����ʱ�Ӳ�ֵ
    run_time_ms = present_time_ms - prev_time_ms;
    //���µ�ǰϵͳʱ��ֵ
    prev_time_ms = present_time_ms;
    //����ϵͳ����ʱ�Ӳ�ֵ
    return run_time_ms;
}

//ϵͳȫ�ֱ��������ֽṹ��ʼ��
void SoftwareInit(void)
{
    g_LowPowerCntState = LOW_POWER_CNT_STATE_ACTIVE;
    eDriverState = SampleRead_State;
}

//�͹��Ĵ�����
void LowPowerConsumption_Cmd(uint8_t LowPowerCnt_State)  
{  
  //�����͹���  
  if(LowPowerCnt_State == LOW_POWER_CNT_STATE_HALT)  
  {  
    //����ͣ��ģʽ  
    Goto_Haltmode();
    //�ı�͹���ģʽ��־λ
    LowPowerCntSet(LOW_POWER_CNT_STATE_ACTIVE);
    //����״̬�ı�Ϊ������ȡ
    eDriverState = SampleRead_State;
    //ϵͳʱ�ӳ�ʼ��
    Sysclk_Init(); 
    //��ʱ��2��ʼ��  
    tim2_init();   
    //�ɼ���ʼ��
    Sample_Init();
  }     
} 

void SimSleep(uint8_t LowPowerCnt_State)
{
    LowPowerConsumption_Cmd(LowPowerCnt_State);
}

uint16_t acData;
uint16_t VocData[5];
// ����
void Comm_Run(int ms)
{
	// ��������Դ������,�����������
//	static int mms = 0;
	static int mss = 0;
	static char buf[30] = {0};
    static uint8_t num = 0;
    uint8_t crc;
	int len = UART_Read(buf, sizeof(buf));  //CdyUARTRead(p,len);
    if(len > 0)
    {
        //UARTWrite((void*)"\r\nThe Receive Data:\r\n",strlen("\r\nThe Receive Data:\r\n"));
        //UART_Write(buf,len);
        if(buf[0] == 0x5f && len == 4)
        {
          crc = buf[0]+buf[1]+buf[2];
          if(crc = buf[3])
          {
              VocData[num] = (buf[1]<<8)+ buf[2];
              num++;
              if(num >= 5)
              {
                 num = 0; 
              }
          }
        }
        mss = 0;
    }
    for(uint8_t i = 0; i<5; i++)
    {
        acData = +VocData[i];
    }
    acData = acData/5;
}
//		  else 
//		  {
//				// �ж��Ƿ񿪹ص�Դ����
//				unsigned char bOn = 0;
//				if(dyCharge_PowerOnoff_Return_Decode(0, &bOn, buf, len))
//				{
//					// �յ����ص�Դ����ظ�����
//					len = dyCharge_PowerOnoff_Return_Encode(Modersid, 1, buf, sizeof(buf));
//					dyUART_Write(buf, len);
//					g_iOutputEnable = bOn;
//					g_iReceiveOutput = 1;
//					g_iReceiveHeart = 0;
//				}
//				else
//				{
//					Receive_Data_Handle(buf,len);
//				}
//		  }
//		}
//		// �ж��Ƿ�ʱ��û�н��յ�����
//		mms += ms;
//		if(mms > _DY_RECV_HEART_CYCLE_)
//		{
//		  mss ++;
//		  mms = 0;
//		  if(mss > _DY_RECV_HEART_MSCYCLE_)
//		  {
//				mss = 0;
//				g_iReceiveHeart = 1;
//		  }
//		  
//		}
//	}
//	else        
//	{
//		if(len > 0)
//		{
//			Receive_Data_Handle(buf,len);
//		}
//	}
//  //  Comm_date_Run(ms);
//}
