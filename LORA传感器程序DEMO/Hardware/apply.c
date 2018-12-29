/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * �ļ����ƣ�apply.c
 * �ļ���ʶ��
 * ժ    Ҫ���ɼ���ȡ
 * 
 * ��ǰ�汾��V1.0.0
 * ��    �ߣ�yjd
 * ������ڣ�2017-03-23
**************************************************/
#include "led.h"
#include "exti.h"
#include "tim2.h"
#include "apply.h"
#include "stm8l15x.h"
#include "ds18b20.h"
#include "gy3x.h"
#include "sht2x.h"
//#include "mq137.h"
#include "sx1278.h"
#include "DriverConfig.h"
#include "ADC_N.h"

/*******************************************************************************
 * ����: void Sysclk_Init(void)
 * ����: ϵͳ������ʼ��
 * �β�: ��
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/

void Sysclk_Init(void)
{
     //ʹ���ⲿ���پ���8M
//   CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSE);//ϵͳ��ʱ��Ϊ
//   
//   while(CLK_GetFlagStatus(CLK_FLAG_HSERDY)==RESET);//�ȴ�HSE׼������
//   CLK_SYSCLKSourceSwitchCmd(ENABLE);//ʹ���л�
//   
//   if(CLK_GetSYSCLKSource()== CLK_SYSCLKSource_HSE)//����л��ɹ�
//   {
//     CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);//1��Ƶ,8M
//   } 
//   
//   
//   
//   
  	CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);  // ����ϵͳʱ��Ϊ16MHz
	while (CLK_GetClockFreq() != 16000000);// �ȴ�ϵͳʱ���ȶ����16MHz
}

///*******************************************************************************
// * ����: void Gpio_Output_LowPower(void)
// * ����: ϵͳ������ʼ��
// * �β�: ��
// * ����: ��
// * ˵��: �� 
// ******************************************************************************/
void Gpio_Output_LowPower(void)
{
    GPIO_Init(GPIOA, GPIO_Pin_All, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOB, GPIO_Pin_All, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOC, GPIO_Pin_All, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOD, GPIO_Pin_All, GPIO_Mode_Out_PP_Low_Slow);
    GY3x_SCL_HIGH();
    if(device_type != GAS) //�˽�������ģ��������
    {
     GY3x_SDA_HIGH(); 
    }
    SHT2x_SCL_HIGH();
    SHT2x_SDA_HIGH();
    DS18B20_DQ_HIGH;
    DS18B20_PW_HIGH;
}

/*******************************************************************************
 * ����:  void Sysclk_LSI(void)
 * ����:  LSI��ʼ��
 * �β�: ��
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/
 void Sysclk_LSI(void)
 {
    CLK_LSICmd(ENABLE);//ʹ��LSI
    CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_LSI);//LSIΪϵͳʱ��
    while(CLK_GetFlagStatus(CLK_FLAG_LSIRDY)==RESET);//�ȴ�LSI׼������
    CLK_SYSCLKSourceSwitchCmd (ENABLE);//ʹ���л�
 }

/*******************************************************************************
 * ����: void RTC_Config(void)
 * ����: �ܽų�ʼ��
 * �β�: ��
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/
void RTC_Config(void)
{
  //CLK_LSEConfig(CLK_LSE_ON);
  CLK_LSICmd(ENABLE);//ʹ��LSI
  CLK_RTCClockConfig(CLK_RTCCLKSource_LSI, CLK_RTCCLKDiv_64);//RTCʱ��ԴLSI��64��Ƶ=38K/64
  while (CLK_GetFlagStatus(CLK_FLAG_LSIRDY) == RESET);//�ȴ�LSI����  
  RTC_WakeUpCmd(DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_RTC, ENABLE);//RTCʱ���ſ�ʹ��
  RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16);//38K/16=2.375k=0.421ms
  RTC_ITConfig(RTC_IT_WUT, ENABLE);//�����ж�
  RTC_SetWakeUpCounter(5000);//2375*0,421=1S����

  //ITC_SetSoftwarePriority(RTC_CSSLSE_IRQn, ITC_PriorityLevel_3);//���ȼ�
  
  enableInterrupts();
}

/**************************************************
 * �������ƣ�void Sample_Init(void)
 * �������ܣ������豸���Ͳ�����ʼ��
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
void Sample_Init(void)
{
  switch(device_type)
  {
    case DS18B20:  
       Ds18b20Gpio_Init();
      break;
    case SHT2X:
       SHT2x_Init();
      break;
    case GY3X:
       GY3x_Init();
      break;
    case SHT2XANDGY3X:
       SHT2x_Init();
       GY3x_Init();
      break;
    case GAS:
//       Adc_Init();
         DrADC_Init();
      break;
  case GASSHT2X:
      SHT2x_Init();
//       Adc_Init();
      DrADC_Init();
      break;
    default:
      break;
  }
}

/**************************************************
 * �������ƣ�void Sampledata_Send(void)
 * �������ܣ������������ߴ���
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
void Sampledata_Send(void)
{
    Lora_Init(); 
    Lora_Interaction_Protocol(); //�ɼ�����Э��ת��
    Process_Lora_Send(Lora_senddata,lora_slen); //�������ݰ�
}

/**************************************************
 * �������ƣ�void Sampledata_Read(int ms)
 * �������ܣ������豸���Ͷ�ȡ��������
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
void Sampledata_Read(int ms)
{
    switch(device_type)
    {
    case DS18B20:  
        DS18B20_ReadTemperature(ms);
        if(eDs18b20sampleState == DS18B20SAMPLE_COMPLETE)
        {
          eDriverState = DataSend_State;
          eDs18b20sampleState = DS18B20_INIT;
        }
        break;
    case SHT2X:
         SHT2x_MeasureTempPoll(ms);
         SHT2x_MeasureHumiPoll(ms);
         if(eSht2xsampleState == SHT2X_SAMPLE_COMPLETE)
         {
           eDriverState = DataSend_State;
           eSht2xsampleState = SHT2X_TEMPI2C_WR;
         }
        break;
    case GY3X:
        GY3x_ReadLightdata(ms); 
        if(eGy3xsampleState  == GY3X_SAMPLE_COMPLETE)
        {
          eDriverState = DataSend_State;
          eGy3xsampleState = GY3X_SAMPLE_START;
        }
        break;
    case SHT2XANDGY3X:
        SHT2x_MeasureTempPoll(ms);
        SHT2x_MeasureHumiPoll(ms);
        GY3x_ReadLightdata(ms);
        if((eSht2xsampleState == SHT2X_SAMPLE_COMPLETE)&& \
          (eGy3xsampleState == GY3X_SAMPLE_COMPLETE))
        {
          eDriverState = DataSend_State;
          eSht2xsampleState = SHT2X_TEMPI2C_WR;
          eGy3xsampleState = GY3X_SAMPLE_START;
        }
        break;
    case GAS:
        
          eDriverState = DataSend_State;
//        Mq137_MeasureNH3ppm(ms);
//        if(eMq137sampleState == MQ137_SAMPLE_COMPLETE)
//        {
//          eDriverState = DataSend_State;
//          eMq137sampleState = MQ137_SAMPLE_START;
//        }
      break;
    case GASSHT2X:
        SHT2x_MeasureTempPoll(ms);
        SHT2x_MeasureHumiPoll(ms);
        //GY3x_ReadLightdata(ms);
        AD_Sample_data();
        if(eSht2xsampleState == SHT2X_SAMPLE_COMPLETE)
        {
          eDriverState = DataSend_State;
          eSht2xsampleState = SHT2X_TEMPI2C_WR;
          //eGy3xsampleState = GY3X_SAMPLE_START;
        }
        break;
    default:
        break;
    }
}
/**************************************************
 * �������ƣ�void Goto_Haltmode(void)
 * �������ܣ�����HALTģʽ���͹���
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
void Goto_Haltmode(void)
{
    TIM2_Cmd(DISABLE); //��ʱ���ر�
    g_MsTimeCnt = 0; //ϵͳms���¼�������
    Gpio_Output_LowPower();//ʹGPIO�ܽŹ��Ľ������
    RTC_Config();    //RTC����
    PWR_FastWakeUpCmd(ENABLE);  //���ٻ���ʹ��
    PWR_UltraLowPowerCmd(ENABLE);//���͹���    
    RTC_ITConfig(RTC_IT_WUT, ENABLE);//���Ѷ�ʱ���ж�ʹ��
    RTC_WakeUpCmd(ENABLE);//RTC����ʹ��
//    Sysclk_LSI();
    halt();//ͣ��ģʽ
}

/**************************************************
 * �������ƣ�ascending_sort(uint a[],uchar n)
 * �������ܣ�������������   
 * ���������a[],n
 * ���������
 * ����ֵ  ��
 **************************************************/
void ascending_sort(uint16_t a[],uint8_t n)
{
  uint8_t i;
  uint8_t j;
  uint16_t temp;
  for(i = 0; i < n; i++)
  {
    for(j = i+1; j < n; j++)
    {
      if(a[i] > a[j])
      {
       temp = a[i];
       a[i] = a[j];
       a[j] = temp;          
      }
    }
  }
}

/**************************************************
 * �������ƣ�float get_averagedata(uint16_t a[],uint8_t n)
 * �������ܣ�ȡƽ�����㷨
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
float get_averagedata(float a[],uint8_t n)
{
  float average = 0;
  float sum = 0;
  uint16_t i;
  for(i = 0; i < n; i++)
  {
    sum = sum + a[i];
  }
  average = (float)sum / n;
  return average ;
}


/**************************************************
 * �������ƣ�int Round(float x)
 * �������ܣ���������
 * ���������
 * ���������
 * ����ֵ  ��
 **************************************************/
int Round(float x)
{
  return (int)(x>=0?(x+0.5):(x-0.5));
}


