/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * 文件名称：apply.c
 * 文件标识：
 * 摘    要：采集读取
 * 
 * 当前版本：V1.0.0
 * 作    者：yjd
 * 完成日期：2017-03-23
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
 * 名称: void Sysclk_Init(void)
 * 功能: 系统函数初始化
 * 形参: 无
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/

void Sysclk_Init(void)
{
     //使用外部高速晶振8M
//   CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSE);//系统主时钟为
//   
//   while(CLK_GetFlagStatus(CLK_FLAG_HSERDY)==RESET);//等待HSE准备就绪
//   CLK_SYSCLKSourceSwitchCmd(ENABLE);//使能切换
//   
//   if(CLK_GetSYSCLKSource()== CLK_SYSCLKSource_HSE)//如果切换成功
//   {
//     CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);//1分频,8M
//   } 
//   
//   
//   
//   
  	CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);  // 设置系统时钟为16MHz
	while (CLK_GetClockFreq() != 16000000);// 等待系统时钟稳定输出16MHz
}

///*******************************************************************************
// * 名称: void Gpio_Output_LowPower(void)
// * 功能: 系统函数初始化
// * 形参: 无
// * 返回: 无
// * 说明: 无 
// ******************************************************************************/
void Gpio_Output_LowPower(void)
{
    GPIO_Init(GPIOA, GPIO_Pin_All, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOB, GPIO_Pin_All, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOC, GPIO_Pin_All, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOD, GPIO_Pin_All, GPIO_Mode_Out_PP_Low_Slow);
    GY3x_SCL_HIGH();
    if(device_type != GAS) //此脚作气体模拟量采样
    {
     GY3x_SDA_HIGH(); 
    }
    SHT2x_SCL_HIGH();
    SHT2x_SDA_HIGH();
    DS18B20_DQ_HIGH;
    DS18B20_PW_HIGH;
}

/*******************************************************************************
 * 名称:  void Sysclk_LSI(void)
 * 功能:  LSI初始化
 * 形参: 无
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/
 void Sysclk_LSI(void)
 {
    CLK_LSICmd(ENABLE);//使能LSI
    CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_LSI);//LSI为系统时钟
    while(CLK_GetFlagStatus(CLK_FLAG_LSIRDY)==RESET);//等待LSI准备就绪
    CLK_SYSCLKSourceSwitchCmd (ENABLE);//使能切换
 }

/*******************************************************************************
 * 名称: void RTC_Config(void)
 * 功能: 管脚初始化
 * 形参: 无
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/
void RTC_Config(void)
{
  //CLK_LSEConfig(CLK_LSE_ON);
  CLK_LSICmd(ENABLE);//使能LSI
  CLK_RTCClockConfig(CLK_RTCCLKSource_LSI, CLK_RTCCLKDiv_64);//RTC时钟源LSI，64分频=38K/64
  while (CLK_GetFlagStatus(CLK_FLAG_LSIRDY) == RESET);//等待LSI就绪  
  RTC_WakeUpCmd(DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_RTC, ENABLE);//RTC时钟门控使能
  RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16);//38K/16=2.375k=0.421ms
  RTC_ITConfig(RTC_IT_WUT, ENABLE);//开启中断
  RTC_SetWakeUpCounter(5000);//2375*0,421=1S左右

  //ITC_SetSoftwarePriority(RTC_CSSLSE_IRQn, ITC_PriorityLevel_3);//优先级
  
  enableInterrupts();
}

/**************************************************
 * 函数名称：void Sample_Init(void)
 * 函数介绍：根据设备类型采样初始化
 * 输入参数：
 * 输出参数：
 * 返回值  ：
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
 * 函数名称：void Sampledata_Send(void)
 * 函数介绍：采样数据无线传输
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/
void Sampledata_Send(void)
{
    Lora_Init(); 
    Lora_Interaction_Protocol(); //采集数据协议转换
    Process_Lora_Send(Lora_senddata,lora_slen); //发送数据包
}

/**************************************************
 * 函数名称：void Sampledata_Read(int ms)
 * 函数介绍：根据设备类型读取采样数据
 * 输入参数：
 * 输出参数：
 * 返回值  ：
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
 * 函数名称：void Goto_Haltmode(void)
 * 函数介绍：进入HALT模式降低功耗
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/
void Goto_Haltmode(void)
{
    TIM2_Cmd(DISABLE); //定时器关闭
    g_MsTimeCnt = 0; //系统ms更新计数清零
    Gpio_Output_LowPower();//使GPIO管脚功耗降到最低
    RTC_Config();    //RTC配置
    PWR_FastWakeUpCmd(ENABLE);  //快速唤醒使能
    PWR_UltraLowPowerCmd(ENABLE);//超低功耗    
    RTC_ITConfig(RTC_IT_WUT, ENABLE);//唤醒定时器中断使能
    RTC_WakeUpCmd(ENABLE);//RTC唤醒使能
//    Sysclk_LSI();
    halt();//停机模式
}

/**************************************************
 * 函数名称：ascending_sort(uint a[],uchar n)
 * 函数介绍：升序排列数组   
 * 输入参数：a[],n
 * 输出参数：
 * 返回值  ：
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
 * 函数名称：float get_averagedata(uint16_t a[],uint8_t n)
 * 函数介绍：取平均数算法
 * 输入参数：
 * 输出参数：
 * 返回值  ：
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
 * 函数名称：int Round(float x)
 * 函数介绍：四舍五入
 * 输入参数：
 * 输出参数：
 * 返回值  ：
 **************************************************/
int Round(float x)
{
  return (int)(x>=0?(x+0.5):(x-0.5));
}


