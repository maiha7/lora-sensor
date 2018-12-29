#include "DriverConfig.h"
#include "tim2.h"
#include "apply.h"
#include "usart_t.h"
#include "stm8l15x_adc.h"
//低功耗模式状态标志
uint8_t g_LowPowerCntState = 0; 

EDriverState  eDriverState;
//初始化AD的IO口
void DrADC_Init(void)
{
    //ADC_IOConfig();
    //ADC_CR1  = 0x41;       // 8bit数据 单次转换模式 先上电唤醒ADC转换       
    //ADC_CR3  = 0x00;       //非外部触发 设置转换通道0 
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
    ADC_Cmd(ADC1, ENABLE);//这是AD的初始化
}

//AD方式、MP-702设备、氨气数据采集
float ADC_NH3_Read(void)
{
        char i = 0;
	unsigned long numtemp = 0;						 
        float Max_Value = 0;
        float Min_Value = 0;
        float temp_l = 0;
	//DrADC_Init();					           //重新初始化AD
        ADC_SoftwareStartConv(ADC1);
        //delay(1000);
        while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC) == 0 );
        temp_l=ADC_GetConversionValue(ADC1);
	Min_Value = temp_l;				   //取一次参考值
	Max_Value = temp_l;
 
	for(i = 0; i < 7; i++)
	{	
	    ADC_SoftwareStartConv(ADC1);
            //delay(1000);
            while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC) == 0 );
            temp_l=ADC_GetConversionValue(ADC1);            //等待转换完成
					
		if(temp_l >= Max_Value)			//取最大值
		{
			Max_Value = temp_l;			
		}
		if(temp_l <= Min_Value)			//取最小值
		{
			Min_Value = temp_l;
		}								
		numtemp += temp_l;	               //累加
	}
	numtemp -= Max_Value;				//减去最大和最小两个值
	numtemp -= Min_Value;
 
	if(numtemp > 0xFFFF)				//超出为错误，否则求平均数并返回值
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
        
        
	return Vale_ppm;   //氨气浓度 ppm
}
/*/MP503
//1-RS/R0对应0-9，对应甲醛浓度ppmx10
//A(4095-A0)/A0(4095-A)
const  uint16_t R0_RS[10]={10,15,25,40,75,150,260,450,800,1000};
uint16_t  pm0ad=PM0;
uint16_t  mp503;
//获取指定口的AD值 MP503 1X,PM25 10X
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


//初始化IO
void DrGPIO_Init(void)
{
  //ST8_GPIO_Init();
}


//写对应IO状态
void DrGPIO_Write(int index, int value)
{
    
}

//读取对应IO口状态
int GPIO_Read(int index)
{
    return 1;
}

//写串口数据
int UART_Write(void* p, int len)
{
  CdyUARTWrite(p,len);
  return len;
}

//读取串口数据
int UART_Read(void* p, int len)
{
  return CdyUARTRead(p,len);
}


//初始化数据
void ROM_Init(void)
{
    
}

//先EEPROM数据
int ROM_Write(int addr, void* p, int len)
{
	//emb_Eeprom_WriteData(0, addr, p, len);
  return len;
}

//读取EEPROM数据
int dyROM_Read(int addr, void* p, int len)
{
	//emb_Eeprom_ReadData(0, addr, p, len);
  return len;
}

// 驱动配置
void DriverConfig(void)
{
    //系统时钟初始化
    Sysclk_Init();
    //定时器2初始化  
    tim2_init();
    //采集初始化
    Sample_Init();
    //初始化串口
    USARTx_Init(2);
    //UARTInit();
    //开全局中断   
    enableInterrupts();
}

//低功耗模式标志
void LowPowerCntSet(uint8_t LowPowerCntState)
{  
   g_LowPowerCntState = LowPowerCntState;  
}  
//读取系统低功耗状态
uint8_t LowPowerCntGet(void)
{
    return g_LowPowerCntState;
}

uint8_t g_iSendData = 0;
// 驱动任务
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
    //采集数据
      Sampledata_Read(ms);
    break;
   case DataSend_State:
    //发送数据
       if(g_iSendData)
       {
           g_iSendData = 0;
           Sampledata_Send();
       }
    break;
   case GotoHalt_State:
      //设置进入低功耗模式标志
      //LowPowerCntSet(LOW_POWER_CNT_STATE_HALT);
       //进入低功耗状态
     // SimSleep(g_LowPowerCntState);
       //驱动状态改变为采样读取
       
       eDriverState = SampleRead_State;
    break;
   default:
    break;
   }
}

//看门狗
void  Clear_IWDG_Flag(void)
{
  IWDG_ReloadCounter();
}


// 得到系统运行毫秒数值
uint32_t SysTimeGetms(void)
{
    //系统运行时钟保存 
    static uint32_t prev_time_ms = 0;
    //运行时间差值
    uint32_t run_time_ms = 0;
    //当前计数值
    uint32_t present_time_ms  = 0;
    //时钟更新
    present_time_ms = g_MsTimeCnt;
    //读取这次系统运行时钟差值
    run_time_ms = present_time_ms - prev_time_ms;
    //更新当前系统时间值
    prev_time_ms = present_time_ms;
    //返回系统运行时钟差值
    return run_time_ms;
}

//系统全局变量、各种结构初始化
void SoftwareInit(void)
{
    g_LowPowerCntState = LOW_POWER_CNT_STATE_ACTIVE;
    eDriverState = SampleRead_State;
}

//低功耗处理函数
void LowPowerConsumption_Cmd(uint8_t LowPowerCnt_State)  
{  
  //开启低功耗  
  if(LowPowerCnt_State == LOW_POWER_CNT_STATE_HALT)  
  {  
    //进入停机模式  
    Goto_Haltmode();
    //改变低功耗模式标志位
    LowPowerCntSet(LOW_POWER_CNT_STATE_ACTIVE);
    //驱动状态改变为采样读取
    eDriverState = SampleRead_State;
    //系统时钟初始化
    Sysclk_Init(); 
    //定时器2初始化  
    tim2_init();   
    //采集初始化
    Sample_Init();
  }     
} 

void SimSleep(uint8_t LowPowerCnt_State)
{
    LowPowerConsumption_Cmd(LowPowerCnt_State);
}

uint16_t acData;
uint16_t VocData[5];
// 运行
void Comm_Run(int ms)
{
	// 接收主电源的心跳,输出开关命令
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
//				// 判断是否开关电源命令
//				unsigned char bOn = 0;
//				if(dyCharge_PowerOnoff_Return_Decode(0, &bOn, buf, len))
//				{
//					// 收到开关电源命令，回复命令
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
//		// 判断是否长时间没有接收到心跳
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
