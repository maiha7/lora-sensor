/**************************************************
 *
 * 
 * 文件名称：ADC_N.c
 * 文件标识：
 * 摘    要：函数
 * 
 * 当前版本：1.0
 * 作    者：
 * 完成日期：
 **************************************************/


#include "ADC_N.h"
#include "tim2.h"
#include "DriverConfig.h" 
float ADC_NH3_DATA = 0;
//时间配置
void AD_Sample_data(void)
{
  if(adtime == 1)  
 {
    ADC_NH3_DATA = ADC_NH3_Read();//1s采样一次NH3值
  }
}
