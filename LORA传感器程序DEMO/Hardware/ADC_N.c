/**************************************************
 *
 * 
 * �ļ����ƣ�ADC_N.c
 * �ļ���ʶ��
 * ժ    Ҫ������
 * 
 * ��ǰ�汾��1.0
 * ��    �ߣ�
 * ������ڣ�
 **************************************************/


#include "ADC_N.h"
#include "tim2.h"
#include "DriverConfig.h" 
float ADC_NH3_DATA = 0;
//ʱ������
void AD_Sample_data(void)
{
  if(adtime == 1)  
 {
    ADC_NH3_DATA = ADC_NH3_Read();//1s����һ��NH3ֵ
  }
}
