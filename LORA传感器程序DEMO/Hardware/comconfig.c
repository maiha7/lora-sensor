/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * �ļ����ƣ�comconfig.c
 * �ļ���ʶ��
 * ժ    Ҫ�����Դ���������Ϣ
 * 
 * ��ǰ�汾��V1.0.0
 * ��    �ߣ�yjd
 * ������ڣ�2017-11-09
**************************************************/
#include "rs485.h"
#include "led.h"
#include "string.h"  
#include "flash.h" 
#include "comconfig.h" 


SCalData  CDataCalibration[AT_CAL_TYPE_MAX_CNT];

/*xa(����������ֵ) ya(����ֵ) ��֪�ı궨�ڵ㣬n �궨�ڵ������x ����ֵ��y ��ֵ�㷨��õ�ֵ
 �Ƽ�4��5���ڵ㣬�޷����㷵��0*/
 //�����㷨
uint8_t fold_line(float *xa,float *ya,int n,const float x,float *y, EFoldLineType type)
{
  if(n < 2)	//�ڵ�������2���޷������ֵ
  {
    return 0;
  }
  if(xa[0] > x)	//����ֵС�ڱ궨�ڵ㷶Χ
  {
    if(fltLastPoint == type)
    {
      y[0] = x+(ya[0]-xa[0]);
    }
    else if(fltExtLine == type)
    {
      y[0] = (ya[1]-ya[0])*(x-xa[0])/(xa[1]-xa[0])+ya[0];
    }
    return 1;
    //return false;
  }
  int i;
  //n-1�����߶�
  for(i=0; i<n-1; i++)	
  {
    if(xa[i+1] >= x)	//����x���������߶���
    {
      if(xa[i] == xa[i+1])	//2���궨�ڵ���ͬʱ����ʽ������޷�����
      {
        return 0;
      }
      float tx = (x-xa[i])/(xa[i+1]-xa[i]);
      float ty = ya[i+1]-ya[i];
      y[0] = ty*tx+ya[i];	//����ʽ��ʽ����߶���yֵ
      break;
    }
  }
  if(i >= n-1)	//x���ڱ궨�ڵ㷶Χ
  {
    if(fltLastPoint == type)
    {
      y[0] = x+(ya[n-1]-xa[n-1]);
    }
    else if(fltExtLine == type)
    {
      y[0] = (ya[n-1]-ya[n-2])*(x-xa[n-2])/(xa[n-1]-xa[n-2])+ya[n-2];
    }
    return 1;
    //return false;
  }
  return 1;
}

// У׼����
float CalibrateData(SCalData *caldata, float fValue)
{
  // ����У׼����
  float y = fValue;
  caldata->cnt = min(caldata->cnt, countof(caldata->samples));
  //����У׼
  if(fold_line(caldata->samples, caldata->calibrs, caldata->cnt, fValue, &y, fltExtLine))
  {
    return y;
  }
  else
  {
    return fValue;
  }
}