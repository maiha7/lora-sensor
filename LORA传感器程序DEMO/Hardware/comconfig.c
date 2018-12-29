/**************************************************
 * Copyright (c) 
 * All rights reserved.
 * 
 * 文件名称：comconfig.c
 * 文件标识：
 * 摘    要：调试串口配置信息
 * 
 * 当前版本：V1.0.0
 * 作    者：yjd
 * 完成日期：2017-11-09
**************************************************/
#include "rs485.h"
#include "led.h"
#include "string.h"  
#include "flash.h" 
#include "comconfig.h" 


SCalData  CDataCalibration[AT_CAL_TYPE_MAX_CNT];

/*xa(传感器测量值) ya(理论值) 已知的标定节点，n 标定节点个数，x 测量值，y 插值算法获得的值
 推荐4到5个节点，无法计算返回0*/
 //折线算法
uint8_t fold_line(float *xa,float *ya,int n,const float x,float *y, EFoldLineType type)
{
  if(n < 2)	//节点数少于2个无法计算差值
  {
    return 0;
  }
  if(xa[0] > x)	//测量值小于标定节点范围
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
  //n-1条折线段
  for(i=0; i<n-1; i++)	
  {
    if(xa[i+1] >= x)	//查找x落在哪条线段上
    {
      if(xa[i] == xa[i+1])	//2个标定节点相同时多项式无穷大无法计算
      {
        return 0;
      }
      float tx = (x-xa[i])/(xa[i+1]-xa[i]);
      float ty = ya[i+1]-ya[i];
      y[0] = ty*tx+ya[i];	//两点式公式求得线段上y值
      break;
    }
  }
  if(i >= n-1)	//x大于标定节点范围
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

// 校准数据
float CalibrateData(SCalData *caldata, float fValue)
{
  // 调用校准函数
  float y = fValue;
  caldata->cnt = min(caldata->cnt, countof(caldata->samples));
  //折线校准
  if(fold_line(caldata->samples, caldata->calibrs, caldata->cnt, fValue, &y, fltExtLine))
  {
    return y;
  }
  else
  {
    return fValue;
  }
}