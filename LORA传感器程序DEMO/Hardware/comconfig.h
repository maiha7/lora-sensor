#ifndef __COMCONFIG_H
#define __COMCONFIG_H

// 最大值
#define max(a,b)  (((a) > (b)) ? (a) : (b))
// 最小值
#define min(a,b)  (((a) < (b)) ? (a) : (b))

// 一维数组个数
#ifndef countof
#define countof(a)          (sizeof(a) / sizeof(a[0]))
#endif

// 校准类型最大个数
#define AT_CAL_TYPE_MAX_CNT   5
// 校准数据最大个数
#define AT_CAL_DATA_MAX_CNT   15


typedef enum
{
  eCalLight = 0,
  eCalTemp,
  eCalPHum,
}ECalc;

// 折线法标定范围外计算类型
typedef enum EFoldLineType
{
	fltLastPoint = 0, // 按最后一点的偏移量计算
	fltExtLine = 1,   // 按最近一段延长线计算
}EFoldLineType;


// 校准数据
typedef struct CalData
{
  uint8_t calType;                      // 校准类型
  uint8_t cnt;                             // 校准数据的个数
  float zero;                               // 保留字段
  float samples[AT_CAL_DATA_MAX_CNT];   // 采样值 
  float calibrs[AT_CAL_DATA_MAX_CNT];     // 校准值 
  char  reserve[8];                      // 保留
}SCalData;


extern SCalData CDataCalibration[AT_CAL_TYPE_MAX_CNT];

uint8_t fold_line(float *xa,float *ya,int n, const float x, float *y, EFoldLineType type);

float CalibrateData(SCalData *caldata, float fValue);

#endif