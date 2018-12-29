#ifndef __COMCONFIG_H
#define __COMCONFIG_H

// ���ֵ
#define max(a,b)  (((a) > (b)) ? (a) : (b))
// ��Сֵ
#define min(a,b)  (((a) < (b)) ? (a) : (b))

// һά�������
#ifndef countof
#define countof(a)          (sizeof(a) / sizeof(a[0]))
#endif

// У׼����������
#define AT_CAL_TYPE_MAX_CNT   5
// У׼����������
#define AT_CAL_DATA_MAX_CNT   15


typedef enum
{
  eCalLight = 0,
  eCalTemp,
  eCalPHum,
}ECalc;

// ���߷��궨��Χ���������
typedef enum EFoldLineType
{
	fltLastPoint = 0, // �����һ���ƫ��������
	fltExtLine = 1,   // �����һ���ӳ��߼���
}EFoldLineType;


// У׼����
typedef struct CalData
{
  uint8_t calType;                      // У׼����
  uint8_t cnt;                             // У׼���ݵĸ���
  float zero;                               // �����ֶ�
  float samples[AT_CAL_DATA_MAX_CNT];   // ����ֵ 
  float calibrs[AT_CAL_DATA_MAX_CNT];     // У׼ֵ 
  char  reserve[8];                      // ����
}SCalData;


extern SCalData CDataCalibration[AT_CAL_TYPE_MAX_CNT];

uint8_t fold_line(float *xa,float *ya,int n, const float x, float *y, EFoldLineType type);

float CalibrateData(SCalData *caldata, float fValue);

#endif