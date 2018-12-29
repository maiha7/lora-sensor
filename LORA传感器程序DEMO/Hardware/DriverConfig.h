// ��������
#ifndef __DY_DRIVER_CONFIG_H__
#define __DY_DRIVER_CONFIG_H__
#include "stm8l15x.h"

//�͹���ģʽ״̬��־
extern uint8_t g_LowPowerCntState;
extern uint16_t acData;
//�͹��Ŀ��ƺ�
#define  LOW_POWER_CNT_STATE_HALT    1 //ͣ��ģʽ
#define  LOW_POWER_CNT_STATE_ACTIVE  0 //����״̬

typedef enum
{
  SampleRead_State = 0,//������ȡ״̬
  DataSend_State, //���ݷ���״̬
  GotoHalt_State //����ͣ��״̬
}EDriverState;

extern EDriverState  eDriverState;
//AD
extern void DrADC_Init(void);
extern float ADC_NH3_Read(void);
//GPIO
void DrGPIO_Init(void);
void DrGPIO_Write(int index, int value);
int GPIO_Read(int index);
//UART
void UART_Init(void);
int UART_Write(void* p, int len);
int UART_Read(void* p, int len);
//ROM
void ROM_Init(void);
int ROM_Write(int addr, void* p, int len);
int ROM_Read(int addr, void* p, int len);


void Comm_Run(int ms);
// bsp��ʼ��
extern void BspInit(void);
// ��������
extern void DriverConfig(void);
// ��������, ms:����ʱ��(����)
extern void DriverTask(int ms);
// ʱ���ӳ�, ms:�ӳ�ʱ��(����)
extern void TimeDelay(int ms);
// ģ����˯��(ʵ��������ʵ�ֺ���Ϊ��), ms:˯��ʱ��(����)
extern  void SimSleep(uint8_t LowPowerCnt_State);
// �õ�������
extern uint32_t SysTimeGetms(void);
// ������Ź���־
extern void  Clear_IWDG_Flag(void);
// ���ϵͳ������ʼ��
extern void SoftwareInit(void);
// ϵͳ����͹���
extern void LowPowerConsumption_Cmd(uint8_t LowPowerCnt_State);  
#endif // __DY_DRIVER_CONFIG_H__
