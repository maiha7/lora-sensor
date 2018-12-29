#ifndef __LED_H
#define __LED_H
#include "stm8l15x.h"
/* Private define ------------------------------------------------------------*/
#define LED_PORT    GPIOA
#define LED_PIN     GPIO_Pin_4

#define LED_OUT     GPIO_Init(LED_PORT,LED_PIN, GPIO_Mode_Out_PP_High_Fast) 
#define LED_LOW     GPIO_ResetBits(LED_PORT, LED_PIN)
#define LED_HIGH    GPIO_SetBits(LED_PORT, LED_PIN)
//extern u8 chipID[12];
//extern char devID_str[50];
void LED_Init(void);
void Led_Toggle(void);
void Led_Off(void);
void Led_On(void);
void delay_us(u16 num);
void delay_ms(u16 num);
void delay_s(u16 num);
void GetchipID(u8 *chipID);

#endif

