#ifndef __EXTI_H
#define __EXTI_H
#include "stm8l15x_itc.h"
#include "stm8l15x_it.h"
#include "stm8l15x_exti.h"
#include "stm8l15x.h"
/* Private define ------------------------------------------------------------*/
#define SONIC_PORT  GPIOD
#define ECHO_PIN    GPIO_Pin_0
#define TRIG_PIN    GPIO_Pin_2

void exti_init(void);

#endif