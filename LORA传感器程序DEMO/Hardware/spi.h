#ifndef __SPI_H
#define __SPI_H
#include "stm8l15x.h"

#define   SPI_SCK_PIN               GPIO_Pin_5                  /* PB.05 */
#define   SPI_SCK_GPIO_PORT         GPIOB                       /* GPIOB */
#define   SPI_MISO_PIN              GPIO_Pin_7                  /* PB.07 */
#define   SPI_MISO_GPIO_PORT        GPIOB                       /* GPIOB */
#define   SPI_MOSI_PIN              GPIO_Pin_6                  /* PB.06 */
#define   SPI_MOSI_GPIO_PORT        GPIOB                       /* GPIOAB */
#define   SPI_CS_PIN                GPIO_Pin_4                  /* PB.04 */
#define   CS_GPIO_PORT              GPIOB                       /* GPIOB */

void SPI1_DeInit(void);
void SPI1_Init_Gpio(void);
void SPI1_Init(void);
uint8_t SPI1_SendByte(uint8_t byte);

#endif