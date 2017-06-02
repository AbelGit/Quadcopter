#ifndef __DRIVER_H
#define __DRIVER_H

#include "stm32f10x.h"

//usart
#define USART_RCC_PORT  RCC_APB2Periph_GPIOA
#define USART_RCC_MODE_PORT  RCC_APB2Periph_USART1
#define USART_Tx_Pin  GPIO_Pin_9
#define USART_Rx_Pin  GPIO_Pin_10
#define USART_PORT  GPIOA
#define USART_NAME  USART1


//tim4 B
#define MOTOR_RCC_PORT 	RCC_APB2Periph_GPIOB
#define MOTOR_PORT  GPIOB
#define TIMER	TIM4
#define MOTOR1  GPIO_Pin_6
#define MOTOR2  GPIO_Pin_7
#define MOTOR3  GPIO_Pin_8
#define MOTOR4  GPIO_Pin_9


//mpu 6050
#define MPU6050_RCC_PORT  RCC_APB2Periph_GPIOA
#define MPU6050_PORT  GPIOA
#define MPU6050_AD0  GPIO_Pin_15
#define MPU6050_RCC_IIC_PORT  RCC_APB2Periph_GPIOB
#define MPU6050_IIC_PORT  GPIOB
#define MPU6050_SCL_PIN  GPIO_Pin_10
#define MPU6050_SDA_PIN  GPIO_Pin_11

//SPI 
#define SPI_RCC_PORT  RCC_APB2Periph_SPI1
#define SPI_PORT  SPI1

//NRF24L01+
#define NRF_RCC_PORT1  RCC_APB2Periph_GPIOA
#define NRF_RCC_PORT2  RCC_APB2Periph_GPIOC
#define NRF_RCC_PORT3  RCC_APB2Periph_GPIOG

#define NRF_IRQ_PORT  GPIOC
#define NRF_MISO_PORT  GPIOA
#define NRF_MOSI_PORT  GPIOA
#define NRF_SCK_PORT  GPIOA
#define NRF_CE_PORT  GPIOG
#define NRF_CSN_PORT  GPIOG

#define NRF_IRQ_PIN  GPIO_Pin_4
#define NRF_SCK_PIN  GPIO_Pin_5
#define NRF_MISO_PIN  GPIO_Pin_6
#define NRF_MOSI_PIN  GPIO_Pin_7
#define NRF_CE_PIN  GPIO_Pin_8
#define NRF_CSN_PIN  GPIO_Pin_15



void driver_init(void);
void data_init(void);


#endif 


