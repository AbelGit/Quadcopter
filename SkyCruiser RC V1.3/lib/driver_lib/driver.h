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

//SPI 
#define SPI_RCC_PORT  RCC_APB2Periph_SPI1
#define SPI_PORT  SPI1
//NRF24L01+
#define NRF_RCC_PORT1  RCC_APB2Periph_GPIOA
#define NRF_RCC_PORT2  RCC_APB2Periph_GPIOA
#define NRF_RCC_PORT3  RCC_APB2Periph_GPIOA

#define NRF_IRQ_PORT  GPIOA
#define NRF_MISO_PORT  GPIOA
#define NRF_MOSI_PORT  GPIOA
#define NRF_SCK_PORT  GPIOA
#define NRF_CE_PORT  GPIOA
#define NRF_CSN_PORT  GPIOA

#define NRF_CSN_PIN  GPIO_Pin_1
#define NRF_CE_PIN  GPIO_Pin_2
#define NRF_IRQ_PIN  GPIO_Pin_3
#define NRF_SCK_PIN  GPIO_Pin_5
#define NRF_MISO_PIN  GPIO_Pin_6
#define NRF_MOSI_PIN  GPIO_Pin_7

//key  PD 8,9,10,11    
#define KEY_EXTI_PIN1	GPIO_Pin_8
#define KEY_EXTI_PIN2	GPIO_Pin_9
#define KEY_EXTI_PIN3	GPIO_Pin_10
#define KEY_EXTI_PIN4	GPIO_Pin_11
#define KEY_EXTI_PIN5	GPIO_Pin_0
#define KEY_EXTI_PIN6	GPIO_Pin_1
#define KEY_EXTI_PIN7	GPIO_Pin_2
#define KEY_EXTI_PIN8	GPIO_Pin_3
#define KEY_EXTI_PIN9	GPIO_Pin_4
#define KEY_EXTI_PORT	GPIOD
#define KEY_EXTI_LINE_PORT1	GPIO_PortSourceGPIOD
#define KEY_EXTI_LINE_PIN1	GPIO_PinSource8
#define KEY_EXTI_LINE_PIN2	GPIO_PinSource9
#define KEY_EXTI_LINE_PIN3	GPIO_PinSource10
#define KEY_EXTI_LINE_PIN4	GPIO_PinSource11
#define KEY_EXTI_LINE_PIN5	GPIO_PinSource0
#define KEY_EXTI_LINE_PIN6	GPIO_PinSource1
#define KEY_EXTI_LINE_PIN7	GPIO_PinSource2
#define KEY_EXTI_LINE_PIN8	GPIO_PinSource3
#define KEY_EXTI_LINE_PIN9	GPIO_PinSource4
#define KEY_EXTI_RCC_PORT1	RCC_APB2Periph_GPIOD
#define KEY_EXTI_LINE1	EXTI_Line8
#define KEY_EXTI_LINE2	EXTI_Line9
#define KEY_EXTI_LINE3	EXTI_Line10
#define KEY_EXTI_LINE4	EXTI_Line11

#define KEY_EXTI_LINE5	EXTI_Line0
#define KEY_EXTI_LINE6	EXTI_Line1
#define KEY_EXTI_LINE7	EXTI_Line2
#define KEY_EXTI_LINE8	EXTI_Line3
#define KEY_EXTI_LINE9	EXTI_Line4

//rocker  
//按键    PC 14, 15 
#define ROCKER_KEY_EXTI_PIN1	GPIO_Pin_14	
#define ROCKER_KEY_EXTI_PIN2	GPIO_Pin_15	
#define ROCKER_KEY_EXTI_PORT1  GPIOC
#define ROCKER_KEY_EXTI_LINE_PORT1	GPIO_PortSourceGPIOC
#define ROCKER_KEY_EXTI_LINE_PIN1	GPIO_PinSource14
#define ROCKER_KEY_EXTI_LINE_PIN2	GPIO_PinSource15
#define ROCKER_KEY_EXTI_RCC_PORT	RCC_APB2Periph_GPIOC
#define ROCKER_KEY_EXTI_LINE1		EXTI_Line14
#define ROCKER_KEY_EXTI_LINE2		EXTI_Line15
//ADC  PC 0,1,2,3    ADC1->10,11,12,13
#define ROCKER_ADC_RCC_PORT1	RCC_APB2Periph_GPIOC
#define ROCKER_ADC_PIN1		GPIO_Pin_0
#define ROCKER_ADC_PIN2		GPIO_Pin_1
#define ROCKER_ADC_PIN3		GPIO_Pin_2
#define ROCKER_ADC_PIN4		GPIO_Pin_3
#define ROCKER_ADC_PORT		GPIOC
#define ROCKER_ADC_CHANNEL1		ADC_Channel_10
#define ROCKER_ADC_CHANNEL2		ADC_Channel_11
#define ROCKER_ADC_CHANNEL3		ADC_Channel_12
#define ROCKER_ADC_CHANNEL4		ADC_Channel_13

// OLED IIC 接口    PB6,7
#define OLED_RCC_PORT 		RCC_APB2Periph_GPIOB
#define OLED_PORT 			GPIOB
#define OLED_SCL_PIN		GPIO_Pin_6
#define OLED_SDA_PIN 		GPIO_Pin_7



//--------------------------------Sensor data --------------------------------
typedef struct
{
	float x;
	float y;
	float z;
}float_val;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}int16_val;

typedef struct 
{
	 int16_val origin;  //原始值
	 float_val averag;  //平均值
	 float_val histor;  //历史值
	 int16_val quiet;   //静态值
	 float_val radian;  //弧度值 
}val;

typedef struct 
{   
	val acc;
	val gyro;
}sensor_val;
//---------------------------------device flag ---------------------------------

typedef struct 
{
	u8 OledExist;      // OLED存在 
	u8 RockerExist;      // rocker
	u8 NrfExist;      // NRF存在
	u8 LockYaw;       // 航向锁定       
	u8 ParamSave;     // 参数保存标志
}Flag_t;
//-------------------------------veriable--------------------------------------

extern Flag_t flag;
extern sensor_val sensor;

//-------------------------------func declare ----------------------------------
void driver_init(void);
void data_init(void);
void data_reload(void);   //tim5中每隔2ms update one time 
void paramLoad(void);

#endif 


