
  
#include "led.h"
#include "driver.h"

 /**
  * @brief  
  * @param  
  * @retval 
  */
void LED_Init(void)
{		
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(LED_RCC_PORT, ENABLE); 

	GPIO_InitStructure.GPIO_Pin = LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(LED_PORT, &GPIO_InitStructure);	
	
	GPIO_SetBits(LED_PORT, LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin);
}


