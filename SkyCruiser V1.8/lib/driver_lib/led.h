#ifndef __LED_H
#define	__LED_H

#include "stm32f10x.h"

#define ON  0
#define OFF 1

#define LED1(a)	if (a)	\
					GPIO_SetBits(LED_PORT, LED1_Pin);\
					else		\
					GPIO_ResetBits(LED_PORT, LED1_Pin)

#define LED2(a)	if (a)	\
					GPIO_SetBits(LED_PORT, LED2_Pin);\
					else		\
					GPIO_ResetBits(LED_PORT, LED2_Pin)

#define LED3(a)	if (a)	\
					GPIO_SetBits(LED_PORT, LED3_Pin);\
					else		\
					GPIO_ResetBits(LED_PORT, LED3_Pin)

#define LED4(a) if(a) \
					GPIO_SetBits(LED_PORT, LED4_Pin);\
					else		\
					GPIO_ResetBits(LED_PORT, LED4_Pin)
					
					
void LED_Init(void);

#endif /* __LED_H */
