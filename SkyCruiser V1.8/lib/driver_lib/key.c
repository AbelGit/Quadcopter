
#include "stm32f10x.h"
#include "key.h"
#include "sys.h" 
#include "delay.h"
#include "stdio.h"
#include "pin_use.h"


/*********************************** EXTI Key *********************************/
u16 pwm_value = 1100;

//  Key1 config 
static void key_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(KEY_RCC_PORT, ENABLE);

	GPIO_InitStructure.GPIO_Pin  = KEY_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(KEY_PORT, &GPIO_InitStructure); 
}

static void key_nvic_init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(KEY_NVIC_PRI_GROUP);
  
  /* 配置中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = KEY_IRQN;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = KEY_NVIC_PREEMPT_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = KEY_NVIC_SUB_PRI;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

static void key_exti_init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	RCC_APB2PeriphClockCmd(KEY_RCC_MODE_PORT, ENABLE);
											
	key_nvic_init();
	
	GPIO_EXTILineConfig(KEY_EXTI_LINE_PORT, KEY_EXTI_LINE_PIN);
	
	EXTI_InitStructure.EXTI_Line = KEY_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿中断
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure); 
}

//key1 exti config 
void key_init(void) 
{ 
 	key_gpio_init();
	key_exti_init();
}


void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) 
	{
		pwm_value += 10;
		plane_start(pwm_value);
		EXTI_ClearITPendingBit(EXTI_Line0);     //清除中断标志位
	}  
}

