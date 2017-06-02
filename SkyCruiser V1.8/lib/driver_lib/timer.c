/**
  ******************************************************************************
  * @file    timer.c
  * @author  Iron li 
  * @version V1.5
  * @date    2017/1/11
  * @attention    http://www.chuxue123.com   http://www.openedv.com  
  * @brief  
    1. 
	2. 
	3. 
  *****************************************************************************/ 

#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "timer.h"
#include "usart_report.h"
#include "control.h"
#include "driver.h"
#include "nrf24L01.h"
#include "rc.h"



//野火 定时器章节有对这个内容的详细讲解。


void Tim4_init(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  				TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	/**********************************************************
	72 000 000/72=1M
	1000 000/2500=400Hz
	所以产生的PWM为400Hz
	周期为2.5ms，对应2500的计算值，1ms~2ms对应的计算值为1000~2000；
	**********************************************************/
	TIM_TimeBaseStructure.TIM_Period = 2499;		//计数上线	
	TIM_TimeBaseStructure.TIM_Prescaler = 71;	//pwm时钟分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1000;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}

void PWM_OUT_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOA and GPIOC clock enable */
	RCC_APB2PeriphClockCmd(MOTOR_RCC_PORT, ENABLE); 

	GPIO_InitStructure.GPIO_Pin =  MOTOR1 | MOTOR2 | MOTOR3 | MOTOR4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MOTOR_PORT, &GPIO_InitStructure);

	Tim4_init();
}

/**************************实现函数********************************************
*函数原型:		
*功　　能:2ms中断一次,计数器为2000		
*******************************************************************************/
void TIM5_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//基础设置，时基和比较输出设置，由于这里只需定时，所以不用OC比较输出
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
	TIM_DeInit(TIM5);

	TIM_TimeBaseStructure.TIM_Period = 1999;//装载值
	//prescaler is 1200,that is 72000000/72/500=2000Hz;
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;//分频系数
	//set clock division 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
	//count up
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	//clear the TIM5 overflow interrupt flag
	TIM_ClearFlag(TIM5,TIM_FLAG_Update);
	//TIM5 overflow interrupt enable
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	//enable TIM5
	TIM_Cmd(TIM5,DISABLE);
}


void TIM5_IRQHandler(void)		    //2ms中断一次
{
	if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
	{
		get_euler_angles();
		Calculate_Target();
		CONTROL(Target);
		TIM_ClearITPendingBit (TIM5, TIM_IT_Update);
	}
}



















