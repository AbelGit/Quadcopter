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



//Ұ�� ��ʱ���½��ж�������ݵ���ϸ���⡣


void Tim4_init(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  				TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	/**********************************************************
	72 000 000/72=1M
	1000 000/2500=400Hz
	���Բ�����PWMΪ400Hz
	����Ϊ2.5ms����Ӧ2500�ļ���ֵ��1ms~2ms��Ӧ�ļ���ֵΪ1000~2000��
	**********************************************************/
	TIM_TimeBaseStructure.TIM_Period = 2499;		//��������	
	TIM_TimeBaseStructure.TIM_Prescaler = 71;	//pwmʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//���ϼ���
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MOTOR_PORT, &GPIO_InitStructure);

	Tim4_init();
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		
*��������:2ms�ж�һ��,������Ϊ2000		
*******************************************************************************/
void TIM5_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//�������ã�ʱ���ͱȽ�������ã���������ֻ�趨ʱ�����Բ���OC�Ƚ����
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
	TIM_DeInit(TIM5);

	TIM_TimeBaseStructure.TIM_Period = 1999;//װ��ֵ
	//prescaler is 1200,that is 72000000/72/500=2000Hz;
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;//��Ƶϵ��
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


void TIM5_IRQHandler(void)		    //2ms�ж�һ��
{
	if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
	{
		get_euler_angles();
		Calculate_Target();
		CONTROL(Target);
		TIM_ClearITPendingBit (TIM5, TIM_IT_Update);
	}
}



















