#include <stdio.h>

#include "stm32f10x.h"
#include "timer.h"
#include "driver.h"
#include "rc.h"
#include "oled.h"


/**************************ʵ�ֺ���********************************************
*����ԭ��:		
*��������:2ms�ж�һ��,������Ϊ2000		
*******************************************************************************/
void TIM2_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//�������ã�ʱ���ͱȽ�������ã���������ֻ�趨ʱ�����Բ���OC�Ƚ����
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_DeInit(TIM2);

	TIM_TimeBaseStructure.TIM_Period = 1999;//װ��ֵ
	//prescaler is 1200,that is 72000000/72/500=2000Hz;
	TIM_TimeBaseStructure.TIM_Prescaler = 71;//��Ƶϵ��
	//set clock division 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
	//count up
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	//clear the TIM5 overflow interrupt flag
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);

	//TIM2 overflow interrupt enable
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void)		    //2ms�ж�һ��
{	
	if(TIM2->SR & TIM_IT_Update)	
	{    
		data_reload();
		TIM2->SR = ~TIM_FLAG_Update;  //����жϱ�־
	}
}






















