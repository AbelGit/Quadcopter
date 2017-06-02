/**
  ******************************************************************************
  * @file    motor.c
  * @author  Iron li 
  * @version V1.5
  * @date    2017/1/11
  * @attention    http://www.chuxue123.com   http://www.openedv.com  
  * @brief  
	1. 使用PWM 对电机进行速度控制.
	2. 
	3. 
	
  *****************************************************************************/ 
  
#include "motor.h"
#include "stm32f10x.h"
#include "driver.h"

void plane_start(u16 value) //启动飞行  value=1140
{
	TIMER->CCR1 = value;
	TIMER->CCR2 = value;
	TIMER->CCR3 = value;
	TIMER->CCR4 = value;
}

void plane_hover(u16 value) //悬停
{}
void plane_stop(void) //停止  value=1000
{
	TIMER->CCR1 = 1000;
	TIMER->CCR2 = 1000;
	TIMER->CCR3 = 1000;
	TIMER->CCR4 = 1000;
}

void plane_pwm_reflash(s16 *motor)
{
	u8 i;	
	for(i=0; i<4; i++)
	{
		if(*(motor+i) > MOTOR_MAX_PWM)  *(motor+i) = MOTOR_MAX_PWM;
	}
	for(i=0;i<4;i++)
	{
		if(*(motor+i) <= 0 )  *(motor+i) = 0;
	}

	TIMER->CCR1 = 1000 + *motor;
	TIMER->CCR2 = 1000 + *(motor+1);
	TIMER->CCR3 = 1000 + *(motor+2);
	TIMER->CCR4 = 1000 + *(motor+3);
}


void limit_pwm(u8 i, u16 value)
{
	if(value > MOTOR_MAX_PWM)
	{
		value = MOTOR_MAX_PWM;
	}
	if(value < MOTOR_MIN_PWM)
	{
		value = MOTOR_MIN_PWM;
	}
}

void limit_all_PWM(u16 *motor)
{
	uint8_t i;

    for (i = 0; i < 4; i++)
	{
		limit_pwm(i, motor[i]);
	}
}




























