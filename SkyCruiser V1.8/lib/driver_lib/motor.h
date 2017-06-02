#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

#define MOTOR_MAX_PWM 2000
#define MOTOR_MIN_PWM 1000


void plane_start(u16 value); //��������
void plane_hover(u16 value); //��ͣ
void plane_stop(void); //ֹͣ  value=1000
void plane_pwm_reflash(s16 *motor);


void limit_pwm(u8 i, u16 value);
void limit_all_PWM(u16 *motor);

#endif





