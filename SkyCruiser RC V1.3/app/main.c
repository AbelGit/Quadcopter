
/**
  ******************************************************************************
  * @file    main.c
  * @author  Iron
  * @version V1.0
  * @date    2017/3/2
  * @attention    http://www.chuxue123.com   http://www.openedv.com  
  * @brief  
	1. ���ڵ�Դ�����ɶ�������˰�����ʱ��oled��ʾ���ݶ��������������Դ�߲�þ��С�
	2. �������ֻ�����������ַ��Ͳ���ȷ�ˡ�
	3. 
  *****************************************************************************/ 
#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "driver.h"
#include "nrf24L01.h"
#include "rc.h"
#include "rocker.h"
#include "delay.h"
#include "timer.h"
#include "oled.h"

//oled ��itof ����bug
//baud 921600

int main(void)
{	
	driver_init();
	data_init();
	
	OLED_Show_Start();
	
	while(1)
	{	
		OLED_Show_Sel();
		rc_control();	
	}
}

	














