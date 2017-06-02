
/**
  ******************************************************************************
  * @file    main.c
  * @author  Iron
  * @version V1.0
  * @date    2017/3/2
  * @attention    http://www.chuxue123.com   http://www.openedv.com  
  * @brief  
	1. 由于电源接线松动，造成了按键的时候oled显示数据都动的情况，将电源线插好就行。
	2. 串口助手坏掉，打出的字符就不正确了。
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

//oled 中itof 存在bug
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

	














