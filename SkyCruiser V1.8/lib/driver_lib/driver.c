

//所有关于驱动文件的宏替换，硬件的初始化，都在driver.c / driver.h  文件中。
#include <stdio.h>

#include "stm32f10x.h"
#include "stm32f10x_it.h"

#include "driver.h"
#include "timer.h"
#include "mpu6050.h"
#include "nrf24L01.h"
#include "rc.h"
#include "usart.h"
#include "delay.h"
#include "mpuiic.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "control.h"

//驱动初始化
void driver_init(void)
{
	Plane_Nvic_Init();
	usart_init(921600); 		
	delay_init(72);
	TIM5_Config();
	PWM_OUT_Config();
	NRF24L01_Init();

	flag.MpuExist = !MPU_Init();
	mpu_dmp_init();			// 加载并使能 DMP 库
}

//各数据的初始化
void data_init(void)
{
	//开机数据加载，检测器件正常。
	flag.ARMED = 0;
	flag.MagExist = 0;
	flag.MagIssue = 1;
	flag.ParamSave = 1;
	flag.calibratingA = 1;
	flag.NrfExist = NRF24L01_Check();

	//euler data init
	rc_data.armed_val = 0;
	rc_data.yaw_val = 0;
	rc_data.roll_val = 0;
	rc_data.pitch_val = 0;
	rc_data.throttle_val = 1100;
	rc_data.sensitivity_val = 80;
	
	//采集飞机静态偏移量 
	
	
	//加载并初始化pid参数，内外环
	paramLoad(); 
	
	//工作模式设定
	NRF24L01_Mode(1);   // 1. 接收模式，2. 发送模式，3. 接收模式2（全双工）4. 发送模式2（全双工）
}




