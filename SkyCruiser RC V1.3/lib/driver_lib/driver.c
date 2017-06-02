

//所有关于驱动文件的宏替换，硬件的初始化，都在driver.c / driver.h  文件中。
#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "driver.h"
#include "include.h"
#include "nrf24L01.h"
#include "key_exti.h"
#include "rocker.h"
#include "rc.h"
#include "timer.h"
#include "oled.h"
#include "bmp.h"


Flag_t flag;
sensor_val sensor;



//驱动初始化
void driver_init(void)
{
	Plane_Nvic_Init();
	usart_init(921600); 		
	delay_init(72);
	TIM2_Config();
	NRF24L01_Init();
	OLED_Init();			
	OLED_Clear();	
	EXTI_Key_init();  //unigress 红板上的独立按键 
	Rocker_EXTI_Key_init(); //摇杆按键
	rocker_adc_init();  // 摇杆四路 adc
}


void data_init(void)
{
	//初始化 检测遥控各功能部件
	flag.LockYaw = 1;
	flag.NrfExist = NRF24L01_Check();
	flag.OledExist = 1;
	flag.ParamSave = 1;
	flag.RockerExist = 1;
	
	//遥控上电初始化首次通信时赋予plane的各项数据
	rc_data.armed_val = 0;
	rc_data.throttle_val = 1100.0f;
	rc_data.pitch_val = 0.0f;
	rc_data.roll_val = 0.0f;
	rc_data.yaw_val = 0.0f;
	rc_data.sensitivity_val = 80.0f;
	
	//pid参数初始化
	paramLoad();
	
	//获取各个器件起始的偏移量
	
	
	//初始化 设置各个功能部件  
	NRF24L01_Mode(2);   //2 发送模式
}

// 将数据限制到 -8 - +8 之内 
static int real_data_limit(const int real_data)
{
	int limit_data = 0;
	
	limit_data = real_data;
	
	if(limit_data >= 8)
	{
		limit_data = 8;
	}else if(limit_data <= -8)
	{
		limit_data = -8;
	}
	return limit_data;
}
/* 
throttle: 通过rocker left/right key to change 
sensitivy: 通过EXTI8/9 改变 
*/
void data_reload(void)
{
	int throttle_temp = 0;
	int pitch_temp = 0;
	int roll_temp = 0;
	int yaw_temp = 0;
	
	//----------------------- throttle -------------rocker up/down
	throttle_temp = 100 * rocker_get_adc_val(ROCKER_ADC_CHANNEL1);
	rc_data.throttle_val = 1000 + ((259-throttle_temp)*2.7);
	
	//------------------------pitch --------------------rocker up/down
	pitch_temp = 10 * rocker_get_adc_val(ROCKER_ADC_CHANNEL3);
	rc_data.pitch_val = real_data_limit(pitch_temp - 24);
	
	//------------------------roll----------------------rocker left/right
	roll_temp = 10 * rocker_get_adc_val(ROCKER_ADC_CHANNEL4);
	rc_data.roll_val = real_data_limit(24 - roll_temp); 
	
	//-------------------------yaw ---------------------rocker left/right 
	yaw_temp = 10 * rocker_get_adc_val(ROCKER_ADC_CHANNEL2);
	rc_data.yaw_val = real_data_limit(25 - yaw_temp);
}

/*
1.先将外环全部置为0，调节内环 （这种情况只调节内环，打舵是没有反应的，只有加上外环P之后才会打舵响应）
2.逐步加大内环P, 直到系统开始振荡
3.稍微减少P，加大D，可以相对抑制系统抖动
4.加大I，可一定程度上减小系统累积角速度积分误差。

5.开始调节外环，I可有可无， 逐步加大P （P稍微加大一点就会导致飞机打舵灵敏度的提高，从1到4，灵敏度逐步提高）

Pitch 较好的参数：
cp 达到13时候剧烈抖动，12相对合适
sp  	si		cp		ci		cd
2.5		0		12.0	0.09	0.35
2.5		0		12.0	0		0.3
2.5		0		12.0	0		0.35
2.5		0		11.0	0		0.35

4.0		0.02	2.8		0.09	0.35    //扰动以后，基本没有反应，回复力很小

Roll 较好的参数：
sp		si		cp		ci		cd
1.5		0		8.5		0		0.9		//偏一点  比下面的8.7稳定 
1.5		0		8.7		0		0.9		//偏一点   
1.5		0		8.5		0.1		0.9
1.5 	0		8.5		0.09	0.9

1.5		0.02	2.0		0.09	0.35	 //扰动以后，基本没有反应，回复力很小

综合：
group1： 开始起飞效果不错， 起飞后失态
Pitch:  2.5		0		11.0	0		0.35
Roll:	1.5		0		8.5		0.09	0.9

*/
void paramLoad(void)
{
	/* 抖动相当大 */
	//pitch
	ctrl.pitch.shell.kp = 2.5; 
	ctrl.pitch.shell.ki = 0.0;
	ctrl.pitch.core.kp = 11.0;   
	ctrl.pitch.core.ki = 0.0;   
	ctrl.pitch.core.kd = 0.35;  
	
	//roll
	ctrl.roll.shell.kp = 1.5;
	ctrl.roll.shell.ki = 0.0;
	ctrl.roll.core.kp = 8.5;   
	ctrl.roll.core.ki = 0.09; 
	ctrl.roll.core.kd = 0.9;
	
	/* 回复里很小，平滑 */
	//pitch
//	ctrl.pitch.shell.kp = 2.5; 
//	ctrl.pitch.shell.ki = 0.02;
//	ctrl.pitch.core.kp = 2.8;   
//	ctrl.pitch.core.ki = 0.09;   
//	ctrl.pitch.core.kd = 0.35;  
//	
//	//roll
//	ctrl.roll.shell.kp = 1.5;
//	ctrl.roll.shell.ki = 0.02;
//	ctrl.roll.core.kp = 2.0;   
//	ctrl.roll.core.ki = 0.09; 
//	ctrl.roll.core.kd = 0.35;
	

	//yaw
	ctrl.yaw.shell.kp = 1.5;
	ctrl.yaw.shell.kd = 0.0;
	ctrl.yaw.core.kp = 1.8;
	ctrl.yaw.core.ki = 0.0;
	ctrl.yaw.core.kd = 0.1;
	
	//limit for the max increment
	ctrl.pitch.shell.increment_max = 20;
	ctrl.roll.shell.increment_max = 20;
	
	ctrl.ctrlRate = 0;
}



















