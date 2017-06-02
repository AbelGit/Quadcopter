

//���й��������ļ��ĺ��滻��Ӳ���ĳ�ʼ��������driver.c / driver.h  �ļ��С�
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



//������ʼ��
void driver_init(void)
{
	Plane_Nvic_Init();
	usart_init(921600); 		
	delay_init(72);
	TIM2_Config();
	NRF24L01_Init();
	OLED_Init();			
	OLED_Clear();	
	EXTI_Key_init();  //unigress ����ϵĶ������� 
	Rocker_EXTI_Key_init(); //ҡ�˰���
	rocker_adc_init();  // ҡ����· adc
}


void data_init(void)
{
	//��ʼ�� ���ң�ظ����ܲ���
	flag.LockYaw = 1;
	flag.NrfExist = NRF24L01_Check();
	flag.OledExist = 1;
	flag.ParamSave = 1;
	flag.RockerExist = 1;
	
	//ң���ϵ��ʼ���״�ͨ��ʱ����plane�ĸ�������
	rc_data.armed_val = 0;
	rc_data.throttle_val = 1100.0f;
	rc_data.pitch_val = 0.0f;
	rc_data.roll_val = 0.0f;
	rc_data.yaw_val = 0.0f;
	rc_data.sensitivity_val = 80.0f;
	
	//pid������ʼ��
	paramLoad();
	
	//��ȡ����������ʼ��ƫ����
	
	
	//��ʼ�� ���ø������ܲ���  
	NRF24L01_Mode(2);   //2 ����ģʽ
}

// ���������Ƶ� -8 - +8 ֮�� 
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
throttle: ͨ��rocker left/right key to change 
sensitivy: ͨ��EXTI8/9 �ı� 
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
1.�Ƚ��⻷ȫ����Ϊ0�������ڻ� ���������ֻ�����ڻ��������û�з�Ӧ�ģ�ֻ�м����⻷P֮��Ż�����Ӧ��
2.�𲽼Ӵ��ڻ�P, ֱ��ϵͳ��ʼ��
3.��΢����P���Ӵ�D�������������ϵͳ����
4.�Ӵ�I����һ���̶��ϼ�Сϵͳ�ۻ����ٶȻ�����

5.��ʼ�����⻷��I���п��ޣ� �𲽼Ӵ�P ��P��΢�Ӵ�һ��ͻᵼ�·ɻ���������ȵ���ߣ���1��4������������ߣ�

Pitch �ϺõĲ�����
cp �ﵽ13ʱ����Ҷ�����12��Ժ���
sp  	si		cp		ci		cd
2.5		0		12.0	0.09	0.35
2.5		0		12.0	0		0.3
2.5		0		12.0	0		0.35
2.5		0		11.0	0		0.35

4.0		0.02	2.8		0.09	0.35    //�Ŷ��Ժ󣬻���û�з�Ӧ���ظ�����С

Roll �ϺõĲ�����
sp		si		cp		ci		cd
1.5		0		8.5		0		0.9		//ƫһ��  �������8.7�ȶ� 
1.5		0		8.7		0		0.9		//ƫһ��   
1.5		0		8.5		0.1		0.9
1.5 	0		8.5		0.09	0.9

1.5		0.02	2.0		0.09	0.35	 //�Ŷ��Ժ󣬻���û�з�Ӧ���ظ�����С

�ۺϣ�
group1�� ��ʼ���Ч������ ��ɺ�ʧ̬
Pitch:  2.5		0		11.0	0		0.35
Roll:	1.5		0		8.5		0.09	0.9

*/
void paramLoad(void)
{
	/* �����൱�� */
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
	
	/* �ظ����С��ƽ�� */
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



















