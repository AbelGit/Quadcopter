

//���й��������ļ��ĺ��滻��Ӳ���ĳ�ʼ��������driver.c / driver.h  �ļ��С�
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

//������ʼ��
void driver_init(void)
{
	Plane_Nvic_Init();
	usart_init(921600); 		
	delay_init(72);
	TIM5_Config();
	PWM_OUT_Config();
	NRF24L01_Init();

	flag.MpuExist = !MPU_Init();
	mpu_dmp_init();			// ���ز�ʹ�� DMP ��
}

//�����ݵĳ�ʼ��
void data_init(void)
{
	//�������ݼ��أ��������������
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
	
	//�ɼ��ɻ���̬ƫ���� 
	
	
	//���ز���ʼ��pid���������⻷
	paramLoad(); 
	
	//����ģʽ�趨
	NRF24L01_Mode(1);   // 1. ����ģʽ��2. ����ģʽ��3. ����ģʽ2��ȫ˫����4. ����ģʽ2��ȫ˫����
}




