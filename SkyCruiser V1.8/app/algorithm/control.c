
#include <stdio.h>
#include <math.h>
#include "my_math.h"
#include "stm32f10x.h"
#include "control.h"
#include "rc.h"
#include "mpu6050.h"
#include "usart_report.h"
#include "ahrs.h"
#include "motor.h"

extern float pitch;
extern float roll;
extern float yaw; 	 //dmp ��������̬��

struct _ctrl ctrl;
struct _target Target;

s16 Moto_duty[4];
s16 *motor_array = Moto_duty;

/*====================================================================================================*/
/*====================================================================================================*
**���� : Calculate_target
**���� : ����Ŀ����
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Calculate_Target(void)
{	
	Target.Pitch = rc_data.pitch_val + 1.0f;
	Target.Roll = rc_data.roll_val - 1.0f;
	
	//Ŀ�꺽����ơ������Ŵ�����С���ֵʱ����Ϊ�û�ϣ����ɡ���ô��ʱ�ĺ�����ΪĿ�꺽��
	if(rc_data.throttle_val > RC_MINCHECK ) 
	{
		if(flag.LockYaw != 1)
		{  
			flag.LockYaw = 1;
			Target.Yaw = yaw; //����ǰ�ĺ�����ΪĿ�꺽��
		}
	}
	else 
	{
		flag.LockYaw = 0;	
		Target.Yaw = yaw;
	} 
}



/*====================================================================================================*/
/*====================================================================================================*
**���� : CONTROL(struct _target Goal) 
**���� : ����PID ��?
**���� : Goal
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void CONTROL(struct _target Goal)   
{
	
	float  deviation_pitch,deviation_roll,deviation_yaw;  // erler ƫ�� 
	
	//�ڻ�����2�ο��ƣ��⻷����1�ο��ƣ��ڻ�����Ƶ��Ϊ�⻷��2�� 
	if(ctrl.ctrlRate >= 2)
	{
		//---------------�⻷(�ǶȻ�)PI   �⻷PI����������������ٶ� -----------
		//pitch
		deviation_pitch = Goal.Pitch - pitch;   //����Ŀ�Ľ�ƫ���Ϊ0
		ctrl.pitch.shell.increment += deviation_pitch;
		ctrl.pitch.shell.increment = data_limit(ctrl.pitch.shell.increment,ctrl.pitch.shell.increment_max,-ctrl.pitch.shell.increment_max);
		ctrl.pitch.shell.pid_out = ctrl.pitch.shell.kp * deviation_pitch + ctrl.pitch.shell.ki * ctrl.pitch.shell.increment;

		//roll
		deviation_roll = Goal.Roll - roll;
		ctrl.roll.shell.increment += deviation_roll;
		ctrl.roll.shell.increment = data_limit(ctrl.roll.shell.increment,ctrl.roll.shell.increment_max,-ctrl.roll.shell.increment_max);
		ctrl.roll.shell.pid_out  = ctrl.roll.shell.kp * deviation_roll + ctrl.roll.shell.ki * ctrl.roll.shell.increment;

		//yaw
		if((Goal.Yaw - yaw)>180 || (Goal.Yaw - yaw)<-180)
		{
			if(Goal.Yaw>0 && yaw<0)  deviation_yaw= (-180 - yaw) +(Goal.Yaw - 180);
			if(Goal.Yaw<0 && yaw>0)  deviation_yaw= (180 - yaw) +(Goal.Yaw + 180);
		}
		else 
			deviation_yaw = Goal.Yaw - yaw;
			ctrl.yaw.shell.pid_out = ctrl.yaw.shell.kp * deviation_yaw;

			ctrl.ctrlRate = 0; 
	}
	ctrl.ctrlRate ++;
	
	//-------------------------�ڻ��������ʻ����PPID ---------------------------
	AHRS_getValues();
	Attitude_RatePID();  
	
	//-------------------------���Ƶ��-----------------------------------------
	motor_control();
	
//	printf("\r\n------------Roll--------------\r\n");
//	printf("\r\n sp: %f \r\n", ctrl.roll.shell.kp);
//	printf("\r\n si: %f \r\n", ctrl.roll.shell.ki);
//	printf("\r\n cp: %f \r\n", ctrl.roll.core.kp);
//	printf("\r\n ci: %f \r\n", ctrl.roll.core.ki);
//	printf("\r\n cd: %f \r\n", ctrl.roll.core.kd);
//	printf("\r\n--------------------------\r\n");
	
//	printf("\r\n------------PITCH--------------\r\n");
//	printf("\r\n sp: %f \r\n", ctrl.pitch.shell.kp);
//	printf("\r\n si: %f \r\n", ctrl.pitch.shell.ki);
//	printf("\r\n cp: %f \r\n", ctrl.pitch.core.kp);
//	printf("\r\n ci: %f \r\n", ctrl.pitch.core.ki);
//	printf("\r\n cd: %f \r\n", ctrl.pitch.core.kd);
//	printf("\r\n--------------------------\r\n");
}


/*====================================================================================================*/
/*====================================================================================================*
**���� : Attitude_RatePID
**���� : �����ʿ���PID
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Attitude_RatePID(void)
{
	float E_pitch,E_roll,E_yaw;

	// ����ƫ��  
	E_pitch = ctrl.pitch.shell.pid_out - sensor.gyro.averag.y;  //�������ٶ� - IMUgyroƽ��ֵ
	E_roll  = ctrl.roll.shell.pid_out  - sensor.gyro.averag.x;
	E_yaw   = ctrl.yaw.shell.pid_out   - sensor.gyro.averag.z;

	// ����
	ctrl.pitch.core.increment += E_pitch;
	ctrl.roll.core.increment  += E_roll;
	ctrl.yaw.core.increment   += E_yaw;
	
	// �����޷�
	ctrl.pitch.core.increment = data_limit(ctrl.pitch.core.increment,20,-20);
	ctrl.roll.core.increment  = data_limit(ctrl.roll.core.increment,20,-20);		
	ctrl.yaw.core.increment   = data_limit(ctrl.yaw.core.increment,20,-20);
	
	//������
	ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * E_pitch;
	ctrl.roll.core.kp_out  = ctrl.roll.core.kp  * E_roll;
	ctrl.yaw.core.kp_out   = ctrl.yaw.core.kp   * E_yaw;

	//������
	ctrl.pitch.core.ki_out = ctrl.pitch.core.ki * ctrl.pitch.core.increment;
	ctrl.roll.core.ki_out  = ctrl.roll.core.ki  * ctrl.roll.core.increment;
	ctrl.yaw.core.ki_out   = ctrl.yaw.core.ki   * ctrl.yaw.core.increment;

	// ΢����
	ctrl.pitch.core.kd_out = ctrl.pitch.core.kd * (sensor.gyro.histor.y - sensor.gyro.averag.y)*33;
	ctrl.roll.core.kd_out  = ctrl.roll.core.kd  * (sensor.gyro.histor.x - sensor.gyro.averag.x)*33;
	ctrl.yaw.core.kd_out   = ctrl.yaw.core.kd   * (sensor.gyro.histor.z - sensor.gyro.averag.z)*33;	

	sensor.gyro.histor.y = sensor.gyro.averag.y;
	sensor.gyro.histor.x = sensor.gyro.averag.x; 
	sensor.gyro.histor.z = sensor.gyro.averag.z;	

	ctrl.pitch.core.pid_out = ctrl.pitch.core.kp_out + ctrl.pitch.core.ki_out + ctrl.pitch.core.kd_out;
	ctrl.roll.core.pid_out  = ctrl.roll.core.kp_out  + ctrl.roll.core.ki_out  + ctrl.roll.core.kd_out;
	ctrl.yaw.core.pid_out   = ctrl.yaw.core.kp_out   + ctrl.yaw.core.kd_out;

	// pid out 
	ctrl.pitch.core.pid_out = ctrl.pitch.core.pid_out*0.8 + ctrl.pitch.shell.pid_out/2;
	ctrl.roll.core.pid_out  = ctrl.roll.core.pid_out *0.8 + ctrl.roll.shell.pid_out/2; 
	ctrl.yaw.core.pid_out   = ctrl.yaw.core.pid_out;
}


//pid ��4������Ŀ��ơ�
void motor_control(void)
{
	s16 pitch_tho, roll_tho, yaw_tho;

	pitch_tho = ctrl.pitch.core.pid_out;
	roll_tho  = ctrl.roll.core.pid_out;    
	yaw_tho   = -ctrl.yaw.core.pid_out;

	if(rc_data.throttle_val > RC_MINCHECK) 
	{
		//hover_tho Ϊ��ͣ���� 
		int hover_tho	= (rc_data.throttle_val-1000)/cos(roll/RtA)/cos(pitch/RtA);

		// 0,1,2,3 ��Ӧ1,2,3,4 �ĸ�������ӻ�ͷ��ʼ˳ʱ������� ������1,3 ��� 2,4
		// I �����ᣬ ���tho���� 
		Moto_duty[0] = hover_tho - pitch_tho - yaw_tho;
		Moto_duty[1] = hover_tho - roll_tho + yaw_tho;
		Moto_duty[2] = hover_tho + pitch_tho - yaw_tho;
		Moto_duty[3] = hover_tho + roll_tho + yaw_tho;
	}else
	{	
		array_assign(&Moto_duty[0],IDLING, 4);  //Moto_duty[i] = 140 
		Reset_Integral();	//�������	
	}
	
	if(flag.ARMED)
	{
		plane_pwm_reflash(Moto_duty);	// 1000+210=1140
	}
	else
	{
		plane_stop();
	}			
}


/*====================================================================================================*/
/*====================================================================================================*
**���� : Reset_Integral
**���� : ��������
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Reset_Integral(void)
{
	ctrl.pitch.shell.increment = 0;
	ctrl.roll.shell.increment= 0;	
	ctrl.pitch.core.increment = 0;		
	ctrl.roll.core.increment = 0;		
	ctrl.yaw.core.increment = 0;
}

void paramLoad(void)
{
	//pitch
	ctrl.pitch.shell.kp = 0.0;    
	ctrl.pitch.shell.ki = 0.0;
	ctrl.pitch.core.kp = 0.0;   
	ctrl.pitch.core.ki = 0.0;   
	ctrl.pitch.core.kd = 0.0;  
	
	//roll
	ctrl.roll.shell.kp = 0;
	ctrl.roll.shell.ki = 0;
	ctrl.roll.core.kp = 0;
	ctrl.roll.core.ki = 0;
	ctrl.roll.core.kd = 0;
	
	//yaw
	ctrl.yaw.shell.kp = 0;
	ctrl.yaw.shell.kd = 0;
	ctrl.yaw.core.kp = 0;
	ctrl.yaw.core.ki = 0;
	ctrl.yaw.core.kd = 0;
	
	//limit for the max increment
	ctrl.pitch.shell.increment_max = 20;
	ctrl.roll.shell.increment_max = 20;
	
	ctrl.ctrlRate = 0;
	
//	EE_READ_ACC_OFFSET();   //��ȡ���ٶ���ƫ
//	EE_READ_MAG_OFFSET();   //��ȡ��������ƫ
//	EE_READ_Attitude_PID(); //��ȡ�ڻ�PID����
//	Gyro_OFFSET();          //�ɼ���������ƫ
}





