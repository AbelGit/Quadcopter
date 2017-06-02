#ifndef __CONTROL_H
#define __CONTROL_H

struct _pid
{
	float kp;
	float ki;
	float kd;
	float increment;
	float increment_max;
	float kp_out;
	float ki_out;
	float kd_out;
	float pid_out;
};

struct _tache
{
	struct _pid shell;
	struct _pid core;	
};

struct _ctrl
{
	u8  ctrlRate;
	struct _tache pitch;    
	struct _tache roll;  
	struct _tache yaw;   
};

struct _target
{
	float Pitch;    
	float Roll;  
	float Yaw;   
	int32_t Altiude; 
};

extern struct _ctrl ctrl;
extern struct _target Target;


void Calculate_Target(void);   //����Ŀ����̬��
void CONTROL(struct _target Goal);    //����pid����
void Attitude_RatePID(void);	//�ڻ�pid����
void motor_control(void);	//�������
void Reset_Integral(void);  // ��������

void paramLoad(void); //pid������ʼ�� 


#endif 






