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


void Calculate_Target(void);   //计算目标姿态角
void CONTROL(struct _target Goal);    //串级pid控制
void Attitude_RatePID(void);	//内环pid控制
void motor_control(void);	//电机控制
void Reset_Integral(void);  // 积分清零

void paramLoad(void); //pid参数初始化 


#endif 






