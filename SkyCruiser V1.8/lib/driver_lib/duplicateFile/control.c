
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
extern float yaw; 	 //dmp ½âËãºóµÄ×ËÌ¬½Ç

struct _ctrl ctrl;
struct _target Target;

s16 Moto_duty[4];
s16 *motor_array = Moto_duty;

/**
  * @brief  cal_target
  * @param  no
  * @retval no
  */
void cal_target(void)
{	
	Target.Pitch = rc_data.pitch_val + 1.0f;
	Target.Roll = rc_data.roll_val - 1.0f;
	
	//Ä¿±êº½Ïò¿ØÖÆ¡£µ±ÓÍÃÅ´óÓÚ×îÐ¡¼ì²éÖµÊ±£¬ÈÏÎªÓÃ»§Ï£ÍûÆð·É¡£ÄÇÃ´´ËÊ±µÄº½Ïò×öÎªÄ¿±êº½Ïò
	if(rc_data.throttle_val > RC_MINCHECK ) 
	{
		if(flag.LockYaw != 1)
		{  
			flag.LockYaw = 1;
			Target.Yaw = yaw; //½«µ±Ç°µÄº½Ïò×öÎªÄ¿±êº½Ïò
		}
	}
	else 
	{
		flag.LockYaw = 0;	
		Target.Yaw = yaw;
	} 
}

/**
  * @brief  in_loop
  * @param  no
  * @retval no
  */
static void core_loop(void)
{
	float E_pitch = 0.0f;
	float E_roll = 0.0f;
	float E_yaw = 0.0f;

	//Æ«²î½ÇËÙ¶È  =  Íâ»·Êä³öÁ¿ÎªÆÚÍû½ÇËÙ¶È - ´ËÊ±µÄÏµÍ³½ÇËÙ¶È£¨¾ùÖµ£© 
	E_roll  = ctrl.roll.shell.pid_out  - sensor.gyro.averag.x;
	E_pitch = ctrl.pitch.shell.pid_out - sensor.gyro.averag.y;  
	E_yaw   = ctrl.yaw.shell.pid_out   - sensor.gyro.averag.z;

	// »ý·Ö
	ctrl.roll.core.increment  += E_roll;
	ctrl.pitch.core.increment += E_pitch;
	ctrl.yaw.core.increment   += E_yaw;
	
	// »ý·ÖÏÞ·ù
	ctrl.pitch.core.increment = data_limit(ctrl.pitch.core.increment,20,-20);
	ctrl.roll.core.increment  = data_limit(ctrl.roll.core.increment,20,-20);		
	ctrl.yaw.core.increment   = data_limit(ctrl.yaw.core.increment,20,-20);
	
	//±ÈÀýÏî
	ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * E_pitch;
	ctrl.roll.core.kp_out  = ctrl.roll.core.kp  * E_roll;
	ctrl.yaw.core.kp_out   = ctrl.yaw.core.kp   * E_yaw;

	//»ý·ÖÏî
	ctrl.pitch.core.ki_out = ctrl.pitch.core.ki * ctrl.pitch.core.increment;
	ctrl.roll.core.ki_out  = ctrl.roll.core.ki  * ctrl.roll.core.increment;
	ctrl.yaw.core.ki_out   = ctrl.yaw.core.ki   * ctrl.yaw.core.increment;

	// Î¢·ÖÏî
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

static void shell_loop(struct _target Goal)
{
	float deviation_pitch = 0.0f;
	float deviation_roll = 0.0f;
	float deviation_yaw = 0.0f;
	
	//---------------Íâ»·(½Ç¶È»·)PI   Íâ»·PIÊä³öµÄÁ¿ÊÇÆÚÍû½ÇËÙ¶È -----------
	//pitch
	deviation_pitch = Goal.Pitch - pitch;   //×îÖÕÄ¿µÄ½«Æ«²î¼õÎª0
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

/**
  * @brief  
  * @param  no
  * @retval no
  */
static void reset_integral(void)
{
	ctrl.pitch.shell.increment = 0;
	ctrl.roll.shell.increment= 0;	
	ctrl.pitch.core.increment = 0;		
	ctrl.roll.core.increment = 0;		
	ctrl.yaw.core.increment = 0;
}

/**
  * @brief  pid ¶Ô4¸öµç»úµÄ¿ØÖÆ¡£
  * @param  no
  * @retval no
  */
static void motor_control(void)
{
	s16 pitch_tho, roll_tho, yaw_tho;

	pitch_tho = ctrl.pitch.core.pid_out;
	roll_tho  = ctrl.roll.core.pid_out;    
	yaw_tho   = -ctrl.yaw.core.pid_out;

	if(rc_data.throttle_val > RC_MINCHECK) 
	{
		//hover_tho ÎªÐüÍ£ÓÍÃÅ 
		int hover_tho	= (rc_data.throttle_val-1000)/cos(roll/RtA)/cos(pitch/RtA);

		// 0,1,2,3 ¶ÔÓ¦1,2,3,4 ËÄ¸öµç»ú£¬´Ó»úÍ·¿ªÊ¼Ë³Ê±Õë¼ÆÊý¡£ »úÉí·½Ïò1,3 ³á°ò 2,4
		// I ÐÍËÄÖá£¬ µç»úthoÅäÖÃ 
		Moto_duty[0] = hover_tho - pitch_tho - yaw_tho;
		Moto_duty[1] = hover_tho - roll_tho + yaw_tho;
		Moto_duty[2] = hover_tho + pitch_tho - yaw_tho;
		Moto_duty[3] = hover_tho + roll_tho + yaw_tho;
	}else
	{	
		array_assign(&Moto_duty[0],IDLING, 4);  //Moto_duty[i] = 140 
		reset_integral();	//Çå³ý»ý·Ö	
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


/**
  * @brief  control
  * @param  no
  * @retval no
  */
void control(struct _target Goal)   
{
	//core:shell = 2:1   ÄÚÍâ»·¿ØÖÆÆµÂÊ 
	if(ctrl.ctrlRate >= 2)
	{
		shell_loop(Goal);
	}
	ctrl.ctrlRate++;

	AHRS_getValues();
	core_loop();  
	motor_control();
	
	printf("\r\n--------------------------\r\n");
	printf("\r\n sp: %f \r\n", ctrl.roll.shell.kp);
	printf("\r\n si: %f \r\n", ctrl.roll.shell.ki);
	printf("\r\n cp: %f \r\n", ctrl.roll.core.kp);
	printf("\r\n ci: %f \r\n", ctrl.roll.core.ki);
	printf("\r\n cd: %f \r\n", ctrl.roll.core.kd);
	printf("\r\n--------------------------\r\n");
}

//Æð·ÉÓÍÃÅ 1624×óÓÒ 

