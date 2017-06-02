#ifndef __RC_H
#define __RC_H

#include <stdio.h>
#include "stm32f10x.h"
#include "sys.h" 

/*------------------------------ÓÍÃÅ¼ì²é-------------------------------------*/
#define RC_MINCHECK   1140
#define RC_MAXCHECK   1800

/*------------------------------ data size -----------------------------*/
#define DATA_SIZE 4

/*----------------------------device ------------------------*/
typedef enum 
{
	ARMED = 0x30,   //½âËø±êÖ¾   0x30 = ×Ö·û0
	THROTTLE,		//ÓÍÃÅ    0x31 = ×Ö·û1
	PITCH,       //×Ö·û2
	ROLL,		//×Ö·û3
	YAW,			//×Ö·û4
	SENSITIVITY,	//ÁéÃô¶È	×Ö·û5
	CMD_EMPTY,		//×Ö·û6
}RC;

typedef enum
{
	//-------Roll 
	ROLL_CP = 0x37,    //roll_core_kp
	ROLL_CI,		//roll_core_ki
	ROLL_CD,		//roll_core_kd
	ROLL_SP,		//roll_shell_kp
	ROLL_SI,		//roll_shell_ki
	//------Pitch
	PITCH_CP = 0x42,
	PITCH_CI,
	PITCH_CD,
	PITCH_SP,
	PITCH_SI,
	//-----Yaw
	YAW_CP = 0x47,
	YAW_CI,
	YAW_CD,
	YAW_SP,
	YAW_SI,
	
}CS_PID;

/*-------------------------device data ------------------*/
typedef struct
{
	float armed_val;
	float throttle_val;
	float pitch_val;
	float roll_val;
	float yaw_val;
	float sensitivity_val;
}_RC_DATA;
extern _RC_DATA rc_data; 

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


/*------------------------------func declare-------------------------*/
void rc_control(void);


#endif














