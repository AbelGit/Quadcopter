#ifndef __RC_H
#define __RC_H

#include "stm32f10x.h"

/*----------------油门检查----------------------*/
#define RC_MINCHECK   1100    //这个值的设定至关重要， 搞不好就飞不动了 
#define RC_MAXCHECK   1800

/*----------------电机怠速----------------------*/
#define IDLING 140

#define DATA_SIZE 4

/*----------------device--------------------*/
typedef enum 
{
	ARMED = 0x30,   //解锁标志   0x30 = 字符0
	THROTTLE,		//油门    0x31 = 字符1
	PITCH,       //字符2
	ROLL,		//字符3
	YAW,			//字符4
	SENSITIVITY,	//灵敏度	字符5
	CMD_EMPTY,		//字符6
}RC;

typedef enum
{
	//--------Roll
	ROLL_CP = 0x37,    //roll_core_kp    	字符7
	ROLL_CI,		//roll_core_ki			字符8
	ROLL_CD,		//roll_core_kd			字符9
	ROLL_SP,		//roll_shell_kp			字符10
	ROLL_SI,		//roll_shell_ki			字符11	
	//-------Pitch
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


/*---------device data ---------*/
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


// parse data 
void parse_recv_data(void);
//euler
void exec_module_armed(void);
void exex_module_throttle(void);
void exex_module_pitch(void);
void exex_module_roll(void);
void exex_module_yaw(void);
void exex_module_sensitivy(void);
//roll_cs_pid
void exec_module_roll_cp(void);
void exec_module_roll_ci(void);
void exec_module_roll_cd(void);
void exec_module_roll_sp(void);
void exec_module_roll_si(void);
//pitch_cs_pid
void exec_module_pitch_cp(void);
void exec_module_pitch_ci(void);
void exec_module_pitch_cd(void);
void exec_module_pitch_sp(void);
void exec_module_pitch_si(void);
//yaw_cs_pid
void exec_module_yaw_cp(void);
void exec_module_yaw_ci(void);
void exec_module_yaw_cd(void);
void exec_module_yaw_sp(void);
void exec_module_yaw_si(void);

#endif





