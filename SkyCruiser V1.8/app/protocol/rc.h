#ifndef __RC_H
#define __RC_H

#include "stm32f10x.h"

/*----------------���ż��----------------------*/
#define RC_MINCHECK   1100    //���ֵ���趨������Ҫ�� �㲻�þͷɲ����� 
#define RC_MAXCHECK   1800

/*----------------�������----------------------*/
#define IDLING 140

#define DATA_SIZE 4

/*----------------device--------------------*/
typedef enum 
{
	ARMED = 0x30,   //������־   0x30 = �ַ�0
	THROTTLE,		//����    0x31 = �ַ�1
	PITCH,       //�ַ�2
	ROLL,		//�ַ�3
	YAW,			//�ַ�4
	SENSITIVITY,	//������	�ַ�5
	CMD_EMPTY,		//�ַ�6
}RC;

typedef enum
{
	//--------Roll
	ROLL_CP = 0x37,    //roll_core_kp    	�ַ�7
	ROLL_CI,		//roll_core_ki			�ַ�8
	ROLL_CD,		//roll_core_kd			�ַ�9
	ROLL_SP,		//roll_shell_kp			�ַ�10
	ROLL_SI,		//roll_shell_ki			�ַ�11	
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





