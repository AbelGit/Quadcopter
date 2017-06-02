
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f10x.h"
#include "nrf24L01.h"
#include "mpu6050.h"
#include "delay.h"
#include "rc.h"
#include "control.h"

_RC_DATA rc_data;
u8 rx_buff[10] = {0};


/*------------解析数据--------------*/
/*
	rx_buff[0]	start 0x2A
	rx_buff[1]	device 		
	rx_buff[2]	device cmd 
	rx_buff[3]	data[1]
	rx_buff[4]	data[2]
	rx_buff[5]	data[3]
	rx_buff[6]	data[4]
	rx_buff[7]	end  0x23
*/

void parse_recv_data(void)
{
	NRF24L01_RxPacket(rx_buff);
	
	switch(rx_buff[1])
	{
		//euler
		case ARMED: exec_module_armed(); break;
		case THROTTLE: exex_module_throttle(); break;
		case PITCH: exex_module_pitch(); break;
		case ROLL: exex_module_roll(); 	break;
		case YAW: exex_module_yaw(); break;
		case SENSITIVITY: exex_module_sensitivy(); break;
		
		//Roll_cs_pid 
		case ROLL_CP: exec_module_roll_cp(); break;
		case ROLL_CI: exec_module_roll_ci(); break;
		case ROLL_CD: exec_module_roll_cd(); break;
		case ROLL_SP: exec_module_roll_sp(); break;
		case ROLL_SI: exec_module_roll_si(); break;
		//Pitch_cs_pid
		case PITCH_CP: exec_module_pitch_cp(); break;
		case PITCH_CI: exec_module_pitch_ci(); break;
		case PITCH_CD: exec_module_pitch_cd(); break;
		case PITCH_SP: exec_module_pitch_sp(); break;
		case PITCH_SI: exec_module_pitch_si(); break;
		//Yaw_cs_pid
		case YAW_CP: exec_module_yaw_cp(); break;
		case YAW_CI: exec_module_yaw_ci(); break;
		case YAW_CD: exec_module_yaw_cd(); break;
		case YAW_SP: exec_module_yaw_sp(); break;
		case YAW_SI: exec_module_yaw_si(); break;
			
	}
}

//-----------------------------euler--------------------------------------------
//电机解锁
void exec_module_armed(void)
{
	rc_data.armed_val =  atoi((char*)(rx_buff+3));  //str to num 
	if(rc_data.armed_val == 1)
	{
		flag.ARMED = 1;
	}else
	{
		flag.ARMED = 0;
	}
}

//油门信号
void exex_module_throttle(void)
{
	if(rx_buff[6] == 0x2E)  //0x2E == .
	{
		rc_data.throttle_val = (rx_buff[3]-0x30)*100 + (rx_buff[4]-0x30)*10 + (rx_buff[5]-0x30);
	}else
	{
		rc_data.throttle_val = (rx_buff[3]-0x30)*1000 + (rx_buff[4]-0x30)*100 + (rx_buff[5]-0x30)*10 + (rx_buff[6]-0x30);
	}
}
//pitch 角度调整
void exex_module_pitch(void)
{
	if((rx_buff[3]==0x2D) && (rx_buff[5]==0x2E))  //0x2D == -
	{
		rc_data.pitch_val =  0-(rx_buff[4]-0x30);
	}else if((rx_buff[3]==0x2D) && (rx_buff[6]==0x2E))   //0x2E == .
	{
		rc_data.pitch_val =  0-((rx_buff[4]-0x30)*10 + (rx_buff[5]-0x30));
	}else
	{
		rc_data.pitch_val = rx_buff[3]-0x30;
	}
}
//roll 角度调整
void exex_module_roll(void)
{
	if((rx_buff[3]==0x2D) && (rx_buff[5]==0x2E))  //0x2D == -
	{
		rc_data.roll_val =  0-(rx_buff[4]-0x30);
	}
	else if(rx_buff[4] == 0x2E)  //0x2E == .
	{
		rc_data.roll_val = rx_buff[3]-0x30;
	}
	else if(rx_buff[5] == 0x2E) 
	{
		rc_data.roll_val =  ((rx_buff[3]-0x30)*10 + (rx_buff[4]-0x30));
	}
}
//yaw 角度调整
void exex_module_yaw(void)
{
	if((rx_buff[3]==0x2D) && (rx_buff[5]==0x2E))  //0x2D == -
	{
		rc_data.yaw_val =  0-(rx_buff[4]-0x30);
	}
	else if(rx_buff[4] == 0x2E)  //0x2E == .
	{
		rc_data.yaw_val = rx_buff[3]-0x30;
	}
	else if(rx_buff[5] == 0x2E) 
	{
		rc_data.yaw_val =  ((rx_buff[3]-0x30)*10 + (rx_buff[4]-0x30));
	}
}
//灵敏度调整
void exex_module_sensitivy(void)
{
	if(rx_buff[5] == 0x2E)  //0x2E == .
	{
		rc_data.sensitivity_val = (rx_buff[3]-0x30)*10 + (rx_buff[4]-0x30);
	}		
}

/*---------------------Roll pid-----------------------*/
static float parse_pid(void)
{
	char pid_buff[5] = {0};
	
	pid_buff[0] = rx_buff[3];
	pid_buff[1] = rx_buff[4];
	pid_buff[2] = rx_buff[5];
	pid_buff[3] = rx_buff[6];
	pid_buff[4] = '\0';
	
	return atof(pid_buff);
}
void exec_module_roll_cp(void)
{
	ctrl.roll.core.kp = parse_pid();
}
void exec_module_roll_ci(void)
{
	ctrl.roll.core.ki = parse_pid();
}
void exec_module_roll_cd(void)
{
	ctrl.roll.core.kd = parse_pid();
}
void exec_module_roll_sp(void)
{
	ctrl.roll.shell.kp = parse_pid();
}
void exec_module_roll_si(void)
{
	ctrl.roll.shell.ki = parse_pid();
}

/*---------------------Pitch pid--------------------------*/
void exec_module_pitch_cp(void)
{
	ctrl.pitch.core.kp = parse_pid();
}
void exec_module_pitch_ci(void)
{
	ctrl.pitch.core.ki = parse_pid();
}
void exec_module_pitch_cd(void)
{
	ctrl.pitch.core.kd = parse_pid();
}
void exec_module_pitch_sp(void)
{
	ctrl.pitch.shell.kp = parse_pid();
}
void exec_module_pitch_si(void)
{
	ctrl.pitch.shell.ki = parse_pid();
}
/*---------------------Yaw pid--------------------------*/
void exec_module_yaw_cp(void)
{
	ctrl.yaw.core.kp = parse_pid();
}
void exec_module_yaw_ci(void)
{
	ctrl.yaw.core.ki = parse_pid();
}
void exec_module_yaw_cd(void)
{
	ctrl.yaw.core.kd = parse_pid();
}
void exec_module_yaw_sp(void)
{
	ctrl.yaw.shell.kp = parse_pid();
}
void exec_module_yaw_si(void)
{
	ctrl.yaw.shell.ki = parse_pid();
}









