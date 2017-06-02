
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "stm32f10x.h"
#include "nrf24L01.h"
#include "rc.h"
#include "delay.h"
#include "oled.h"

_RC_DATA rc_data;
struct _ctrl ctrl;
struct _target Target;

//euler
u8 armed_char_buf[DATA_SIZE] = "0";
u8 throttle_char_buf[DATA_SIZE] = "0";
u8 pitch_char_buf[DATA_SIZE] = "0";
u8 roll_char_buf[DATA_SIZE] = "0";
u8 yaw_char_buf[DATA_SIZE] = "0";
u8 sensitivity_char_buf[DATA_SIZE] = "0";
//roll_cs_pid
u8 roll_cp_char_buf[DATA_SIZE] = "0";
u8 roll_ci_char_buf[DATA_SIZE] = "0";
u8 roll_cd_char_buf[DATA_SIZE] = "0";
u8 roll_sp_char_buf[DATA_SIZE] = "0";
u8 roll_si_char_buf[DATA_SIZE] = "0";
//pitch_cs_pid
u8 pitch_cp_char_buf[DATA_SIZE] = "0";
u8 pitch_ci_char_buf[DATA_SIZE] = "0";
u8 pitch_cd_char_buf[DATA_SIZE] = "0";
u8 pitch_sp_char_buf[DATA_SIZE] = "0";
u8 pitch_si_char_buf[DATA_SIZE] = "0";
//yaw_cs_pid
u8 yaw_cp_char_buf[DATA_SIZE] = "0";
u8 yaw_ci_char_buf[DATA_SIZE] = "0";
u8 yaw_cd_char_buf[DATA_SIZE] = "0";
u8 yaw_sp_char_buf[DATA_SIZE] = "0";
u8 yaw_si_char_buf[DATA_SIZE] = "0";

// use sprintf() concert float data to char to send 
static void float_to_char(void)
{
	//euler
	sprintf((char*)armed_char_buf, "%1.2f", rc_data.armed_val);
	sprintf((char*)throttle_char_buf, "%1.2f", rc_data.throttle_val);
	sprintf((char*)pitch_char_buf, "%1.2f", rc_data.pitch_val);
	sprintf((char*)roll_char_buf, "%1.2f", rc_data.roll_val);
	sprintf((char*)yaw_char_buf, "%1.2f", rc_data.yaw_val);
	sprintf((char*)sensitivity_char_buf, "%1.2f", rc_data.sensitivity_val);
	//roll_cs_pid 
	sprintf((char*)roll_cp_char_buf, "%1.2f", ctrl.roll.core.kp);
	sprintf((char*)roll_ci_char_buf, "%1.2f", ctrl.roll.core.ki);
	sprintf((char*)roll_cd_char_buf, "%1.2f", ctrl.roll.core.kd);
	sprintf((char*)roll_sp_char_buf, "%1.2f", ctrl.roll.shell.kp);
	sprintf((char*)roll_si_char_buf, "%1.2f", ctrl.roll.shell.ki);
	//pitch_cs_pid 
	sprintf((char*)pitch_cp_char_buf, "%1.2f", ctrl.pitch.core.kp);
	sprintf((char*)pitch_ci_char_buf, "%1.2f", ctrl.pitch.core.ki);
	sprintf((char*)pitch_cd_char_buf, "%1.2f", ctrl.pitch.core.kd);
	sprintf((char*)pitch_sp_char_buf, "%1.2f", ctrl.pitch.shell.kp);
	sprintf((char*)pitch_si_char_buf, "%1.2f", ctrl.pitch.shell.ki);
	//yaw_cs_pid 
	sprintf((char*)yaw_cp_char_buf, "%1.2f", ctrl.yaw.core.kp);
	sprintf((char*)yaw_ci_char_buf, "%1.2f", ctrl.yaw.core.ki);
	sprintf((char*)yaw_cd_char_buf, "%1.2f", ctrl.yaw.core.kd);
	sprintf((char*)yaw_sp_char_buf, "%1.2f", ctrl.yaw.shell.kp);
	sprintf((char*)yaw_si_char_buf, "%1.2f", ctrl.yaw.shell.ki);
}

/***************************************************************
func:  pack_send_data
Instruction: packet the protocal object and send to pc
parameter: driver(select machine) , drive_cmd(control), data_len(), data(send what)

***************************************************************/

static void pack_send_data(u8 drive, u8 drive_cmd, u8 data_len, u8 *data)
{
	u8 i;
	u8 *tx_buff = malloc(sizeof(u8)*10);

	memset(tx_buff, 0, 10);

	*tx_buff = 0x2A;   //start * 
	*(tx_buff+1) = drive;  //device 
	*(tx_buff+2) = drive_cmd;  //device cmd   
	for(i = 0; i < data_len; i++)
	{
		*(tx_buff+i+3) = data[i]; //data[1,2,3,4]
	}
	*(tx_buff+7) = 0x23; //end  #
	
	NRF24L01_TxPacket(tx_buff);
	
	printf("\r\n slave data: %s\r\n", (char*)tx_buff);
	
	free(tx_buff);
}

static void pack_send_euler(void)
{
	//euler
	pack_send_data(ARMED, CMD_EMPTY, DATA_SIZE, armed_char_buf);
	pack_send_data(THROTTLE, CMD_EMPTY, DATA_SIZE, throttle_char_buf);
	pack_send_data(PITCH, CMD_EMPTY, DATA_SIZE, pitch_char_buf);
	pack_send_data(ROLL, CMD_EMPTY, DATA_SIZE, roll_char_buf);
	pack_send_data(YAW, CMD_EMPTY, DATA_SIZE, yaw_char_buf);
}

static void pack_send_roll_pid(void)
{
	//Roll_cs_pid
	pack_send_data(ROLL_CP, CMD_EMPTY, DATA_SIZE, roll_cp_char_buf);
	pack_send_data(ROLL_CI, CMD_EMPTY, DATA_SIZE, roll_ci_char_buf);
	pack_send_data(ROLL_CD, CMD_EMPTY, DATA_SIZE, roll_cd_char_buf);
	pack_send_data(ROLL_SP, CMD_EMPTY, DATA_SIZE, roll_sp_char_buf);
	pack_send_data(ROLL_SI, CMD_EMPTY, DATA_SIZE, roll_si_char_buf);
}

static void pack_send_pitch_pid(void)
{
	//Pitch_cs_pid
	pack_send_data(PITCH_CP, CMD_EMPTY, DATA_SIZE, pitch_cp_char_buf);
	pack_send_data(PITCH_CI, CMD_EMPTY, DATA_SIZE, pitch_ci_char_buf);
	pack_send_data(PITCH_CD, CMD_EMPTY, DATA_SIZE, pitch_cd_char_buf);
	pack_send_data(PITCH_SP, CMD_EMPTY, DATA_SIZE, pitch_sp_char_buf);
	pack_send_data(PITCH_SI, CMD_EMPTY, DATA_SIZE, pitch_si_char_buf);
}

static void pack_send_yaw_pid(void)
{
	//Yaw_cs_pid
	pack_send_data(YAW_CP, CMD_EMPTY, DATA_SIZE, yaw_cp_char_buf);
	pack_send_data(YAW_CI, CMD_EMPTY, DATA_SIZE, yaw_ci_char_buf);
	pack_send_data(YAW_CD, CMD_EMPTY, DATA_SIZE, yaw_cd_char_buf);
	pack_send_data(YAW_SP, CMD_EMPTY, DATA_SIZE, yaw_sp_char_buf);
	pack_send_data(YAW_SI, CMD_EMPTY, DATA_SIZE, yaw_si_char_buf);
}

void rc_control(void)
{
	float_to_char();
	
	pack_send_euler();
	
	pack_send_roll_pid();
	pack_send_pitch_pid();
	pack_send_yaw_pid();
}
















