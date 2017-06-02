
#include "ahrs.h"
#include "mpu6050.h"
#include "filter.h"


extern sensor_val sensor;

#define  IIR_ORDER     4      //使用IIR滤波器的阶数
double b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //系数b
double a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//系数a
double InPut_IIR[3][IIR_ORDER+1] = {0};
double OutPut_IIR[3][IIR_ORDER+1] = {0};



// obtain acc and gyro data, and obtain quiet data 
void MPU6050_Dataanl(void)
{
	static short acc_x, acc_y, acc_z;
	static short gyro_x, gyro_y, gyro_z;
	
	//original_data = actual_data - quiet_data   need to do this 
	MPU_Get_Accelerometer(&acc_x, &acc_y, &acc_z); 	//acc actual data
	MPU_Get_Gyroscope(&gyro_x, &gyro_y, &gyro_z);	//gyro actual data
	
	sensor.acc.origin.x = acc_x - sensor.acc.quiet.x;
	sensor.acc.origin.y = acc_y - sensor.acc.quiet.y;
	sensor.acc.origin.z = acc_z;
	
	sensor.gyro.origin.x = gyro_x;
	sensor.gyro.origin.y = gyro_y;
	sensor.gyro.origin.z = gyro_z;
	
	sensor.gyro.radian.x = sensor.gyro.origin.x - sensor.gyro.quiet.x;
	sensor.gyro.radian.y = sensor.gyro.origin.y - sensor.gyro.quiet.y;
	sensor.gyro.radian.z = sensor.gyro.origin.z - sensor.gyro.quiet.z;
	
	// calibration acc data 
	if(flag.calibratingA)
	{
		static s32 tempax=0, tempay=0, tempaz=0;
		static u8 cnt_a=0;
		if(cnt_a == 0)
		{
			sensor.acc.quiet.x = 0;
			sensor.acc.quiet.y = 0;
			sensor.acc.quiet.z = 0;
			tempax = 0;
			tempay = 0;
			tempaz = 0;
			cnt_a = 1;
		}
		tempax += sensor.acc.origin.x;
		tempay += sensor.acc.origin.y;
		tempaz += sensor.acc.origin.z;
		if(cnt_a == 200)
		{
			sensor.acc.quiet.x = tempax/cnt_a;
			sensor.acc.quiet.y = tempay/cnt_a;
			sensor.acc.quiet.z = tempaz/cnt_a;
			cnt_a = 0;
			flag.calibratingA = 0;
			//EE_SAVE_ACC_OFFSET();  //use eeprom save acc offset 
			return ;
		}
		cnt_a++;
	}
}

// 对原始数据滤波处理
void AHRS_getValues(void)
{
	static float x, y, z;
	
	MPU6050_Dataanl();  //obtain actual data, and calibration
	 
	// 加速度计IIR滤波
	sensor.acc.averag.x = IIR_I_Filter(sensor.acc.origin.x, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	sensor.acc.averag.y = IIR_I_Filter(sensor.acc.origin.y, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	sensor.acc.averag.z = IIR_I_Filter(sensor.acc.origin.z, InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	
	// 陀螺仪一阶低通滤波
 	sensor.gyro.averag.x = LPF_1st(x,sensor.gyro.radian.x * Gyro_G,0.386f);	x = sensor.gyro.averag.x;
 	sensor.gyro.averag.y = LPF_1st(y,sensor.gyro.radian.y * Gyro_G,0.386f);	y = sensor.gyro.averag.y;
 	sensor.gyro.averag.z = LPF_1st(z,sensor.gyro.radian.z * Gyro_G,0.386f);	z = sensor.gyro.averag.z;
}













