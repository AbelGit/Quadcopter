#include "my_math.h"
#include "stm32f10x.h"

/**************************实现函数********************************************
*函数原型:    data_limit(float data,flaot toplimit,float lowerlimit)
*功　　能:    数据限幅
输入参数：    data       要操作的数据 
*             toplimit   上限
*             lowerlimit 下限
输出参数：    无
*******************************************************************************/
float data_limit(float data,float toplimit,float lowerlimit)
{
	if(data > toplimit)  data = toplimit;
	else if(data < lowerlimit) data = lowerlimit;
	return data;
}

/**************************实现函数********************************************
*函数原型:    array_assign(int16_t *array,int16_t value)
*功　　能:    对数组赋值
输入参数：    *array   目标数组指针 
*             value      
输出参数：    无
*******************************************************************************/
void array_assign(s16 *array,s16 value,u16 length)
{
	u16 i;	
   for(i=0;i<length;i++)
   {
     *(array+i) = value;
   } 
}











