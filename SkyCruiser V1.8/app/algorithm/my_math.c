#include "my_math.h"
#include "stm32f10x.h"

/**************************ʵ�ֺ���********************************************
*����ԭ��:    data_limit(float data,flaot toplimit,float lowerlimit)
*��������:    �����޷�
���������    data       Ҫ���������� 
*             toplimit   ����
*             lowerlimit ����
���������    ��
*******************************************************************************/
float data_limit(float data,float toplimit,float lowerlimit)
{
	if(data > toplimit)  data = toplimit;
	else if(data < lowerlimit) data = lowerlimit;
	return data;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:    array_assign(int16_t *array,int16_t value)
*��������:    �����鸳ֵ
���������    *array   Ŀ������ָ�� 
*             value      
���������    ��
*******************************************************************************/
void array_assign(s16 *array,s16 value,u16 length)
{
	u16 i;	
   for(i=0;i<length;i++)
   {
     *(array+i) = value;
   } 
}











