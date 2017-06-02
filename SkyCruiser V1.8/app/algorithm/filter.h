
#ifndef __FILTER_H
#define __FILTER_H

#include "stm32f10x.h"


double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na);
float LPF_1st(float oldData, float newData, float lpf_factor);


#endif 


