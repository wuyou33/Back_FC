#ifndef __FILTER_H
#define __FILTER_H

#include "parameter.h"

#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))

#define LPF_1(A,B,C,D) LPF_1_((A),(B),(C),*(D));

typedef struct
{
	float a;
	float b;
	float e_nr;
	float out;
} _filter_1_st;

void anotc_filter_1(float base_hz,float gain_hz,float dT,float in,_filter_1_st *f1);
void Moving_Average1(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out);
void Moving_Average(float moavarray[],u16 len ,u16 *fil_cnt,float in,float *out);
s32 Moving_Median(s32 moavarray[],u16 len ,u16 *fil_p,s32 in);
#define _xyz_f_t xyz_f_t
void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out);
double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na);





#define NUMBER_OF_FIRST_ORDER_FILTERS 5
#define ACC_LOWPASS_X 0
#define ACC_LOWPASS_Y 1
#define ACC_LOWPASS_Z 2
typedef struct firstOrderFilterData {
  float   gx1;
  float   gx2;
  float   gx3;
  float   previousInput;
  float   previousOutput;
} firstOrderFilterData_t;

extern firstOrderFilterData_t firstOrderFilters[NUMBER_OF_FIRST_ORDER_FILTERS];

void initFirstOrderFilter(float T);
float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters,float T);
#endif
