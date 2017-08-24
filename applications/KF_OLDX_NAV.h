/*
 * File: KF_OLDX_NAV.h
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 03-Dec-2016 20:26:50
 */

#ifndef __KF_OLDX_NAV_H__
#define __KF_OLDX_NAV_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "KF_OLDX_NAV_types.h"

/* Function Declarations */
void KF_OLDX_NAV(double X[3], double P[9],  double Z[3], double U,  double A[9],  double B[3],  double H[9], double ga, double gwa, double g_pos, double g_spd, double T);

#endif

/*
 * File trailer for KF_OLDX_NAV.h
 *
 * [EOF]
 */
