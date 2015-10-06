//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: kalman_filter_lr.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 06-Oct-2015 18:59:00
//
#ifndef __KALMAN_FILTER_LR_H__
#define __KALMAN_FILTER_LR_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "kalman_filter_lr_types.h"

// Function Declarations
extern void kalman_filter_lr(emxArray_real_T *r, double A, emxArray_real_T *Pk,
  const emxArray_real_T *Q, double R_fakt, double delta, emxArray_real_T *xl,
  emxArray_real_T *yl, emxArray_real_T *xr, emxArray_real_T *yr, emxArray_real_T
  *xm, emxArray_real_T *ym);

#endif

//
// File trailer for kalman_filter_lr.h
//
// [EOF]
//
