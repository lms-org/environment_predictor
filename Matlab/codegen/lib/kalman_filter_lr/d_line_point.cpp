//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: d_line_point.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 06-Oct-2015 18:59:00
//

// Include Files
#include "rt_nonfinite.h"
#include "kalman_filter_lr.h"
#include "d_line_point.h"

// Function Definitions

//
// Abstand d zwischen der Gerade g (von P in Richtung Q) und dem Punkt M
// S ist der Punkt auf g mit kleinstem Abstand zu M
// Arguments    : const double P[2]
//                const double Q[2]
//                const double M[2]
//                double *d
//                double *lambda
//                double S[2]
// Return Type  : void
//
void d_line_point(const double P[2], const double Q[2], const double M[2],
                  double *d, double *lambda, double S[2])
{
  int i2;
  double a;
  double b_a;
  for (i2 = 0; i2 < 2; i2++) {
    S[i2] = Q[i2] - P[i2];
  }

  *lambda = -(S[0] * (P[0] - M[0]) + S[1] * (P[1] - M[1])) / (S[0] * S[0] + S[1]
    * S[1]);
  for (i2 = 0; i2 < 2; i2++) {
    S[i2] = P[i2] + *lambda * S[i2];
  }

  a = M[0] - S[0];
  b_a = M[1] - S[1];
  *d = sqrt(a * a + b_a * b_a);
}

//
// File trailer for d_line_point.cpp
//
// [EOF]
//
