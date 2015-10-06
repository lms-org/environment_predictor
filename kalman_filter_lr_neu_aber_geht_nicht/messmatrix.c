/*
 * File: messmatrix.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 26-Jun-2015 19:42:29
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "kalman_filter_lr.h"
#include "messmatrix.h"
#include "kalman_filter_lr_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *P
 *                const emxArray_real_T *r
 *                double delta
 *                const emxArray_real_T *ind
 *                const emxArray_real_T *lambda
 *                emxArray_real_T *H
 * Return Type  : void
 */
void messmatrix(const emxArray_real_T *P, const emxArray_real_T *r, double delta,
                const emxArray_real_T *ind, const emxArray_real_T *lambda,
                emxArray_real_T *H)
{
  emxArray_real_T *d_phi0;
  int num_points;
  int r_idx_0;
  int c;
  emxArray_real_T *d_c_x;
  emxArray_real_T *d_c_y;
  unsigned int count;
  b_emxInit_real_T(&d_phi0, 2);
  num_points = r->size[0] - 3;

  /* Ableitungen nach phi0 */
  r_idx_0 = r->size[0];
  c = d_phi0->size[0] * d_phi0->size[1];
  d_phi0->size[0] = r_idx_0;
  d_phi0->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)d_phi0, c, (int)sizeof(double));
  r_idx_0 = r->size[0] << 1;
  for (c = 0; c < r_idx_0; c++) {
    d_phi0->data[c] = 0.0;
  }

  for (r_idx_0 = 1; r_idx_0 - 1 <= num_points + 1; r_idx_0++) {
    d_phi0->data[r_idx_0] = d_phi0->data[r_idx_0 - 1] - delta * sin(P->
      data[r_idx_0 + (P->size[0] << 1)]);

    /* x-Komponenten */
    d_phi0->data[r_idx_0 + d_phi0->size[0]] = d_phi0->data[(r_idx_0 +
      d_phi0->size[0]) - 1] + delta * cos(P->data[r_idx_0 + (P->size[0] << 1)]);

    /* y-Komponenten */
  }

  b_emxInit_real_T(&d_c_x, 2);

  /* Ableitungen nach den Kr�mmungen c_i */
  r_idx_0 = r->size[0];
  c = d_c_x->size[0] * d_c_x->size[1];
  d_c_x->size[0] = r_idx_0;
  emxEnsureCapacity((emxArray__common *)d_c_x, c, (int)sizeof(double));
  r_idx_0 = r->size[0];
  c = d_c_x->size[0] * d_c_x->size[1];
  d_c_x->size[1] = r_idx_0;
  emxEnsureCapacity((emxArray__common *)d_c_x, c, (int)sizeof(double));
  r_idx_0 = r->size[0] * r->size[0];
  for (c = 0; c < r_idx_0; c++) {
    d_c_x->data[c] = 0.0;
  }

  b_emxInit_real_T(&d_c_y, 2);

  /* x-Komponenten */
  r_idx_0 = r->size[0];
  c = d_c_y->size[0] * d_c_y->size[1];
  d_c_y->size[0] = r_idx_0;
  emxEnsureCapacity((emxArray__common *)d_c_y, c, (int)sizeof(double));
  r_idx_0 = r->size[0];
  c = d_c_y->size[0] * d_c_y->size[1];
  d_c_y->size[1] = r_idx_0;
  emxEnsureCapacity((emxArray__common *)d_c_y, c, (int)sizeof(double));
  r_idx_0 = r->size[0] * r->size[0];
  for (c = 0; c < r_idx_0; c++) {
    d_c_y->data[c] = 0.0;
  }

  /* y-Komponenten */
  for (r_idx_0 = 2; r_idx_0 - 2 <= num_points; r_idx_0++) {
    for (c = 2; c - 2 <= num_points; c++) {
      if (1 + r_idx_0 > c) {
        d_c_x->data[r_idx_0 + d_c_x->size[0] * (c - 1)] = d_c_x->data[(r_idx_0 +
          d_c_x->size[0] * (c - 1)) - 1] - delta * delta * sin(P->data[r_idx_0 +
          (P->size[0] << 1)]) / sqrt(1.0 - delta * delta * (r->data[c] * r->
          data[c]) / 4.0);
        d_c_y->data[r_idx_0 + d_c_y->size[0] * (c - 1)] = d_c_y->data[(r_idx_0 +
          d_c_y->size[0] * (c - 1)) - 1] + delta * delta * cos(P->data[r_idx_0 +
          (P->size[0] << 1)]) / sqrt(1.0 - delta * delta * (r->data[c] * r->
          data[c]) / 4.0);
      }
    }
  }

  /* Messmatrix zusammensetzen */
  r_idx_0 = (int)(2.0 * (double)lambda->size[0]);
  c = H->size[0] * H->size[1];
  H->size[0] = r_idx_0;
  emxEnsureCapacity((emxArray__common *)H, c, (int)sizeof(double));
  r_idx_0 = r->size[0];
  c = H->size[0] * H->size[1];
  H->size[1] = r_idx_0;
  emxEnsureCapacity((emxArray__common *)H, c, (int)sizeof(double));
  r_idx_0 = (int)(2.0 * (double)lambda->size[0]) * r->size[0];
  for (c = 0; c < r_idx_0; c++) {
    H->data[c] = 0.0;
  }

  count = 2U;
  for (r_idx_0 = 0; r_idx_0 < ind->size[0]; r_idx_0++) {
    /* alle Messpunkte iterieren */
    H->data[(int)count - 2] = 0.0;

    /* Ableitung nach y0 */
    H->data[(int)count - 1] = 1.0;
    H->data[((int)count + H->size[0]) - 2] = d_phi0->data[(int)ind->data[r_idx_0]
      - 1];

    /* Ableitung nach phi0 */
    H->data[((int)count + H->size[0]) - 1] = d_phi0->data[((int)ind->
      data[r_idx_0] + d_phi0->size[0]) - 1];
    for (c = 0; c < (int)(ind->data[r_idx_0] + -2.0); c++) {
      /* Ableitung nach Kr�mmungen c */
      /*          if c==ind(i) */
      /*              H(count, c)   = d_c_x(ind(i)-1, c-1) - lambda(i)*d^2*sin(P(ind(i), 3))/sqrt(1-d^2*r(c-1)^2/4); */
      /*              H(count+1, c) = d_c_y(ind(i)-1, c-1) + lambda(i)*d^2*cos(P(ind(i), 3))/sqrt(1-d^2*r(c-1)^2/4); */
      /*          else */
      /*              H(count, c)   = d_c_x(ind(i), c-1); */
      /*              H(count+1, c) = d_c_y(ind(i), c-1); */
      /*          end    */
      if (3.0 + (double)c == ind->data[r_idx_0]) {
        H->data[((int)count + H->size[0] * (c + 2)) - 2] = d_c_x->data[((int)
          ind->data[r_idx_0] + d_c_x->size[0] * ((int)((3.0 + (double)c) - 1.0)
          - 1)) - 1] - lambda->data[r_idx_0] * (delta * delta) * sin(P->data
          [((int)(ind->data[r_idx_0] + 1.0) + (P->size[0] << 1)) - 1]) / sqrt
          (1.0 - delta * delta * (r->data[(int)((3.0 + (double)c) - 1.0) - 1] *
            r->data[(int)((3.0 + (double)c) - 1.0) - 1]) / 4.0);
        H->data[((int)count + H->size[0] * (c + 2)) - 1] = d_c_y->data[((int)
          ind->data[r_idx_0] + d_c_y->size[0] * ((int)((3.0 + (double)c) - 1.0)
          - 1)) - 1] + lambda->data[r_idx_0] * (delta * delta) * cos(P->data
          [((int)(ind->data[r_idx_0] + 1.0) + (P->size[0] << 1)) - 1]) / sqrt
          (1.0 - delta * delta * (r->data[(int)((3.0 + (double)c) - 1.0) - 1] *
            r->data[(int)((3.0 + (double)c) - 1.0) - 1]) / 4.0);
      } else {
        H->data[((int)count + H->size[0] * (c + 2)) - 2] = d_c_x->data[((int)
          ind->data[r_idx_0] + d_c_x->size[0] * ((int)((3.0 + (double)c) - 1.0)
          - 1)) - 1];
        H->data[((int)count + H->size[0] * (c + 2)) - 1] = d_c_y->data[((int)
          ind->data[r_idx_0] + d_c_y->size[0] * ((int)((3.0 + (double)c) - 1.0)
          - 1)) - 1];
      }
    }

    count += 2U;
  }

  emxFree_real_T(&d_c_y);
  emxFree_real_T(&d_c_x);
  emxFree_real_T(&d_phi0);
}

/*
 * File trailer for messmatrix.c
 *
 * [EOF]
 */
