//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: kalman_filter_lr.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 06-Oct-2015 18:59:00
//

// Include Files
#include "rt_nonfinite.h"
#include "kalman_filter_lr.h"
#include "d_line_point.h"
#include "kalman_filter_lr_emxutil.h"
#include "eye.h"
#include "mrdivide.h"
#include "messmatrix.h"
#include "getPointsFromState.h"
#include "projectPoints.h"

// Function Declarations
static void b_eml_null_assignment(emxArray_real_T *x, const emxArray_boolean_T
  *idx);
static void eml_null_assignment(emxArray_real_T *x, const emxArray_boolean_T
  *idx);

// Function Definitions

//
// Arguments    : emxArray_real_T *x
//                const emxArray_boolean_T *idx
// Return Type  : void
//
static void b_eml_null_assignment(emxArray_real_T *x, const emxArray_boolean_T
  *idx)
{
  int nrowx;
  int nrows;
  int k;
  int i;
  int j;
  emxArray_real_T *b_x;
  nrowx = x->size[0];
  nrows = 0;
  for (k = 1; k <= idx->size[0]; k++) {
    nrows += idx->data[k - 1];
  }

  nrows = x->size[0] - nrows;
  i = 0;
  for (k = 1; k <= nrowx; k++) {
    if ((k > idx->size[0]) || (!idx->data[k - 1])) {
      for (j = 0; j < 3; j++) {
        x->data[i + x->size[0] * j] = x->data[(k + x->size[0] * j) - 1];
      }

      i++;
    }
  }

  if (1 > nrows) {
    nrows = 0;
  }

  emxInit_real_T1(&b_x, 2);
  i = b_x->size[0] * b_x->size[1];
  b_x->size[0] = nrows;
  b_x->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)b_x, i, (int)sizeof(double));
  for (i = 0; i < 3; i++) {
    for (j = 0; j < nrows; j++) {
      b_x->data[j + b_x->size[0] * i] = x->data[j + x->size[0] * i];
    }
  }

  i = x->size[0] * x->size[1];
  x->size[0] = b_x->size[0];
  x->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)x, i, (int)sizeof(double));
  for (i = 0; i < 3; i++) {
    nrows = b_x->size[0];
    for (j = 0; j < nrows; j++) {
      x->data[j + x->size[0] * i] = b_x->data[j + b_x->size[0] * i];
    }
  }

  emxFree_real_T(&b_x);
}

//
// Arguments    : emxArray_real_T *x
//                const emxArray_boolean_T *idx
// Return Type  : void
//
static void eml_null_assignment(emxArray_real_T *x, const emxArray_boolean_T
  *idx)
{
  int nxin;
  int nrowx;
  int nxout;
  int k;
  int k0;
  emxArray_real_T *b_x;
  nxin = x->size[0];
  nrowx = x->size[0];
  nxout = 0;
  for (k = 1; k <= idx->size[0]; k++) {
    nxout += idx->data[k - 1];
  }

  nxout = x->size[0] - nxout;
  k0 = -1;
  for (k = 1; k <= nxin; k++) {
    if ((k > idx->size[0]) || (!idx->data[k - 1])) {
      k0++;
      x->data[k0] = x->data[k - 1];
    }
  }

  if (nrowx != 1) {
    if (1 > nxout) {
      nxout = 0;
    }

    emxInit_real_T(&b_x, 1);
    k0 = b_x->size[0];
    b_x->size[0] = nxout;
    emxEnsureCapacity((emxArray__common *)b_x, k0, (int)sizeof(double));
    for (k0 = 0; k0 < nxout; k0++) {
      b_x->data[k0] = x->data[k0];
    }

    k0 = x->size[0];
    x->size[0] = b_x->size[0];
    emxEnsureCapacity((emxArray__common *)x, k0, (int)sizeof(double));
    nxout = b_x->size[0];
    for (k0 = 0; k0 < nxout; k0++) {
      x->data[k0] = b_x->data[k0];
    }

    emxFree_real_T(&b_x);
  } else {
    k0 = x->size[0];
    if (1 > nxout) {
      x->size[0] = 0;
    } else {
      x->size[0] = nxout;
    }

    emxEnsureCapacity((emxArray__common *)x, k0, (int)sizeof(double));
  }
}

//
// "kalman_filter_lr"
//
//  ----Nomenklatur----
//   * r: Zustandsvektor = [y0, phi0, kappa_1, kappa_2, ... , kappa_(n-1)]
//   --> y0: y-Wert des ersten Punktes (x-Wert ist immer 0)
//   --> phi0: Anfangssteigung zwischem ersten und zweiten Punkt
//   --> kappa_i: Krümmungen an den entsprechenden Punkten (Krümmungsdefinition: 1/R,
//   wobei R der Radius des Kreises ist, auf dem der betrachtete Punkt, sein Vorgänger
//   und sein Nachfolger liegen)
//   * Pk: Kovarianzmatrix des Zustands dim[n x n], mit n = Dimension des Zustandsvektors
//   * A: State-Transition-Matrix dim[n x n] (ist zurzeit eine Einheitsmatrix, da noch keine
//    Messwerte fuer die Eigenbewegung verfügbar sind)
//   * Q: Kovarianz des Zustandsübergangs (Prozessrauschen) dim[n x n] (symmetrische Matrix, mit weg von der
//    Diagonalen abnehmenden Eintraegen)
//   * R_fakt: Unsicherheit der Messwerte (Messrauschen)
//   * delta: Abstand der Punkte (delta*n ergibt die Länge des praedizierten Fahrstreifens)
//   * xl, yl: Vektoren mit den Messwerten für die linke Spur
//   * xr, yr: Vektoren mit den Messwerten für die rechte Spur
//
//   ----Grober Ablauf des Algorithmus----
//   1. Projektion der Punkte von der Mittellinie nach links bzw. rechts
//   2. Fuer jeden Messpunkt: Berechnung des kleinsten Abstands zum aktuell praedizierten Strassenverlauf
//   3. Assemblierung der Jakobimatrix fuer die Projektion aus dem Zustandsraum r auf x-y-Koordinaten
//   4. Kalman-Filter: Praediktion -> Messwerte einbeziehen -> Update
//   5. Zustandsbegrenzungen einbringen
// Arguments    : emxArray_real_T *r
//                double A
//                emxArray_real_T *Pk
//                const emxArray_real_T *Q
//                double R_fakt
//                double delta
//                emxArray_real_T *xl
//                emxArray_real_T *yl
//                emxArray_real_T *xr
//                emxArray_real_T *yr
//                emxArray_real_T *xm
//                emxArray_real_T *ym
// Return Type  : void
//
void kalman_filter_lr(emxArray_real_T *r, double, emxArray_real_T *Pk, const
                      emxArray_real_T *Q, double R_fakt, double delta,
                      emxArray_real_T *xl, emxArray_real_T *yl, emxArray_real_T *
                      xr, emxArray_real_T *yr, emxArray_real_T *xm,
                      emxArray_real_T *ym)
{
  emxArray_real_T *P;
  emxArray_real_T *z_m;
  emxArray_real_T *zmm;
  emxArray_real_T *y_tilde;
  int cr;
  int br;
  int ar;
  int i4;
  emxArray_real_T *D;
  int ib;
  int s;
  int m;
  double minval;
  double a;
  double dist_point;
  double b_P[2];
  double c_P[2];
  double b_xl[2];
  double unusedU2[2];
  double lambda;
  double dist_line;
  emxArray_boolean_T *ind;
  emxArray_real_T *b_D;
  emxArray_real_T *c_D;
  emxArray_real_T *Hl;
  emxArray_real_T *zl;
  emxArray_real_T *zml;
  unsigned int count;
  double d_P[2];
  double e_P[2];
  double b_xr[2];
  emxArray_real_T *d_D;
  emxArray_real_T *e_D;
  emxArray_real_T *Hr;
  emxArray_real_T *zr;
  emxArray_real_T *zmr;
  double f_P[2];
  double g_P[2];
  double b_xm[2];
  emxArray_real_T *f_D;
  emxArray_real_T *g_D;
  emxArray_real_T *Hm;
  emxArray_real_T *H;
  int i5;
  int i;
  emxArray_real_T *b_Hl;
  int k;
  unsigned int Hl_idx_0;
  int ic;
  int ia;
  emxArray_real_T *y;
  int c;
  emxArray_real_T *b_zl;
  emxArray_real_T *b_zml;
  emxArray_real_T *b_y;
  emxArray_real_T *C;
  emxArray_real_T *c_y;
  emxArray_real_T *b_C;
  emxArray_real_T *c_C;
  emxArray_real_T *d_C;
  emxArray_real_T *b_Hr;
  emxInit_real_T1(&P, 2);
  emxInit_real_T(&z_m, 1);
  emxInit_real_T(&zmm, 1);
  emxInit_real_T(&y_tilde, 1);

  // % linke Seitenlinie
  projectPoints(r, delta, 0.38, y_tilde, z_m, zmm);

  // Punkte von Mittellinie nach links projizieren
  cr = y_tilde->size[0];
  br = z_m->size[0];
  ar = zmm->size[0];
  i4 = P->size[0] * P->size[1];
  P->size[0] = cr;
  P->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)P, i4, (int)sizeof(double));
  for (i4 = 0; i4 < cr; i4++) {
    P->data[i4] = y_tilde->data[i4];
  }

  for (i4 = 0; i4 < br; i4++) {
    P->data[i4 + P->size[0]] = z_m->data[i4];
  }

  for (i4 = 0; i4 < ar; i4++) {
    P->data[i4 + (P->size[0] << 1)] = zmm->data[i4];
  }

  emxInit_real_T1(&D, 2);
  i4 = D->size[0] * D->size[1];
  D->size[0] = xl->size[0];
  D->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)D, i4, (int)sizeof(double));
  ib = xl->size[0] * 3;
  for (i4 = 0; i4 < ib; i4++) {
    D->data[i4] = 10000.0;
  }

  //  Fuer jeden Messpunkt den nähesten Punkt der aktuellen Praediktion finden
  for (s = 0; s < r->size[0]; s++) {
    for (m = 0; m < xl->size[0]; m++) {
      minval = P->data[s] - xl->data[m];
      a = P->data[s + P->size[0]] - yl->data[m];
      dist_point = sqrt(minval * minval + a * a);
      if (dist_point < D->data[m + (D->size[0] << 1)]) {
        D->data[m] = 1.0 + (double)s;
        D->data[m + D->size[0]] = 0.0;
        D->data[m + (D->size[0] << 1)] = dist_point;
      }

      if (1.0 + (double)s > 1.0) {
        b_P[0] = P->data[(int)((1.0 + (double)s) - 1.0) - 1];
        b_P[1] = P->data[((int)((1.0 + (double)s) - 1.0) + P->size[0]) - 1];
        c_P[0] = P->data[s];
        c_P[1] = P->data[s + P->size[0]];
        b_xl[0] = xl->data[m];
        b_xl[1] = yl->data[m];
        d_line_point(b_P, c_P, b_xl, &dist_line, &lambda, unusedU2);
        if ((dist_line < D->data[m + (D->size[0] << 1)]) && (lambda > 0.0) &&
            (lambda < 1.0)) {
          D->data[m] = (1.0 + (double)s) - 1.0;
          D->data[m + D->size[0]] = lambda;
          D->data[m + (D->size[0] << 1)] = dist_line;
        }
      }
    }
  }

  emxInit_boolean_T(&ind, 1);

  //  Messpunkte ausfiltern, die vor dem letzten prädizierten Punkt oder zu weit entfernt liegen 
  ib = D->size[0];
  br = r->size[0];
  i4 = ind->size[0];
  ind->size[0] = ib;
  emxEnsureCapacity((emxArray__common *)ind, i4, (int)sizeof(boolean_T));
  for (i4 = 0; i4 < ib; i4++) {
    ind->data[i4] = ((D->data[i4] == br) || (D->data[i4 + (D->size[0] << 1)] >
      0.5));
  }

  emxInit_real_T(&b_D, 1);
  eml_null_assignment(xl, ind);
  eml_null_assignment(yl, ind);
  b_eml_null_assignment(D, ind);

  //  Messmatrix, Mess- und Erwartungsvektor bauen
  ib = D->size[0];
  i4 = b_D->size[0];
  b_D->size[0] = ib;
  emxEnsureCapacity((emxArray__common *)b_D, i4, (int)sizeof(double));
  for (i4 = 0; i4 < ib; i4++) {
    b_D->data[i4] = D->data[i4];
  }

  emxInit_real_T(&c_D, 1);
  ib = D->size[0];
  i4 = c_D->size[0];
  c_D->size[0] = ib;
  emxEnsureCapacity((emxArray__common *)c_D, i4, (int)sizeof(double));
  for (i4 = 0; i4 < ib; i4++) {
    c_D->data[i4] = D->data[i4 + D->size[0]];
  }

  emxInit_real_T1(&Hl, 2);
  emxInit_real_T(&zl, 1);
  messmatrix(P, r, delta, b_D, c_D, Hl);
  i4 = zl->size[0];
  zl->size[0] = (int)(2.0 * (double)xl->size[0]);
  emxEnsureCapacity((emxArray__common *)zl, i4, (int)sizeof(double));
  ib = (int)(2.0 * (double)xl->size[0]);
  emxFree_real_T(&c_D);
  emxFree_real_T(&b_D);
  for (i4 = 0; i4 < ib; i4++) {
    zl->data[i4] = 0.0;
  }

  emxInit_real_T(&zml, 1);
  i4 = zml->size[0];
  zml->size[0] = (int)(2.0 * (double)xl->size[0]);
  emxEnsureCapacity((emxArray__common *)zml, i4, (int)sizeof(double));
  ib = (int)(2.0 * (double)xl->size[0]);
  for (i4 = 0; i4 < ib; i4++) {
    zml->data[i4] = 0.0;
  }

  count = 2U;
  for (m = 0; m < xl->size[0]; m++) {
    zl->data[(int)count - 2] = xl->data[m];
    zl->data[(int)count - 1] = yl->data[m];
    zml->data[(int)count - 2] = P->data[(int)D->data[m] - 1] + D->data[m +
      D->size[0]] * (P->data[(int)(D->data[m] + 1.0) - 1] - P->data[(int)D->
                     data[m] - 1]);
    zml->data[(int)count - 1] = P->data[((int)D->data[m] + P->size[0]) - 1] +
      D->data[m + D->size[0]] * (P->data[((int)(D->data[m] + 1.0) + P->size[0])
      - 1] - P->data[((int)D->data[m] + P->size[0]) - 1]);
    count += 2U;
  }

  // % rechte Seitenlinie
  projectPoints(r, delta, -0.38, y_tilde, z_m, zmm);

  // Punkte von Mittellinie nach rechts projizieren
  cr = y_tilde->size[0];
  br = z_m->size[0];
  ar = zmm->size[0];
  i4 = P->size[0] * P->size[1];
  P->size[0] = cr;
  P->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)P, i4, (int)sizeof(double));
  for (i4 = 0; i4 < cr; i4++) {
    P->data[i4] = y_tilde->data[i4];
  }

  for (i4 = 0; i4 < br; i4++) {
    P->data[i4 + P->size[0]] = z_m->data[i4];
  }

  for (i4 = 0; i4 < ar; i4++) {
    P->data[i4 + (P->size[0] << 1)] = zmm->data[i4];
  }

  i4 = D->size[0] * D->size[1];
  D->size[0] = xr->size[0];
  D->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)D, i4, (int)sizeof(double));
  ib = xr->size[0] * 3;
  for (i4 = 0; i4 < ib; i4++) {
    D->data[i4] = 10000.0;
  }

  //  Fuer jeden Messpunkt den nähesten Punkt der aktuellen Praediktion finden
  for (s = 0; s < r->size[0]; s++) {
    for (m = 0; m < xr->size[0]; m++) {
      minval = P->data[s] - xr->data[m];
      a = P->data[s + P->size[0]] - yr->data[m];
      dist_point = sqrt(minval * minval + a * a);
      if (dist_point < D->data[m + (D->size[0] << 1)]) {
        D->data[m] = 1.0 + (double)s;
        D->data[m + D->size[0]] = 0.0;
        D->data[m + (D->size[0] << 1)] = dist_point;
      }

      if (1.0 + (double)s > 1.0) {
        d_P[0] = P->data[(int)((1.0 + (double)s) - 1.0) - 1];
        d_P[1] = P->data[((int)((1.0 + (double)s) - 1.0) + P->size[0]) - 1];
        e_P[0] = P->data[s];
        e_P[1] = P->data[s + P->size[0]];
        b_xr[0] = xr->data[m];
        b_xr[1] = yr->data[m];
        d_line_point(d_P, e_P, b_xr, &dist_line, &lambda, unusedU2);
        if ((dist_line < D->data[m + (D->size[0] << 1)]) && (lambda > 0.0) &&
            (lambda < 1.0)) {
          D->data[m] = (1.0 + (double)s) - 1.0;
          D->data[m + D->size[0]] = lambda;
          D->data[m + (D->size[0] << 1)] = dist_line;
        }
      }
    }
  }

  //  Messpunkte ausfiltern, die vor dem letzten prädizierten Punkt oder zu weit entfernt liegen 
  //  ind = ((D(:, 1) == numel(r) & D(:, 2) == 0) | D(:, 3) > 0.5 );
  ib = D->size[0];
  br = r->size[0];
  i4 = ind->size[0];
  ind->size[0] = ib;
  emxEnsureCapacity((emxArray__common *)ind, i4, (int)sizeof(boolean_T));
  for (i4 = 0; i4 < ib; i4++) {
    ind->data[i4] = ((D->data[i4] == br) || (D->data[i4 + (D->size[0] << 1)] >
      0.5));
  }

  emxInit_real_T(&d_D, 1);
  eml_null_assignment(xr, ind);
  eml_null_assignment(yr, ind);
  b_eml_null_assignment(D, ind);

  //  Messmatrix, Mess- und Erwartungsvektor bauen
  ib = D->size[0];
  i4 = d_D->size[0];
  d_D->size[0] = ib;
  emxEnsureCapacity((emxArray__common *)d_D, i4, (int)sizeof(double));
  for (i4 = 0; i4 < ib; i4++) {
    d_D->data[i4] = D->data[i4];
  }

  emxInit_real_T(&e_D, 1);
  ib = D->size[0];
  i4 = e_D->size[0];
  e_D->size[0] = ib;
  emxEnsureCapacity((emxArray__common *)e_D, i4, (int)sizeof(double));
  for (i4 = 0; i4 < ib; i4++) {
    e_D->data[i4] = D->data[i4 + D->size[0]];
  }

  emxInit_real_T1(&Hr, 2);
  emxInit_real_T(&zr, 1);
  messmatrix(P, r, delta, d_D, e_D, Hr);
  i4 = zr->size[0];
  zr->size[0] = (int)(2.0 * (double)xr->size[0]);
  emxEnsureCapacity((emxArray__common *)zr, i4, (int)sizeof(double));
  ib = (int)(2.0 * (double)xr->size[0]);
  emxFree_real_T(&e_D);
  emxFree_real_T(&d_D);
  for (i4 = 0; i4 < ib; i4++) {
    zr->data[i4] = 0.0;
  }

  emxInit_real_T(&zmr, 1);
  i4 = zmr->size[0];
  zmr->size[0] = (int)(2.0 * (double)xr->size[0]);
  emxEnsureCapacity((emxArray__common *)zmr, i4, (int)sizeof(double));
  ib = (int)(2.0 * (double)xr->size[0]);
  for (i4 = 0; i4 < ib; i4++) {
    zmr->data[i4] = 0.0;
  }

  count = 2U;
  for (m = 0; m < xr->size[0]; m++) {
    zr->data[(int)count - 2] = xr->data[m];
    zr->data[(int)count - 1] = yr->data[m];
    zmr->data[(int)count - 2] = P->data[(int)D->data[m] - 1] + D->data[m +
      D->size[0]] * (P->data[(int)(D->data[m] + 1.0) - 1] - P->data[(int)D->
                     data[m] - 1]);
    zmr->data[(int)count - 1] = P->data[((int)D->data[m] + P->size[0]) - 1] +
      D->data[m + D->size[0]] * (P->data[((int)(D->data[m] + 1.0) + P->size[0])
      - 1] - P->data[((int)D->data[m] + P->size[0]) - 1]);
    count += 2U;
  }

  // % Mittellinie
  getPointsFromState(r, delta, y_tilde, z_m, zmm);
  cr = y_tilde->size[0];
  br = z_m->size[0];
  ar = zmm->size[0];
  i4 = P->size[0] * P->size[1];
  P->size[0] = cr;
  P->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)P, i4, (int)sizeof(double));
  for (i4 = 0; i4 < cr; i4++) {
    P->data[i4] = y_tilde->data[i4];
  }

  for (i4 = 0; i4 < br; i4++) {
    P->data[i4 + P->size[0]] = z_m->data[i4];
  }

  for (i4 = 0; i4 < ar; i4++) {
    P->data[i4 + (P->size[0] << 1)] = zmm->data[i4];
  }

  i4 = D->size[0] * D->size[1];
  D->size[0] = xm->size[0];
  D->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)D, i4, (int)sizeof(double));
  ib = xm->size[0] * 3;
  for (i4 = 0; i4 < ib; i4++) {
    D->data[i4] = 10000.0;
  }

  //  Fuer jeden Messpunkt den nähesten Punkt der aktuellen Praediktion finden
  for (s = 0; s < r->size[0]; s++) {
    for (m = 0; m < xm->size[0]; m++) {
      minval = P->data[s] - xm->data[m];
      a = P->data[s + P->size[0]] - ym->data[m];
      dist_point = sqrt(minval * minval + a * a);
      if (dist_point < D->data[m + (D->size[0] << 1)]) {
        D->data[m] = 1.0 + (double)s;
        D->data[m + D->size[0]] = 0.0;
        D->data[m + (D->size[0] << 1)] = dist_point;
      }

      if (1.0 + (double)s > 1.0) {
        f_P[0] = P->data[(int)((1.0 + (double)s) - 1.0) - 1];
        f_P[1] = P->data[((int)((1.0 + (double)s) - 1.0) + P->size[0]) - 1];
        g_P[0] = P->data[s];
        g_P[1] = P->data[s + P->size[0]];
        b_xm[0] = xm->data[m];
        b_xm[1] = ym->data[m];
        d_line_point(f_P, g_P, b_xm, &dist_line, &lambda, unusedU2);
        if ((dist_line < D->data[m + (D->size[0] << 1)]) && (lambda > 0.0) &&
            (lambda < 1.0)) {
          D->data[m] = (1.0 + (double)s) - 1.0;
          D->data[m + D->size[0]] = lambda;
          D->data[m + (D->size[0] << 1)] = dist_line;
        }
      }
    }
  }

  //  Messpunkte ausfiltern, die vor dem letzten prädizierten Punkt oder zu weit entfernt liegen 
  ib = D->size[0];
  br = r->size[0];
  i4 = ind->size[0];
  ind->size[0] = ib;
  emxEnsureCapacity((emxArray__common *)ind, i4, (int)sizeof(boolean_T));
  for (i4 = 0; i4 < ib; i4++) {
    ind->data[i4] = ((D->data[i4] == br) || (D->data[i4 + (D->size[0] << 1)] >
      0.5));
  }

  emxInit_real_T(&f_D, 1);
  eml_null_assignment(xm, ind);
  eml_null_assignment(ym, ind);
  b_eml_null_assignment(D, ind);

  //  Messmatrix, Mess- und Erwartungsvektor bauen
  ib = D->size[0];
  i4 = f_D->size[0];
  f_D->size[0] = ib;
  emxEnsureCapacity((emxArray__common *)f_D, i4, (int)sizeof(double));
  emxFree_boolean_T(&ind);
  for (i4 = 0; i4 < ib; i4++) {
    f_D->data[i4] = D->data[i4];
  }

  emxInit_real_T(&g_D, 1);
  ib = D->size[0];
  i4 = g_D->size[0];
  g_D->size[0] = ib;
  emxEnsureCapacity((emxArray__common *)g_D, i4, (int)sizeof(double));
  for (i4 = 0; i4 < ib; i4++) {
    g_D->data[i4] = D->data[i4 + D->size[0]];
  }

  emxInit_real_T1(&Hm, 2);
  messmatrix(P, r, delta, f_D, g_D, Hm);
  i4 = z_m->size[0];
  z_m->size[0] = (int)(2.0 * (double)xm->size[0]);
  emxEnsureCapacity((emxArray__common *)z_m, i4, (int)sizeof(double));
  ib = (int)(2.0 * (double)xm->size[0]);
  emxFree_real_T(&g_D);
  emxFree_real_T(&f_D);
  for (i4 = 0; i4 < ib; i4++) {
    z_m->data[i4] = 0.0;
  }

  i4 = zmm->size[0];
  zmm->size[0] = (int)(2.0 * (double)xm->size[0]);
  emxEnsureCapacity((emxArray__common *)zmm, i4, (int)sizeof(double));
  ib = (int)(2.0 * (double)xm->size[0]);
  for (i4 = 0; i4 < ib; i4++) {
    zmm->data[i4] = 0.0;
  }

  count = 2U;
  for (m = 0; m < xm->size[0]; m++) {
    z_m->data[(int)count - 2] = xm->data[m];
    z_m->data[(int)count - 1] = ym->data[m];
    zmm->data[(int)count - 2] = P->data[(int)D->data[m] - 1] + D->data[m +
      D->size[0]] * (P->data[(int)(D->data[m] + 1.0) - 1] - P->data[(int)D->
                     data[m] - 1]);
    zmm->data[(int)count - 1] = P->data[((int)D->data[m] + P->size[0]) - 1] +
      D->data[m + D->size[0]] * (P->data[((int)(D->data[m] + 1.0) + P->size[0])
      - 1] - P->data[((int)D->data[m] + P->size[0]) - 1]);
    count += 2U;
  }

  emxFree_real_T(&D);
  emxFree_real_T(&P);
  emxInit_real_T1(&H, 2);

  // % Linke und rechte Linie kombinieren
  i4 = H->size[0] * H->size[1];
  H->size[0] = (Hl->size[0] + Hr->size[0]) + Hm->size[0];
  H->size[1] = Hl->size[1];
  emxEnsureCapacity((emxArray__common *)H, i4, (int)sizeof(double));
  ib = Hl->size[1];
  for (i4 = 0; i4 < ib; i4++) {
    ar = Hl->size[0];
    for (i5 = 0; i5 < ar; i5++) {
      H->data[i5 + H->size[0] * i4] = Hl->data[i5 + Hl->size[0] * i4];
    }
  }

  ib = Hr->size[1];
  for (i4 = 0; i4 < ib; i4++) {
    ar = Hr->size[0];
    for (i5 = 0; i5 < ar; i5++) {
      H->data[(i5 + Hl->size[0]) + H->size[0] * i4] = Hr->data[i5 + Hr->size[0] *
        i4];
    }
  }

  ib = Hm->size[1];
  for (i4 = 0; i4 < ib; i4++) {
    ar = Hm->size[0];
    for (i5 = 0; i5 < ar; i5++) {
      H->data[((i5 + Hl->size[0]) + Hr->size[0]) + H->size[0] * i4] = Hm->
        data[i5 + Hm->size[0] * i4];
    }
  }

  // Messmatrix
  // Messwerte
  // erwartete Werte
  // % KALMAN FILTER
  i4 = Hl->size[0] * Hl->size[1];
  Hl->size[0] = r->size[0];
  Hl->size[1] = r->size[0];
  emxEnsureCapacity((emxArray__common *)Hl, i4, (int)sizeof(double));
  ib = r->size[0] * r->size[0];
  for (i4 = 0; i4 < ib; i4++) {
    Hl->data[i4] = 0.0;
  }

  Hl->data[0] = 1.0;
  Hl->data[1 + Hl->size[0]] = 1.0;
  Hl->data[1 + (Hl->size[0] << 1)] = 0.013 / sqrt(1.0 - delta * delta * (r->
    data[2] * r->data[2]) / 4.0);
  for (i = 2; i - 2 < (int)(((double)r->size[0] - 1.0) + -2.0); i++) {
    Hl->data[i + Hl->size[0] * i] = 1.0 - 0.013 / delta;
    Hl->data[i + Hl->size[0] * ((int)((3.0 + (double)(i - 2)) + 1.0) - 1)] =
      0.013 / delta;
  }

  Hl->data[(r->size[0] + Hl->size[0] * (r->size[0] - 1)) - 1] = 1.0;
  i4 = y_tilde->size[0];
  y_tilde->size[0] = r->size[0];
  emxEnsureCapacity((emxArray__common *)y_tilde, i4, (int)sizeof(double));
  ib = r->size[0];
  for (i4 = 0; i4 < ib; i4++) {
    y_tilde->data[i4] = r->data[i4];
  }

  emxInit_real_T(&b_Hl, 1);
  if ((Hl->size[1] == 1) || (r->size[0] == 1)) {
    i4 = b_Hl->size[0];
    b_Hl->size[0] = Hl->size[0];
    emxEnsureCapacity((emxArray__common *)b_Hl, i4, (int)sizeof(double));
    ib = Hl->size[0];
    for (i4 = 0; i4 < ib; i4++) {
      b_Hl->data[i4] = 0.0;
      ar = Hl->size[1];
      for (i5 = 0; i5 < ar; i5++) {
        b_Hl->data[i4] += Hl->data[i4 + Hl->size[0] * i5] * r->data[i5];
      }
    }

    i4 = r->size[0];
    r->size[0] = b_Hl->size[0];
    emxEnsureCapacity((emxArray__common *)r, i4, (int)sizeof(double));
    ib = b_Hl->size[0];
    for (i4 = 0; i4 < ib; i4++) {
      r->data[i4] = b_Hl->data[i4];
    }
  } else {
    k = Hl->size[1];
    Hl_idx_0 = (unsigned int)Hl->size[0];
    i4 = r->size[0];
    r->size[0] = (int)Hl_idx_0;
    emxEnsureCapacity((emxArray__common *)r, i4, (int)sizeof(double));
    m = Hl->size[0];
    i4 = r->size[0];
    emxEnsureCapacity((emxArray__common *)r, i4, (int)sizeof(double));
    ib = r->size[0];
    for (i4 = 0; i4 < ib; i4++) {
      r->data[i4] = 0.0;
    }

    if (Hl->size[0] == 0) {
    } else {
      cr = 0;
      while ((m > 0) && (cr <= 0)) {
        for (ic = 1; ic <= m; ic++) {
          r->data[ic - 1] = 0.0;
        }

        cr = m;
      }

      br = 0;
      cr = 0;
      while ((m > 0) && (cr <= 0)) {
        ar = 0;
        i4 = br + k;
        for (ib = br; ib + 1 <= i4; ib++) {
          if (y_tilde->data[ib] != 0.0) {
            ia = ar;
            for (ic = 0; ic + 1 <= m; ic++) {
              ia++;
              r->data[ic] += y_tilde->data[ib] * Hl->data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        cr = m;
      }
    }
  }

  emxFree_real_T(&b_Hl);
  eye(2.0 * (((double)xl->size[0] + (double)xr->size[0]) + (double)xm->size[0]),
      Hr);
  i4 = Hr->size[0] * Hr->size[1];
  emxEnsureCapacity((emxArray__common *)Hr, i4, (int)sizeof(double));
  br = Hr->size[0];
  cr = Hr->size[1];
  ib = br * cr;
  for (i4 = 0; i4 < ib; i4++) {
    Hr->data[i4] *= R_fakt;
  }

  emxInit_real_T1(&y, 2);
  if ((Hl->size[1] == 1) || (Pk->size[0] == 1)) {
    i4 = y->size[0] * y->size[1];
    y->size[0] = Hl->size[0];
    y->size[1] = Pk->size[1];
    emxEnsureCapacity((emxArray__common *)y, i4, (int)sizeof(double));
    ib = Hl->size[0];
    for (i4 = 0; i4 < ib; i4++) {
      ar = Pk->size[1];
      for (i5 = 0; i5 < ar; i5++) {
        y->data[i4 + y->size[0] * i5] = 0.0;
        cr = Hl->size[1];
        for (br = 0; br < cr; br++) {
          y->data[i4 + y->size[0] * i5] += Hl->data[i4 + Hl->size[0] * br] *
            Pk->data[br + Pk->size[0] * i5];
        }
      }
    }
  } else {
    k = Hl->size[1];
    unusedU2[0] = Hl->size[0];
    unusedU2[1] = Pk->size[1];
    i4 = y->size[0] * y->size[1];
    y->size[0] = (int)unusedU2[0];
    y->size[1] = (int)unusedU2[1];
    emxEnsureCapacity((emxArray__common *)y, i4, (int)sizeof(double));
    m = Hl->size[0];
    i4 = y->size[0] * y->size[1];
    emxEnsureCapacity((emxArray__common *)y, i4, (int)sizeof(double));
    ib = y->size[1];
    for (i4 = 0; i4 < ib; i4++) {
      ar = y->size[0];
      for (i5 = 0; i5 < ar; i5++) {
        y->data[i5 + y->size[0] * i4] = 0.0;
      }
    }

    if ((Hl->size[0] == 0) || (Pk->size[1] == 0)) {
    } else {
      c = Hl->size[0] * (Pk->size[1] - 1);
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        i4 = cr + m;
        for (ic = cr; ic + 1 <= i4; ic++) {
          y->data[ic] = 0.0;
        }

        cr += m;
      }

      br = 0;
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        ar = 0;
        i4 = br + k;
        for (ib = br; ib + 1 <= i4; ib++) {
          if (Pk->data[ib] != 0.0) {
            ia = ar;
            i5 = cr + m;
            for (ic = cr; ic + 1 <= i5; ic++) {
              ia++;
              y->data[ic] += Pk->data[ib] * Hl->data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        cr += m;
      }
    }
  }

  i4 = Hm->size[0] * Hm->size[1];
  Hm->size[0] = Hl->size[1];
  Hm->size[1] = Hl->size[0];
  emxEnsureCapacity((emxArray__common *)Hm, i4, (int)sizeof(double));
  ib = Hl->size[0];
  for (i4 = 0; i4 < ib; i4++) {
    ar = Hl->size[1];
    for (i5 = 0; i5 < ar; i5++) {
      Hm->data[i5 + Hm->size[0] * i4] = Hl->data[i4 + Hl->size[0] * i5];
    }
  }

  if ((y->size[1] == 1) || (Hm->size[0] == 1)) {
    i4 = Pk->size[0] * Pk->size[1];
    Pk->size[0] = y->size[0];
    Pk->size[1] = Hm->size[1];
    emxEnsureCapacity((emxArray__common *)Pk, i4, (int)sizeof(double));
    ib = y->size[0];
    for (i4 = 0; i4 < ib; i4++) {
      ar = Hm->size[1];
      for (i5 = 0; i5 < ar; i5++) {
        Pk->data[i4 + Pk->size[0] * i5] = 0.0;
        cr = y->size[1];
        for (br = 0; br < cr; br++) {
          Pk->data[i4 + Pk->size[0] * i5] += y->data[i4 + y->size[0] * br] *
            Hm->data[br + Hm->size[0] * i5];
        }
      }
    }
  } else {
    k = y->size[1];
    unusedU2[0] = (unsigned int)y->size[0];
    unusedU2[1] = (unsigned int)Hm->size[1];
    i4 = Pk->size[0] * Pk->size[1];
    Pk->size[0] = (int)unusedU2[0];
    Pk->size[1] = (int)unusedU2[1];
    emxEnsureCapacity((emxArray__common *)Pk, i4, (int)sizeof(double));
    m = y->size[0];
    i4 = Pk->size[0] * Pk->size[1];
    emxEnsureCapacity((emxArray__common *)Pk, i4, (int)sizeof(double));
    ib = Pk->size[1];
    for (i4 = 0; i4 < ib; i4++) {
      ar = Pk->size[0];
      for (i5 = 0; i5 < ar; i5++) {
        Pk->data[i5 + Pk->size[0] * i4] = 0.0;
      }
    }

    if ((y->size[0] == 0) || (Hm->size[1] == 0)) {
    } else {
      c = y->size[0] * (Hm->size[1] - 1);
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        i4 = cr + m;
        for (ic = cr; ic + 1 <= i4; ic++) {
          Pk->data[ic] = 0.0;
        }

        cr += m;
      }

      br = 0;
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        ar = 0;
        i4 = br + k;
        for (ib = br; ib + 1 <= i4; ib++) {
          if (Hm->data[ib] != 0.0) {
            ia = ar;
            i5 = cr + m;
            for (ic = cr; ic + 1 <= i5; ic++) {
              ia++;
              Pk->data[ic] += Hm->data[ib] * y->data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        cr += m;
      }
    }
  }

  emxFree_real_T(&y);
  i4 = Pk->size[0] * Pk->size[1];
  emxEnsureCapacity((emxArray__common *)Pk, i4, (int)sizeof(double));
  ib = Pk->size[1];
  for (i4 = 0; i4 < ib; i4++) {
    ar = Pk->size[0];
    for (i5 = 0; i5 < ar; i5++) {
      Pk->data[i5 + Pk->size[0] * i4] += Q->data[i5 + Q->size[0] * i4];
    }
  }

  emxInit_real_T(&b_zl, 1);
  emxInit_real_T(&b_zml, 1);
  i4 = b_zl->size[0];
  b_zl->size[0] = (zl->size[0] + zr->size[0]) + z_m->size[0];
  emxEnsureCapacity((emxArray__common *)b_zl, i4, (int)sizeof(double));
  ib = zl->size[0];
  for (i4 = 0; i4 < ib; i4++) {
    b_zl->data[i4] = zl->data[i4];
  }

  ib = zr->size[0];
  for (i4 = 0; i4 < ib; i4++) {
    b_zl->data[i4 + zl->size[0]] = zr->data[i4];
  }

  ib = z_m->size[0];
  for (i4 = 0; i4 < ib; i4++) {
    b_zl->data[(i4 + zl->size[0]) + zr->size[0]] = z_m->data[i4];
  }

  emxFree_real_T(&z_m);
  emxFree_real_T(&zr);
  emxFree_real_T(&zl);
  i4 = b_zml->size[0];
  b_zml->size[0] = (zml->size[0] + zmr->size[0]) + zmm->size[0];
  emxEnsureCapacity((emxArray__common *)b_zml, i4, (int)sizeof(double));
  ib = zml->size[0];
  for (i4 = 0; i4 < ib; i4++) {
    b_zml->data[i4] = zml->data[i4];
  }

  ib = zmr->size[0];
  for (i4 = 0; i4 < ib; i4++) {
    b_zml->data[i4 + zml->size[0]] = zmr->data[i4];
  }

  ib = zmm->size[0];
  for (i4 = 0; i4 < ib; i4++) {
    b_zml->data[(i4 + zml->size[0]) + zmr->size[0]] = zmm->data[i4];
  }

  emxFree_real_T(&zmm);
  emxFree_real_T(&zmr);
  emxFree_real_T(&zml);
  i4 = y_tilde->size[0];
  y_tilde->size[0] = b_zl->size[0];
  emxEnsureCapacity((emxArray__common *)y_tilde, i4, (int)sizeof(double));
  ib = b_zl->size[0];
  for (i4 = 0; i4 < ib; i4++) {
    y_tilde->data[i4] = b_zl->data[i4] - b_zml->data[i4];
  }

  emxFree_real_T(&b_zml);
  emxFree_real_T(&b_zl);
  emxInit_real_T1(&b_y, 2);
  if ((H->size[1] == 1) || (Pk->size[0] == 1)) {
    i4 = b_y->size[0] * b_y->size[1];
    b_y->size[0] = H->size[0];
    b_y->size[1] = Pk->size[1];
    emxEnsureCapacity((emxArray__common *)b_y, i4, (int)sizeof(double));
    ib = H->size[0];
    for (i4 = 0; i4 < ib; i4++) {
      ar = Pk->size[1];
      for (i5 = 0; i5 < ar; i5++) {
        b_y->data[i4 + b_y->size[0] * i5] = 0.0;
        cr = H->size[1];
        for (br = 0; br < cr; br++) {
          b_y->data[i4 + b_y->size[0] * i5] += H->data[i4 + H->size[0] * br] *
            Pk->data[br + Pk->size[0] * i5];
        }
      }
    }
  } else {
    k = H->size[1];
    unusedU2[0] = H->size[0];
    unusedU2[1] = Pk->size[1];
    i4 = b_y->size[0] * b_y->size[1];
    b_y->size[0] = (int)unusedU2[0];
    b_y->size[1] = (int)unusedU2[1];
    emxEnsureCapacity((emxArray__common *)b_y, i4, (int)sizeof(double));
    m = H->size[0];
    i4 = b_y->size[0] * b_y->size[1];
    emxEnsureCapacity((emxArray__common *)b_y, i4, (int)sizeof(double));
    ib = b_y->size[1];
    for (i4 = 0; i4 < ib; i4++) {
      ar = b_y->size[0];
      for (i5 = 0; i5 < ar; i5++) {
        b_y->data[i5 + b_y->size[0] * i4] = 0.0;
      }
    }

    if ((H->size[0] == 0) || (Pk->size[1] == 0)) {
    } else {
      c = H->size[0] * (Pk->size[1] - 1);
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        i4 = cr + m;
        for (ic = cr; ic + 1 <= i4; ic++) {
          b_y->data[ic] = 0.0;
        }

        cr += m;
      }

      br = 0;
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        ar = 0;
        i4 = br + k;
        for (ib = br; ib + 1 <= i4; ib++) {
          if (Pk->data[ib] != 0.0) {
            ia = ar;
            i5 = cr + m;
            for (ic = cr; ic + 1 <= i5; ic++) {
              ia++;
              b_y->data[ic] += Pk->data[ib] * H->data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        cr += m;
      }
    }
  }

  i4 = Hm->size[0] * Hm->size[1];
  Hm->size[0] = H->size[1];
  Hm->size[1] = H->size[0];
  emxEnsureCapacity((emxArray__common *)Hm, i4, (int)sizeof(double));
  ib = H->size[0];
  for (i4 = 0; i4 < ib; i4++) {
    ar = H->size[1];
    for (i5 = 0; i5 < ar; i5++) {
      Hm->data[i5 + Hm->size[0] * i4] = H->data[i4 + H->size[0] * i5];
    }
  }

  emxInit_real_T1(&C, 2);
  if ((b_y->size[1] == 1) || (Hm->size[0] == 1)) {
    i4 = C->size[0] * C->size[1];
    C->size[0] = b_y->size[0];
    C->size[1] = Hm->size[1];
    emxEnsureCapacity((emxArray__common *)C, i4, (int)sizeof(double));
    ib = b_y->size[0];
    for (i4 = 0; i4 < ib; i4++) {
      ar = Hm->size[1];
      for (i5 = 0; i5 < ar; i5++) {
        C->data[i4 + C->size[0] * i5] = 0.0;
        cr = b_y->size[1];
        for (br = 0; br < cr; br++) {
          C->data[i4 + C->size[0] * i5] += b_y->data[i4 + b_y->size[0] * br] *
            Hm->data[br + Hm->size[0] * i5];
        }
      }
    }
  } else {
    k = b_y->size[1];
    unusedU2[0] = (unsigned int)b_y->size[0];
    unusedU2[1] = (unsigned int)Hm->size[1];
    i4 = C->size[0] * C->size[1];
    C->size[0] = (int)unusedU2[0];
    C->size[1] = (int)unusedU2[1];
    emxEnsureCapacity((emxArray__common *)C, i4, (int)sizeof(double));
    m = b_y->size[0];
    i4 = C->size[0] * C->size[1];
    emxEnsureCapacity((emxArray__common *)C, i4, (int)sizeof(double));
    ib = C->size[1];
    for (i4 = 0; i4 < ib; i4++) {
      ar = C->size[0];
      for (i5 = 0; i5 < ar; i5++) {
        C->data[i5 + C->size[0] * i4] = 0.0;
      }
    }

    if ((b_y->size[0] == 0) || (Hm->size[1] == 0)) {
    } else {
      c = b_y->size[0] * (Hm->size[1] - 1);
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        i4 = cr + m;
        for (ic = cr; ic + 1 <= i4; ic++) {
          C->data[ic] = 0.0;
        }

        cr += m;
      }

      br = 0;
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        ar = 0;
        i4 = br + k;
        for (ib = br; ib + 1 <= i4; ib++) {
          if (Hm->data[ib] != 0.0) {
            ia = ar;
            i5 = cr + m;
            for (ic = cr; ic + 1 <= i5; ic++) {
              ia++;
              C->data[ic] += Hm->data[ib] * b_y->data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        cr += m;
      }
    }
  }

  emxFree_real_T(&b_y);
  i4 = Hm->size[0] * Hm->size[1];
  Hm->size[0] = H->size[1];
  Hm->size[1] = H->size[0];
  emxEnsureCapacity((emxArray__common *)Hm, i4, (int)sizeof(double));
  ib = H->size[0];
  for (i4 = 0; i4 < ib; i4++) {
    ar = H->size[1];
    for (i5 = 0; i5 < ar; i5++) {
      Hm->data[i5 + Hm->size[0] * i4] = H->data[i4 + H->size[0] * i5];
    }
  }

  emxInit_real_T1(&c_y, 2);
  if ((Pk->size[1] == 1) || (Hm->size[0] == 1)) {
    i4 = c_y->size[0] * c_y->size[1];
    c_y->size[0] = Pk->size[0];
    c_y->size[1] = Hm->size[1];
    emxEnsureCapacity((emxArray__common *)c_y, i4, (int)sizeof(double));
    ib = Pk->size[0];
    for (i4 = 0; i4 < ib; i4++) {
      ar = Hm->size[1];
      for (i5 = 0; i5 < ar; i5++) {
        c_y->data[i4 + c_y->size[0] * i5] = 0.0;
        cr = Pk->size[1];
        for (br = 0; br < cr; br++) {
          c_y->data[i4 + c_y->size[0] * i5] += Pk->data[i4 + Pk->size[0] * br] *
            Hm->data[br + Hm->size[0] * i5];
        }
      }
    }
  } else {
    k = Pk->size[1];
    unusedU2[0] = Pk->size[0];
    unusedU2[1] = Hm->size[1];
    i4 = c_y->size[0] * c_y->size[1];
    c_y->size[0] = (int)unusedU2[0];
    c_y->size[1] = (int)unusedU2[1];
    emxEnsureCapacity((emxArray__common *)c_y, i4, (int)sizeof(double));
    m = Pk->size[0];
    i4 = c_y->size[0] * c_y->size[1];
    emxEnsureCapacity((emxArray__common *)c_y, i4, (int)sizeof(double));
    ib = c_y->size[1];
    for (i4 = 0; i4 < ib; i4++) {
      ar = c_y->size[0];
      for (i5 = 0; i5 < ar; i5++) {
        c_y->data[i5 + c_y->size[0] * i4] = 0.0;
      }
    }

    if ((Pk->size[0] == 0) || (Hm->size[1] == 0)) {
    } else {
      c = Pk->size[0] * (Hm->size[1] - 1);
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        i4 = cr + m;
        for (ic = cr; ic + 1 <= i4; ic++) {
          c_y->data[ic] = 0.0;
        }

        cr += m;
      }

      br = 0;
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        ar = 0;
        i4 = br + k;
        for (ib = br; ib + 1 <= i4; ib++) {
          if (Hm->data[ib] != 0.0) {
            ia = ar;
            i5 = cr + m;
            for (ic = cr; ic + 1 <= i5; ic++) {
              ia++;
              c_y->data[ic] += Hm->data[ib] * Pk->data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        cr += m;
      }
    }
  }

  emxInit_real_T1(&b_C, 2);
  i4 = b_C->size[0] * b_C->size[1];
  b_C->size[0] = C->size[0];
  b_C->size[1] = C->size[1];
  emxEnsureCapacity((emxArray__common *)b_C, i4, (int)sizeof(double));
  ib = C->size[0] * C->size[1];
  for (i4 = 0; i4 < ib; i4++) {
    b_C->data[i4] = C->data[i4] + Hr->data[i4];
  }

  emxFree_real_T(&C);
  mrdivide(c_y, b_C, Hl);
  emxFree_real_T(&b_C);
  emxFree_real_T(&c_y);
  emxInit_real_T(&c_C, 1);
  if ((Hl->size[1] == 1) || (y_tilde->size[0] == 1)) {
    i4 = c_C->size[0];
    c_C->size[0] = Hl->size[0];
    emxEnsureCapacity((emxArray__common *)c_C, i4, (int)sizeof(double));
    ib = Hl->size[0];
    for (i4 = 0; i4 < ib; i4++) {
      c_C->data[i4] = 0.0;
      ar = Hl->size[1];
      for (i5 = 0; i5 < ar; i5++) {
        c_C->data[i4] += Hl->data[i4 + Hl->size[0] * i5] * y_tilde->data[i5];
      }
    }
  } else {
    k = Hl->size[1];
    Hl_idx_0 = (unsigned int)Hl->size[0];
    i4 = c_C->size[0];
    c_C->size[0] = (int)Hl_idx_0;
    emxEnsureCapacity((emxArray__common *)c_C, i4, (int)sizeof(double));
    m = Hl->size[0];
    cr = c_C->size[0];
    i4 = c_C->size[0];
    c_C->size[0] = cr;
    emxEnsureCapacity((emxArray__common *)c_C, i4, (int)sizeof(double));
    for (i4 = 0; i4 < cr; i4++) {
      c_C->data[i4] = 0.0;
    }

    if (Hl->size[0] == 0) {
    } else {
      cr = 0;
      while ((m > 0) && (cr <= 0)) {
        for (ic = 1; ic <= m; ic++) {
          c_C->data[ic - 1] = 0.0;
        }

        cr = m;
      }

      br = 0;
      cr = 0;
      while ((m > 0) && (cr <= 0)) {
        ar = 0;
        i4 = br + k;
        for (ib = br; ib + 1 <= i4; ib++) {
          if (y_tilde->data[ib] != 0.0) {
            ia = ar;
            for (ic = 0; ic + 1 <= m; ic++) {
              ia++;
              c_C->data[ic] += y_tilde->data[ib] * Hl->data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        cr = m;
      }
    }
  }

  emxFree_real_T(&y_tilde);
  i4 = r->size[0];
  emxEnsureCapacity((emxArray__common *)r, i4, (int)sizeof(double));
  ib = r->size[0];
  for (i4 = 0; i4 < ib; i4++) {
    r->data[i4] += c_C->data[i4];
  }

  emxFree_real_T(&c_C);
  eye((double)r->size[0], Hr);
  emxInit_real_T1(&d_C, 2);
  if ((Hl->size[1] == 1) || (H->size[0] == 1)) {
    i4 = d_C->size[0] * d_C->size[1];
    d_C->size[0] = Hl->size[0];
    d_C->size[1] = H->size[1];
    emxEnsureCapacity((emxArray__common *)d_C, i4, (int)sizeof(double));
    ib = Hl->size[0];
    for (i4 = 0; i4 < ib; i4++) {
      ar = H->size[1];
      for (i5 = 0; i5 < ar; i5++) {
        d_C->data[i4 + d_C->size[0] * i5] = 0.0;
        cr = Hl->size[1];
        for (br = 0; br < cr; br++) {
          d_C->data[i4 + d_C->size[0] * i5] += Hl->data[i4 + Hl->size[0] * br] *
            H->data[br + H->size[0] * i5];
        }
      }
    }
  } else {
    k = Hl->size[1];
    unusedU2[0] = (unsigned int)Hl->size[0];
    unusedU2[1] = (unsigned int)H->size[1];
    i4 = d_C->size[0] * d_C->size[1];
    d_C->size[0] = (int)unusedU2[0];
    d_C->size[1] = (int)unusedU2[1];
    emxEnsureCapacity((emxArray__common *)d_C, i4, (int)sizeof(double));
    m = Hl->size[0];
    i4 = d_C->size[0] * d_C->size[1];
    emxEnsureCapacity((emxArray__common *)d_C, i4, (int)sizeof(double));
    ib = d_C->size[1];
    for (i4 = 0; i4 < ib; i4++) {
      ar = d_C->size[0];
      for (i5 = 0; i5 < ar; i5++) {
        d_C->data[i5 + d_C->size[0] * i4] = 0.0;
      }
    }

    if ((Hl->size[0] == 0) || (H->size[1] == 0)) {
    } else {
      c = Hl->size[0] * (H->size[1] - 1);
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        i4 = cr + m;
        for (ic = cr; ic + 1 <= i4; ic++) {
          d_C->data[ic] = 0.0;
        }

        cr += m;
      }

      br = 0;
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        ar = 0;
        i4 = br + k;
        for (ib = br; ib + 1 <= i4; ib++) {
          if (H->data[ib] != 0.0) {
            ia = ar;
            i5 = cr + m;
            for (ic = cr; ic + 1 <= i5; ic++) {
              ia++;
              d_C->data[ic] += H->data[ib] * Hl->data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        cr += m;
      }
    }
  }

  emxFree_real_T(&H);
  emxFree_real_T(&Hl);
  i4 = Hr->size[0] * Hr->size[1];
  emxEnsureCapacity((emxArray__common *)Hr, i4, (int)sizeof(double));
  br = Hr->size[0];
  cr = Hr->size[1];
  ib = br * cr;
  for (i4 = 0; i4 < ib; i4++) {
    Hr->data[i4] -= d_C->data[i4];
  }

  emxFree_real_T(&d_C);
  i4 = Hm->size[0] * Hm->size[1];
  Hm->size[0] = Pk->size[0];
  Hm->size[1] = Pk->size[1];
  emxEnsureCapacity((emxArray__common *)Hm, i4, (int)sizeof(double));
  ib = Pk->size[0] * Pk->size[1];
  for (i4 = 0; i4 < ib; i4++) {
    Hm->data[i4] = Pk->data[i4];
  }

  emxInit_real_T1(&b_Hr, 2);
  if ((Hr->size[1] == 1) || (Pk->size[0] == 1)) {
    i4 = b_Hr->size[0] * b_Hr->size[1];
    b_Hr->size[0] = Hr->size[0];
    b_Hr->size[1] = Pk->size[1];
    emxEnsureCapacity((emxArray__common *)b_Hr, i4, (int)sizeof(double));
    ib = Hr->size[0];
    for (i4 = 0; i4 < ib; i4++) {
      ar = Pk->size[1];
      for (i5 = 0; i5 < ar; i5++) {
        b_Hr->data[i4 + b_Hr->size[0] * i5] = 0.0;
        cr = Hr->size[1];
        for (br = 0; br < cr; br++) {
          b_Hr->data[i4 + b_Hr->size[0] * i5] += Hr->data[i4 + Hr->size[0] * br]
            * Pk->data[br + Pk->size[0] * i5];
        }
      }
    }

    i4 = Pk->size[0] * Pk->size[1];
    Pk->size[0] = b_Hr->size[0];
    Pk->size[1] = b_Hr->size[1];
    emxEnsureCapacity((emxArray__common *)Pk, i4, (int)sizeof(double));
    ib = b_Hr->size[1];
    for (i4 = 0; i4 < ib; i4++) {
      ar = b_Hr->size[0];
      for (i5 = 0; i5 < ar; i5++) {
        Pk->data[i5 + Pk->size[0] * i4] = b_Hr->data[i5 + b_Hr->size[0] * i4];
      }
    }
  } else {
    k = Hr->size[1];
    unusedU2[0] = Hr->size[0];
    unusedU2[1] = Pk->size[1];
    i4 = Pk->size[0] * Pk->size[1];
    Pk->size[0] = (int)unusedU2[0];
    Pk->size[1] = (int)unusedU2[1];
    emxEnsureCapacity((emxArray__common *)Pk, i4, (int)sizeof(double));
    m = Hr->size[0];
    i4 = Pk->size[0] * Pk->size[1];
    emxEnsureCapacity((emxArray__common *)Pk, i4, (int)sizeof(double));
    ib = Pk->size[1];
    for (i4 = 0; i4 < ib; i4++) {
      ar = Pk->size[0];
      for (i5 = 0; i5 < ar; i5++) {
        Pk->data[i5 + Pk->size[0] * i4] = 0.0;
      }
    }

    if ((Hr->size[0] == 0) || (Hm->size[1] == 0)) {
    } else {
      c = Hr->size[0] * (Hm->size[1] - 1);
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        i4 = cr + m;
        for (ic = cr; ic + 1 <= i4; ic++) {
          Pk->data[ic] = 0.0;
        }

        cr += m;
      }

      br = 0;
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        ar = 0;
        i4 = br + k;
        for (ib = br; ib + 1 <= i4; ib++) {
          if (Hm->data[ib] != 0.0) {
            ia = ar;
            i5 = cr + m;
            for (ic = cr; ic + 1 <= i5; ic++) {
              ia++;
              Pk->data[ic] += Hm->data[ib] * Hr->data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        cr += m;
      }
    }
  }

  emxFree_real_T(&b_Hr);
  emxFree_real_T(&Hm);
  emxFree_real_T(&Hr);

  // % Zustandsbegrenzungen
  //  Krümmung
  //  bezogen auf Mittellinie, 0.72 entspricht einem minimalen Innenradius von 1m 
  i4 = r->size[0];
  for (i = 2; i - 2 < (int)((double)i4 + -2.0); i++) {
    if ((-0.75 >= r->data[i]) || rtIsNaN(r->data[i])) {
      minval = -0.75;
    } else {
      minval = r->data[i];
    }

    r->data[i] = minval;
    if ((0.75 <= r->data[i]) || rtIsNaN(r->data[i])) {
      minval = 0.75;
    } else {
      minval = r->data[i];
    }

    r->data[i] = minval;
  }

  //  y-Wert des ersten Punktes
  if ((0.5 <= r->data[0]) || rtIsNaN(r->data[0])) {
    minval = 0.5;
  } else {
    minval = r->data[0];
  }

  if ((-0.5 >= minval) || rtIsNaN(minval)) {
    r->data[0] = -0.5;
  } else {
    r->data[0] = minval;
  }

  //  Startwinkel
  if ((0.78 <= r->data[1]) || rtIsNaN(r->data[1])) {
    minval = 0.78;
  } else {
    minval = r->data[1];
  }

  if ((-0.78 >= minval) || rtIsNaN(minval)) {
    r->data[1] = -0.78;
  } else {
    r->data[1] = minval;
  }

  //  45°
}

//
// File trailer for kalman_filter_lr.cpp
//
// [EOF]
//
