#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7717500870753027025) {
   out_7717500870753027025[0] = delta_x[0] + nom_x[0];
   out_7717500870753027025[1] = delta_x[1] + nom_x[1];
   out_7717500870753027025[2] = delta_x[2] + nom_x[2];
   out_7717500870753027025[3] = delta_x[3] + nom_x[3];
   out_7717500870753027025[4] = delta_x[4] + nom_x[4];
   out_7717500870753027025[5] = delta_x[5] + nom_x[5];
   out_7717500870753027025[6] = delta_x[6] + nom_x[6];
   out_7717500870753027025[7] = delta_x[7] + nom_x[7];
   out_7717500870753027025[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5756825933590615713) {
   out_5756825933590615713[0] = -nom_x[0] + true_x[0];
   out_5756825933590615713[1] = -nom_x[1] + true_x[1];
   out_5756825933590615713[2] = -nom_x[2] + true_x[2];
   out_5756825933590615713[3] = -nom_x[3] + true_x[3];
   out_5756825933590615713[4] = -nom_x[4] + true_x[4];
   out_5756825933590615713[5] = -nom_x[5] + true_x[5];
   out_5756825933590615713[6] = -nom_x[6] + true_x[6];
   out_5756825933590615713[7] = -nom_x[7] + true_x[7];
   out_5756825933590615713[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8047934900049386876) {
   out_8047934900049386876[0] = 1.0;
   out_8047934900049386876[1] = 0;
   out_8047934900049386876[2] = 0;
   out_8047934900049386876[3] = 0;
   out_8047934900049386876[4] = 0;
   out_8047934900049386876[5] = 0;
   out_8047934900049386876[6] = 0;
   out_8047934900049386876[7] = 0;
   out_8047934900049386876[8] = 0;
   out_8047934900049386876[9] = 0;
   out_8047934900049386876[10] = 1.0;
   out_8047934900049386876[11] = 0;
   out_8047934900049386876[12] = 0;
   out_8047934900049386876[13] = 0;
   out_8047934900049386876[14] = 0;
   out_8047934900049386876[15] = 0;
   out_8047934900049386876[16] = 0;
   out_8047934900049386876[17] = 0;
   out_8047934900049386876[18] = 0;
   out_8047934900049386876[19] = 0;
   out_8047934900049386876[20] = 1.0;
   out_8047934900049386876[21] = 0;
   out_8047934900049386876[22] = 0;
   out_8047934900049386876[23] = 0;
   out_8047934900049386876[24] = 0;
   out_8047934900049386876[25] = 0;
   out_8047934900049386876[26] = 0;
   out_8047934900049386876[27] = 0;
   out_8047934900049386876[28] = 0;
   out_8047934900049386876[29] = 0;
   out_8047934900049386876[30] = 1.0;
   out_8047934900049386876[31] = 0;
   out_8047934900049386876[32] = 0;
   out_8047934900049386876[33] = 0;
   out_8047934900049386876[34] = 0;
   out_8047934900049386876[35] = 0;
   out_8047934900049386876[36] = 0;
   out_8047934900049386876[37] = 0;
   out_8047934900049386876[38] = 0;
   out_8047934900049386876[39] = 0;
   out_8047934900049386876[40] = 1.0;
   out_8047934900049386876[41] = 0;
   out_8047934900049386876[42] = 0;
   out_8047934900049386876[43] = 0;
   out_8047934900049386876[44] = 0;
   out_8047934900049386876[45] = 0;
   out_8047934900049386876[46] = 0;
   out_8047934900049386876[47] = 0;
   out_8047934900049386876[48] = 0;
   out_8047934900049386876[49] = 0;
   out_8047934900049386876[50] = 1.0;
   out_8047934900049386876[51] = 0;
   out_8047934900049386876[52] = 0;
   out_8047934900049386876[53] = 0;
   out_8047934900049386876[54] = 0;
   out_8047934900049386876[55] = 0;
   out_8047934900049386876[56] = 0;
   out_8047934900049386876[57] = 0;
   out_8047934900049386876[58] = 0;
   out_8047934900049386876[59] = 0;
   out_8047934900049386876[60] = 1.0;
   out_8047934900049386876[61] = 0;
   out_8047934900049386876[62] = 0;
   out_8047934900049386876[63] = 0;
   out_8047934900049386876[64] = 0;
   out_8047934900049386876[65] = 0;
   out_8047934900049386876[66] = 0;
   out_8047934900049386876[67] = 0;
   out_8047934900049386876[68] = 0;
   out_8047934900049386876[69] = 0;
   out_8047934900049386876[70] = 1.0;
   out_8047934900049386876[71] = 0;
   out_8047934900049386876[72] = 0;
   out_8047934900049386876[73] = 0;
   out_8047934900049386876[74] = 0;
   out_8047934900049386876[75] = 0;
   out_8047934900049386876[76] = 0;
   out_8047934900049386876[77] = 0;
   out_8047934900049386876[78] = 0;
   out_8047934900049386876[79] = 0;
   out_8047934900049386876[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1531082340704195775) {
   out_1531082340704195775[0] = state[0];
   out_1531082340704195775[1] = state[1];
   out_1531082340704195775[2] = state[2];
   out_1531082340704195775[3] = state[3];
   out_1531082340704195775[4] = state[4];
   out_1531082340704195775[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1531082340704195775[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1531082340704195775[7] = state[7];
   out_1531082340704195775[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5639704107982325358) {
   out_5639704107982325358[0] = 1;
   out_5639704107982325358[1] = 0;
   out_5639704107982325358[2] = 0;
   out_5639704107982325358[3] = 0;
   out_5639704107982325358[4] = 0;
   out_5639704107982325358[5] = 0;
   out_5639704107982325358[6] = 0;
   out_5639704107982325358[7] = 0;
   out_5639704107982325358[8] = 0;
   out_5639704107982325358[9] = 0;
   out_5639704107982325358[10] = 1;
   out_5639704107982325358[11] = 0;
   out_5639704107982325358[12] = 0;
   out_5639704107982325358[13] = 0;
   out_5639704107982325358[14] = 0;
   out_5639704107982325358[15] = 0;
   out_5639704107982325358[16] = 0;
   out_5639704107982325358[17] = 0;
   out_5639704107982325358[18] = 0;
   out_5639704107982325358[19] = 0;
   out_5639704107982325358[20] = 1;
   out_5639704107982325358[21] = 0;
   out_5639704107982325358[22] = 0;
   out_5639704107982325358[23] = 0;
   out_5639704107982325358[24] = 0;
   out_5639704107982325358[25] = 0;
   out_5639704107982325358[26] = 0;
   out_5639704107982325358[27] = 0;
   out_5639704107982325358[28] = 0;
   out_5639704107982325358[29] = 0;
   out_5639704107982325358[30] = 1;
   out_5639704107982325358[31] = 0;
   out_5639704107982325358[32] = 0;
   out_5639704107982325358[33] = 0;
   out_5639704107982325358[34] = 0;
   out_5639704107982325358[35] = 0;
   out_5639704107982325358[36] = 0;
   out_5639704107982325358[37] = 0;
   out_5639704107982325358[38] = 0;
   out_5639704107982325358[39] = 0;
   out_5639704107982325358[40] = 1;
   out_5639704107982325358[41] = 0;
   out_5639704107982325358[42] = 0;
   out_5639704107982325358[43] = 0;
   out_5639704107982325358[44] = 0;
   out_5639704107982325358[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5639704107982325358[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5639704107982325358[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5639704107982325358[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5639704107982325358[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5639704107982325358[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5639704107982325358[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5639704107982325358[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5639704107982325358[53] = -9.8000000000000007*dt;
   out_5639704107982325358[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5639704107982325358[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5639704107982325358[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5639704107982325358[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5639704107982325358[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5639704107982325358[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5639704107982325358[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5639704107982325358[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5639704107982325358[62] = 0;
   out_5639704107982325358[63] = 0;
   out_5639704107982325358[64] = 0;
   out_5639704107982325358[65] = 0;
   out_5639704107982325358[66] = 0;
   out_5639704107982325358[67] = 0;
   out_5639704107982325358[68] = 0;
   out_5639704107982325358[69] = 0;
   out_5639704107982325358[70] = 1;
   out_5639704107982325358[71] = 0;
   out_5639704107982325358[72] = 0;
   out_5639704107982325358[73] = 0;
   out_5639704107982325358[74] = 0;
   out_5639704107982325358[75] = 0;
   out_5639704107982325358[76] = 0;
   out_5639704107982325358[77] = 0;
   out_5639704107982325358[78] = 0;
   out_5639704107982325358[79] = 0;
   out_5639704107982325358[80] = 1;
}
void h_25(double *state, double *unused, double *out_6733206396346686402) {
   out_6733206396346686402[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1835251734303790686) {
   out_1835251734303790686[0] = 0;
   out_1835251734303790686[1] = 0;
   out_1835251734303790686[2] = 0;
   out_1835251734303790686[3] = 0;
   out_1835251734303790686[4] = 0;
   out_1835251734303790686[5] = 0;
   out_1835251734303790686[6] = 1;
   out_1835251734303790686[7] = 0;
   out_1835251734303790686[8] = 0;
}
void h_24(double *state, double *unused, double *out_6326469331191467479) {
   out_6326469331191467479[0] = state[4];
   out_6326469331191467479[1] = state[5];
}
void H_24(double *state, double *unused, double *out_337397864701708880) {
   out_337397864701708880[0] = 0;
   out_337397864701708880[1] = 0;
   out_337397864701708880[2] = 0;
   out_337397864701708880[3] = 0;
   out_337397864701708880[4] = 1;
   out_337397864701708880[5] = 0;
   out_337397864701708880[6] = 0;
   out_337397864701708880[7] = 0;
   out_337397864701708880[8] = 0;
   out_337397864701708880[9] = 0;
   out_337397864701708880[10] = 0;
   out_337397864701708880[11] = 0;
   out_337397864701708880[12] = 0;
   out_337397864701708880[13] = 0;
   out_337397864701708880[14] = 1;
   out_337397864701708880[15] = 0;
   out_337397864701708880[16] = 0;
   out_337397864701708880[17] = 0;
}
void h_30(double *state, double *unused, double *out_7008400458631192291) {
   out_7008400458631192291[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4353584692811039313) {
   out_4353584692811039313[0] = 0;
   out_4353584692811039313[1] = 0;
   out_4353584692811039313[2] = 0;
   out_4353584692811039313[3] = 0;
   out_4353584692811039313[4] = 1;
   out_4353584692811039313[5] = 0;
   out_4353584692811039313[6] = 0;
   out_4353584692811039313[7] = 0;
   out_4353584692811039313[8] = 0;
}
void h_26(double *state, double *unused, double *out_2006727190970346646) {
   out_2006727190970346646[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1906251584570265538) {
   out_1906251584570265538[0] = 0;
   out_1906251584570265538[1] = 0;
   out_1906251584570265538[2] = 0;
   out_1906251584570265538[3] = 0;
   out_1906251584570265538[4] = 0;
   out_1906251584570265538[5] = 0;
   out_1906251584570265538[6] = 0;
   out_1906251584570265538[7] = 1;
   out_1906251584570265538[8] = 0;
}
void h_27(double *state, double *unused, double *out_5042758561611005442) {
   out_5042758561611005442[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2178821381010614402) {
   out_2178821381010614402[0] = 0;
   out_2178821381010614402[1] = 0;
   out_2178821381010614402[2] = 0;
   out_2178821381010614402[3] = 1;
   out_2178821381010614402[4] = 0;
   out_2178821381010614402[5] = 0;
   out_2178821381010614402[6] = 0;
   out_2178821381010614402[7] = 0;
   out_2178821381010614402[8] = 0;
}
void h_29(double *state, double *unused, double *out_6212436399243034378) {
   out_6212436399243034378[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4863816037125431497) {
   out_4863816037125431497[0] = 0;
   out_4863816037125431497[1] = 1;
   out_4863816037125431497[2] = 0;
   out_4863816037125431497[3] = 0;
   out_4863816037125431497[4] = 0;
   out_4863816037125431497[5] = 0;
   out_4863816037125431497[6] = 0;
   out_4863816037125431497[7] = 0;
   out_4863816037125431497[8] = 0;
}
void h_28(double *state, double *unused, double *out_8402312244437700529) {
   out_8402312244437700529[0] = state[0];
}
void H_28(double *state, double *unused, double *out_218582979944099077) {
   out_218582979944099077[0] = 1;
   out_218582979944099077[1] = 0;
   out_218582979944099077[2] = 0;
   out_218582979944099077[3] = 0;
   out_218582979944099077[4] = 0;
   out_218582979944099077[5] = 0;
   out_218582979944099077[6] = 0;
   out_218582979944099077[7] = 0;
   out_218582979944099077[8] = 0;
}
void h_31(double *state, double *unused, double *out_3216876996099125222) {
   out_3216876996099125222[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2532459686803617014) {
   out_2532459686803617014[0] = 0;
   out_2532459686803617014[1] = 0;
   out_2532459686803617014[2] = 0;
   out_2532459686803617014[3] = 0;
   out_2532459686803617014[4] = 0;
   out_2532459686803617014[5] = 0;
   out_2532459686803617014[6] = 0;
   out_2532459686803617014[7] = 0;
   out_2532459686803617014[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_7717500870753027025) {
  err_fun(nom_x, delta_x, out_7717500870753027025);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5756825933590615713) {
  inv_err_fun(nom_x, true_x, out_5756825933590615713);
}
void car_H_mod_fun(double *state, double *out_8047934900049386876) {
  H_mod_fun(state, out_8047934900049386876);
}
void car_f_fun(double *state, double dt, double *out_1531082340704195775) {
  f_fun(state,  dt, out_1531082340704195775);
}
void car_F_fun(double *state, double dt, double *out_5639704107982325358) {
  F_fun(state,  dt, out_5639704107982325358);
}
void car_h_25(double *state, double *unused, double *out_6733206396346686402) {
  h_25(state, unused, out_6733206396346686402);
}
void car_H_25(double *state, double *unused, double *out_1835251734303790686) {
  H_25(state, unused, out_1835251734303790686);
}
void car_h_24(double *state, double *unused, double *out_6326469331191467479) {
  h_24(state, unused, out_6326469331191467479);
}
void car_H_24(double *state, double *unused, double *out_337397864701708880) {
  H_24(state, unused, out_337397864701708880);
}
void car_h_30(double *state, double *unused, double *out_7008400458631192291) {
  h_30(state, unused, out_7008400458631192291);
}
void car_H_30(double *state, double *unused, double *out_4353584692811039313) {
  H_30(state, unused, out_4353584692811039313);
}
void car_h_26(double *state, double *unused, double *out_2006727190970346646) {
  h_26(state, unused, out_2006727190970346646);
}
void car_H_26(double *state, double *unused, double *out_1906251584570265538) {
  H_26(state, unused, out_1906251584570265538);
}
void car_h_27(double *state, double *unused, double *out_5042758561611005442) {
  h_27(state, unused, out_5042758561611005442);
}
void car_H_27(double *state, double *unused, double *out_2178821381010614402) {
  H_27(state, unused, out_2178821381010614402);
}
void car_h_29(double *state, double *unused, double *out_6212436399243034378) {
  h_29(state, unused, out_6212436399243034378);
}
void car_H_29(double *state, double *unused, double *out_4863816037125431497) {
  H_29(state, unused, out_4863816037125431497);
}
void car_h_28(double *state, double *unused, double *out_8402312244437700529) {
  h_28(state, unused, out_8402312244437700529);
}
void car_H_28(double *state, double *unused, double *out_218582979944099077) {
  H_28(state, unused, out_218582979944099077);
}
void car_h_31(double *state, double *unused, double *out_3216876996099125222) {
  h_31(state, unused, out_3216876996099125222);
}
void car_H_31(double *state, double *unused, double *out_2532459686803617014) {
  H_31(state, unused, out_2532459686803617014);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
