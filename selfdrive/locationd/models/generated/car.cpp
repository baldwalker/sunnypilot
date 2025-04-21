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
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6381148626922471769) {
   out_6381148626922471769[0] = delta_x[0] + nom_x[0];
   out_6381148626922471769[1] = delta_x[1] + nom_x[1];
   out_6381148626922471769[2] = delta_x[2] + nom_x[2];
   out_6381148626922471769[3] = delta_x[3] + nom_x[3];
   out_6381148626922471769[4] = delta_x[4] + nom_x[4];
   out_6381148626922471769[5] = delta_x[5] + nom_x[5];
   out_6381148626922471769[6] = delta_x[6] + nom_x[6];
   out_6381148626922471769[7] = delta_x[7] + nom_x[7];
   out_6381148626922471769[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8278583823519988565) {
   out_8278583823519988565[0] = -nom_x[0] + true_x[0];
   out_8278583823519988565[1] = -nom_x[1] + true_x[1];
   out_8278583823519988565[2] = -nom_x[2] + true_x[2];
   out_8278583823519988565[3] = -nom_x[3] + true_x[3];
   out_8278583823519988565[4] = -nom_x[4] + true_x[4];
   out_8278583823519988565[5] = -nom_x[5] + true_x[5];
   out_8278583823519988565[6] = -nom_x[6] + true_x[6];
   out_8278583823519988565[7] = -nom_x[7] + true_x[7];
   out_8278583823519988565[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2438603045394859286) {
   out_2438603045394859286[0] = 1.0;
   out_2438603045394859286[1] = 0.0;
   out_2438603045394859286[2] = 0.0;
   out_2438603045394859286[3] = 0.0;
   out_2438603045394859286[4] = 0.0;
   out_2438603045394859286[5] = 0.0;
   out_2438603045394859286[6] = 0.0;
   out_2438603045394859286[7] = 0.0;
   out_2438603045394859286[8] = 0.0;
   out_2438603045394859286[9] = 0.0;
   out_2438603045394859286[10] = 1.0;
   out_2438603045394859286[11] = 0.0;
   out_2438603045394859286[12] = 0.0;
   out_2438603045394859286[13] = 0.0;
   out_2438603045394859286[14] = 0.0;
   out_2438603045394859286[15] = 0.0;
   out_2438603045394859286[16] = 0.0;
   out_2438603045394859286[17] = 0.0;
   out_2438603045394859286[18] = 0.0;
   out_2438603045394859286[19] = 0.0;
   out_2438603045394859286[20] = 1.0;
   out_2438603045394859286[21] = 0.0;
   out_2438603045394859286[22] = 0.0;
   out_2438603045394859286[23] = 0.0;
   out_2438603045394859286[24] = 0.0;
   out_2438603045394859286[25] = 0.0;
   out_2438603045394859286[26] = 0.0;
   out_2438603045394859286[27] = 0.0;
   out_2438603045394859286[28] = 0.0;
   out_2438603045394859286[29] = 0.0;
   out_2438603045394859286[30] = 1.0;
   out_2438603045394859286[31] = 0.0;
   out_2438603045394859286[32] = 0.0;
   out_2438603045394859286[33] = 0.0;
   out_2438603045394859286[34] = 0.0;
   out_2438603045394859286[35] = 0.0;
   out_2438603045394859286[36] = 0.0;
   out_2438603045394859286[37] = 0.0;
   out_2438603045394859286[38] = 0.0;
   out_2438603045394859286[39] = 0.0;
   out_2438603045394859286[40] = 1.0;
   out_2438603045394859286[41] = 0.0;
   out_2438603045394859286[42] = 0.0;
   out_2438603045394859286[43] = 0.0;
   out_2438603045394859286[44] = 0.0;
   out_2438603045394859286[45] = 0.0;
   out_2438603045394859286[46] = 0.0;
   out_2438603045394859286[47] = 0.0;
   out_2438603045394859286[48] = 0.0;
   out_2438603045394859286[49] = 0.0;
   out_2438603045394859286[50] = 1.0;
   out_2438603045394859286[51] = 0.0;
   out_2438603045394859286[52] = 0.0;
   out_2438603045394859286[53] = 0.0;
   out_2438603045394859286[54] = 0.0;
   out_2438603045394859286[55] = 0.0;
   out_2438603045394859286[56] = 0.0;
   out_2438603045394859286[57] = 0.0;
   out_2438603045394859286[58] = 0.0;
   out_2438603045394859286[59] = 0.0;
   out_2438603045394859286[60] = 1.0;
   out_2438603045394859286[61] = 0.0;
   out_2438603045394859286[62] = 0.0;
   out_2438603045394859286[63] = 0.0;
   out_2438603045394859286[64] = 0.0;
   out_2438603045394859286[65] = 0.0;
   out_2438603045394859286[66] = 0.0;
   out_2438603045394859286[67] = 0.0;
   out_2438603045394859286[68] = 0.0;
   out_2438603045394859286[69] = 0.0;
   out_2438603045394859286[70] = 1.0;
   out_2438603045394859286[71] = 0.0;
   out_2438603045394859286[72] = 0.0;
   out_2438603045394859286[73] = 0.0;
   out_2438603045394859286[74] = 0.0;
   out_2438603045394859286[75] = 0.0;
   out_2438603045394859286[76] = 0.0;
   out_2438603045394859286[77] = 0.0;
   out_2438603045394859286[78] = 0.0;
   out_2438603045394859286[79] = 0.0;
   out_2438603045394859286[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7546549020743744747) {
   out_7546549020743744747[0] = state[0];
   out_7546549020743744747[1] = state[1];
   out_7546549020743744747[2] = state[2];
   out_7546549020743744747[3] = state[3];
   out_7546549020743744747[4] = state[4];
   out_7546549020743744747[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7546549020743744747[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7546549020743744747[7] = state[7];
   out_7546549020743744747[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2986487306094983042) {
   out_2986487306094983042[0] = 1;
   out_2986487306094983042[1] = 0;
   out_2986487306094983042[2] = 0;
   out_2986487306094983042[3] = 0;
   out_2986487306094983042[4] = 0;
   out_2986487306094983042[5] = 0;
   out_2986487306094983042[6] = 0;
   out_2986487306094983042[7] = 0;
   out_2986487306094983042[8] = 0;
   out_2986487306094983042[9] = 0;
   out_2986487306094983042[10] = 1;
   out_2986487306094983042[11] = 0;
   out_2986487306094983042[12] = 0;
   out_2986487306094983042[13] = 0;
   out_2986487306094983042[14] = 0;
   out_2986487306094983042[15] = 0;
   out_2986487306094983042[16] = 0;
   out_2986487306094983042[17] = 0;
   out_2986487306094983042[18] = 0;
   out_2986487306094983042[19] = 0;
   out_2986487306094983042[20] = 1;
   out_2986487306094983042[21] = 0;
   out_2986487306094983042[22] = 0;
   out_2986487306094983042[23] = 0;
   out_2986487306094983042[24] = 0;
   out_2986487306094983042[25] = 0;
   out_2986487306094983042[26] = 0;
   out_2986487306094983042[27] = 0;
   out_2986487306094983042[28] = 0;
   out_2986487306094983042[29] = 0;
   out_2986487306094983042[30] = 1;
   out_2986487306094983042[31] = 0;
   out_2986487306094983042[32] = 0;
   out_2986487306094983042[33] = 0;
   out_2986487306094983042[34] = 0;
   out_2986487306094983042[35] = 0;
   out_2986487306094983042[36] = 0;
   out_2986487306094983042[37] = 0;
   out_2986487306094983042[38] = 0;
   out_2986487306094983042[39] = 0;
   out_2986487306094983042[40] = 1;
   out_2986487306094983042[41] = 0;
   out_2986487306094983042[42] = 0;
   out_2986487306094983042[43] = 0;
   out_2986487306094983042[44] = 0;
   out_2986487306094983042[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2986487306094983042[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2986487306094983042[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2986487306094983042[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2986487306094983042[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2986487306094983042[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2986487306094983042[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2986487306094983042[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2986487306094983042[53] = -9.8000000000000007*dt;
   out_2986487306094983042[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2986487306094983042[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2986487306094983042[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2986487306094983042[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2986487306094983042[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2986487306094983042[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2986487306094983042[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2986487306094983042[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2986487306094983042[62] = 0;
   out_2986487306094983042[63] = 0;
   out_2986487306094983042[64] = 0;
   out_2986487306094983042[65] = 0;
   out_2986487306094983042[66] = 0;
   out_2986487306094983042[67] = 0;
   out_2986487306094983042[68] = 0;
   out_2986487306094983042[69] = 0;
   out_2986487306094983042[70] = 1;
   out_2986487306094983042[71] = 0;
   out_2986487306094983042[72] = 0;
   out_2986487306094983042[73] = 0;
   out_2986487306094983042[74] = 0;
   out_2986487306094983042[75] = 0;
   out_2986487306094983042[76] = 0;
   out_2986487306094983042[77] = 0;
   out_2986487306094983042[78] = 0;
   out_2986487306094983042[79] = 0;
   out_2986487306094983042[80] = 1;
}
void h_25(double *state, double *unused, double *out_8162556459827044702) {
   out_8162556459827044702[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2465088779789805554) {
   out_2465088779789805554[0] = 0;
   out_2465088779789805554[1] = 0;
   out_2465088779789805554[2] = 0;
   out_2465088779789805554[3] = 0;
   out_2465088779789805554[4] = 0;
   out_2465088779789805554[5] = 0;
   out_2465088779789805554[6] = 1;
   out_2465088779789805554[7] = 0;
   out_2465088779789805554[8] = 0;
}
void h_24(double *state, double *unused, double *out_6375974745677310448) {
   out_6375974745677310448[0] = state[4];
   out_6375974745677310448[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4642303203396955527) {
   out_4642303203396955527[0] = 0;
   out_4642303203396955527[1] = 0;
   out_4642303203396955527[2] = 0;
   out_4642303203396955527[3] = 0;
   out_4642303203396955527[4] = 1;
   out_4642303203396955527[5] = 0;
   out_4642303203396955527[6] = 0;
   out_4642303203396955527[7] = 0;
   out_4642303203396955527[8] = 0;
   out_4642303203396955527[9] = 0;
   out_4642303203396955527[10] = 0;
   out_4642303203396955527[11] = 0;
   out_4642303203396955527[12] = 0;
   out_4642303203396955527[13] = 0;
   out_4642303203396955527[14] = 1;
   out_4642303203396955527[15] = 0;
   out_4642303203396955527[16] = 0;
   out_4642303203396955527[17] = 0;
}
void h_30(double *state, double *unused, double *out_4064253161547672838) {
   out_4064253161547672838[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4983421738297054181) {
   out_4983421738297054181[0] = 0;
   out_4983421738297054181[1] = 0;
   out_4983421738297054181[2] = 0;
   out_4983421738297054181[3] = 0;
   out_4983421738297054181[4] = 1;
   out_4983421738297054181[5] = 0;
   out_4983421738297054181[6] = 0;
   out_4983421738297054181[7] = 0;
   out_4983421738297054181[8] = 0;
}
void h_26(double *state, double *unused, double *out_6957822417537203631) {
   out_6957822417537203631[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1276414539084250670) {
   out_1276414539084250670[0] = 0;
   out_1276414539084250670[1] = 0;
   out_1276414539084250670[2] = 0;
   out_1276414539084250670[3] = 0;
   out_1276414539084250670[4] = 0;
   out_1276414539084250670[5] = 0;
   out_1276414539084250670[6] = 0;
   out_1276414539084250670[7] = 1;
   out_1276414539084250670[8] = 0;
}
void h_27(double *state, double *unused, double *out_5282514346221661269) {
   out_5282514346221661269[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7207015809480997398) {
   out_7207015809480997398[0] = 0;
   out_7207015809480997398[1] = 0;
   out_7207015809480997398[2] = 0;
   out_7207015809480997398[3] = 1;
   out_7207015809480997398[4] = 0;
   out_7207015809480997398[5] = 0;
   out_7207015809480997398[6] = 0;
   out_7207015809480997398[7] = 0;
   out_7207015809480997398[8] = 0;
}
void h_29(double *state, double *unused, double *out_1420830869793159617) {
   out_1420830869793159617[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5493653082611446365) {
   out_5493653082611446365[0] = 0;
   out_5493653082611446365[1] = 1;
   out_5493653082611446365[2] = 0;
   out_5493653082611446365[3] = 0;
   out_5493653082611446365[4] = 0;
   out_5493653082611446365[5] = 0;
   out_5493653082611446365[6] = 0;
   out_5493653082611446365[7] = 0;
   out_5493653082611446365[8] = 0;
}
void h_28(double *state, double *unused, double *out_4300430252886363157) {
   out_4300430252886363157[0] = state[0];
}
void H_28(double *state, double *unused, double *out_411254065541915791) {
   out_411254065541915791[0] = 1;
   out_411254065541915791[1] = 0;
   out_411254065541915791[2] = 0;
   out_411254065541915791[3] = 0;
   out_411254065541915791[4] = 0;
   out_411254065541915791[5] = 0;
   out_411254065541915791[6] = 0;
   out_411254065541915791[7] = 0;
   out_411254065541915791[8] = 0;
}
void h_31(double *state, double *unused, double *out_7887362397542538813) {
   out_7887362397542538813[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1902622641317602146) {
   out_1902622641317602146[0] = 0;
   out_1902622641317602146[1] = 0;
   out_1902622641317602146[2] = 0;
   out_1902622641317602146[3] = 0;
   out_1902622641317602146[4] = 0;
   out_1902622641317602146[5] = 0;
   out_1902622641317602146[6] = 0;
   out_1902622641317602146[7] = 0;
   out_1902622641317602146[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6381148626922471769) {
  err_fun(nom_x, delta_x, out_6381148626922471769);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8278583823519988565) {
  inv_err_fun(nom_x, true_x, out_8278583823519988565);
}
void car_H_mod_fun(double *state, double *out_2438603045394859286) {
  H_mod_fun(state, out_2438603045394859286);
}
void car_f_fun(double *state, double dt, double *out_7546549020743744747) {
  f_fun(state,  dt, out_7546549020743744747);
}
void car_F_fun(double *state, double dt, double *out_2986487306094983042) {
  F_fun(state,  dt, out_2986487306094983042);
}
void car_h_25(double *state, double *unused, double *out_8162556459827044702) {
  h_25(state, unused, out_8162556459827044702);
}
void car_H_25(double *state, double *unused, double *out_2465088779789805554) {
  H_25(state, unused, out_2465088779789805554);
}
void car_h_24(double *state, double *unused, double *out_6375974745677310448) {
  h_24(state, unused, out_6375974745677310448);
}
void car_H_24(double *state, double *unused, double *out_4642303203396955527) {
  H_24(state, unused, out_4642303203396955527);
}
void car_h_30(double *state, double *unused, double *out_4064253161547672838) {
  h_30(state, unused, out_4064253161547672838);
}
void car_H_30(double *state, double *unused, double *out_4983421738297054181) {
  H_30(state, unused, out_4983421738297054181);
}
void car_h_26(double *state, double *unused, double *out_6957822417537203631) {
  h_26(state, unused, out_6957822417537203631);
}
void car_H_26(double *state, double *unused, double *out_1276414539084250670) {
  H_26(state, unused, out_1276414539084250670);
}
void car_h_27(double *state, double *unused, double *out_5282514346221661269) {
  h_27(state, unused, out_5282514346221661269);
}
void car_H_27(double *state, double *unused, double *out_7207015809480997398) {
  H_27(state, unused, out_7207015809480997398);
}
void car_h_29(double *state, double *unused, double *out_1420830869793159617) {
  h_29(state, unused, out_1420830869793159617);
}
void car_H_29(double *state, double *unused, double *out_5493653082611446365) {
  H_29(state, unused, out_5493653082611446365);
}
void car_h_28(double *state, double *unused, double *out_4300430252886363157) {
  h_28(state, unused, out_4300430252886363157);
}
void car_H_28(double *state, double *unused, double *out_411254065541915791) {
  H_28(state, unused, out_411254065541915791);
}
void car_h_31(double *state, double *unused, double *out_7887362397542538813) {
  h_31(state, unused, out_7887362397542538813);
}
void car_H_31(double *state, double *unused, double *out_1902622641317602146) {
  H_31(state, unused, out_1902622641317602146);
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
