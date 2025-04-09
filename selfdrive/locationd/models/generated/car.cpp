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
void err_fun(double *nom_x, double *delta_x, double *out_1628263207268497143) {
   out_1628263207268497143[0] = delta_x[0] + nom_x[0];
   out_1628263207268497143[1] = delta_x[1] + nom_x[1];
   out_1628263207268497143[2] = delta_x[2] + nom_x[2];
   out_1628263207268497143[3] = delta_x[3] + nom_x[3];
   out_1628263207268497143[4] = delta_x[4] + nom_x[4];
   out_1628263207268497143[5] = delta_x[5] + nom_x[5];
   out_1628263207268497143[6] = delta_x[6] + nom_x[6];
   out_1628263207268497143[7] = delta_x[7] + nom_x[7];
   out_1628263207268497143[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3105845108139388897) {
   out_3105845108139388897[0] = -nom_x[0] + true_x[0];
   out_3105845108139388897[1] = -nom_x[1] + true_x[1];
   out_3105845108139388897[2] = -nom_x[2] + true_x[2];
   out_3105845108139388897[3] = -nom_x[3] + true_x[3];
   out_3105845108139388897[4] = -nom_x[4] + true_x[4];
   out_3105845108139388897[5] = -nom_x[5] + true_x[5];
   out_3105845108139388897[6] = -nom_x[6] + true_x[6];
   out_3105845108139388897[7] = -nom_x[7] + true_x[7];
   out_3105845108139388897[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4975681247376949868) {
   out_4975681247376949868[0] = 1.0;
   out_4975681247376949868[1] = 0.0;
   out_4975681247376949868[2] = 0.0;
   out_4975681247376949868[3] = 0.0;
   out_4975681247376949868[4] = 0.0;
   out_4975681247376949868[5] = 0.0;
   out_4975681247376949868[6] = 0.0;
   out_4975681247376949868[7] = 0.0;
   out_4975681247376949868[8] = 0.0;
   out_4975681247376949868[9] = 0.0;
   out_4975681247376949868[10] = 1.0;
   out_4975681247376949868[11] = 0.0;
   out_4975681247376949868[12] = 0.0;
   out_4975681247376949868[13] = 0.0;
   out_4975681247376949868[14] = 0.0;
   out_4975681247376949868[15] = 0.0;
   out_4975681247376949868[16] = 0.0;
   out_4975681247376949868[17] = 0.0;
   out_4975681247376949868[18] = 0.0;
   out_4975681247376949868[19] = 0.0;
   out_4975681247376949868[20] = 1.0;
   out_4975681247376949868[21] = 0.0;
   out_4975681247376949868[22] = 0.0;
   out_4975681247376949868[23] = 0.0;
   out_4975681247376949868[24] = 0.0;
   out_4975681247376949868[25] = 0.0;
   out_4975681247376949868[26] = 0.0;
   out_4975681247376949868[27] = 0.0;
   out_4975681247376949868[28] = 0.0;
   out_4975681247376949868[29] = 0.0;
   out_4975681247376949868[30] = 1.0;
   out_4975681247376949868[31] = 0.0;
   out_4975681247376949868[32] = 0.0;
   out_4975681247376949868[33] = 0.0;
   out_4975681247376949868[34] = 0.0;
   out_4975681247376949868[35] = 0.0;
   out_4975681247376949868[36] = 0.0;
   out_4975681247376949868[37] = 0.0;
   out_4975681247376949868[38] = 0.0;
   out_4975681247376949868[39] = 0.0;
   out_4975681247376949868[40] = 1.0;
   out_4975681247376949868[41] = 0.0;
   out_4975681247376949868[42] = 0.0;
   out_4975681247376949868[43] = 0.0;
   out_4975681247376949868[44] = 0.0;
   out_4975681247376949868[45] = 0.0;
   out_4975681247376949868[46] = 0.0;
   out_4975681247376949868[47] = 0.0;
   out_4975681247376949868[48] = 0.0;
   out_4975681247376949868[49] = 0.0;
   out_4975681247376949868[50] = 1.0;
   out_4975681247376949868[51] = 0.0;
   out_4975681247376949868[52] = 0.0;
   out_4975681247376949868[53] = 0.0;
   out_4975681247376949868[54] = 0.0;
   out_4975681247376949868[55] = 0.0;
   out_4975681247376949868[56] = 0.0;
   out_4975681247376949868[57] = 0.0;
   out_4975681247376949868[58] = 0.0;
   out_4975681247376949868[59] = 0.0;
   out_4975681247376949868[60] = 1.0;
   out_4975681247376949868[61] = 0.0;
   out_4975681247376949868[62] = 0.0;
   out_4975681247376949868[63] = 0.0;
   out_4975681247376949868[64] = 0.0;
   out_4975681247376949868[65] = 0.0;
   out_4975681247376949868[66] = 0.0;
   out_4975681247376949868[67] = 0.0;
   out_4975681247376949868[68] = 0.0;
   out_4975681247376949868[69] = 0.0;
   out_4975681247376949868[70] = 1.0;
   out_4975681247376949868[71] = 0.0;
   out_4975681247376949868[72] = 0.0;
   out_4975681247376949868[73] = 0.0;
   out_4975681247376949868[74] = 0.0;
   out_4975681247376949868[75] = 0.0;
   out_4975681247376949868[76] = 0.0;
   out_4975681247376949868[77] = 0.0;
   out_4975681247376949868[78] = 0.0;
   out_4975681247376949868[79] = 0.0;
   out_4975681247376949868[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1358740456813391901) {
   out_1358740456813391901[0] = state[0];
   out_1358740456813391901[1] = state[1];
   out_1358740456813391901[2] = state[2];
   out_1358740456813391901[3] = state[3];
   out_1358740456813391901[4] = state[4];
   out_1358740456813391901[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1358740456813391901[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1358740456813391901[7] = state[7];
   out_1358740456813391901[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6483424864726333523) {
   out_6483424864726333523[0] = 1;
   out_6483424864726333523[1] = 0;
   out_6483424864726333523[2] = 0;
   out_6483424864726333523[3] = 0;
   out_6483424864726333523[4] = 0;
   out_6483424864726333523[5] = 0;
   out_6483424864726333523[6] = 0;
   out_6483424864726333523[7] = 0;
   out_6483424864726333523[8] = 0;
   out_6483424864726333523[9] = 0;
   out_6483424864726333523[10] = 1;
   out_6483424864726333523[11] = 0;
   out_6483424864726333523[12] = 0;
   out_6483424864726333523[13] = 0;
   out_6483424864726333523[14] = 0;
   out_6483424864726333523[15] = 0;
   out_6483424864726333523[16] = 0;
   out_6483424864726333523[17] = 0;
   out_6483424864726333523[18] = 0;
   out_6483424864726333523[19] = 0;
   out_6483424864726333523[20] = 1;
   out_6483424864726333523[21] = 0;
   out_6483424864726333523[22] = 0;
   out_6483424864726333523[23] = 0;
   out_6483424864726333523[24] = 0;
   out_6483424864726333523[25] = 0;
   out_6483424864726333523[26] = 0;
   out_6483424864726333523[27] = 0;
   out_6483424864726333523[28] = 0;
   out_6483424864726333523[29] = 0;
   out_6483424864726333523[30] = 1;
   out_6483424864726333523[31] = 0;
   out_6483424864726333523[32] = 0;
   out_6483424864726333523[33] = 0;
   out_6483424864726333523[34] = 0;
   out_6483424864726333523[35] = 0;
   out_6483424864726333523[36] = 0;
   out_6483424864726333523[37] = 0;
   out_6483424864726333523[38] = 0;
   out_6483424864726333523[39] = 0;
   out_6483424864726333523[40] = 1;
   out_6483424864726333523[41] = 0;
   out_6483424864726333523[42] = 0;
   out_6483424864726333523[43] = 0;
   out_6483424864726333523[44] = 0;
   out_6483424864726333523[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6483424864726333523[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6483424864726333523[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6483424864726333523[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6483424864726333523[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6483424864726333523[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6483424864726333523[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6483424864726333523[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6483424864726333523[53] = -9.8000000000000007*dt;
   out_6483424864726333523[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6483424864726333523[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6483424864726333523[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6483424864726333523[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6483424864726333523[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6483424864726333523[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6483424864726333523[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6483424864726333523[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6483424864726333523[62] = 0;
   out_6483424864726333523[63] = 0;
   out_6483424864726333523[64] = 0;
   out_6483424864726333523[65] = 0;
   out_6483424864726333523[66] = 0;
   out_6483424864726333523[67] = 0;
   out_6483424864726333523[68] = 0;
   out_6483424864726333523[69] = 0;
   out_6483424864726333523[70] = 1;
   out_6483424864726333523[71] = 0;
   out_6483424864726333523[72] = 0;
   out_6483424864726333523[73] = 0;
   out_6483424864726333523[74] = 0;
   out_6483424864726333523[75] = 0;
   out_6483424864726333523[76] = 0;
   out_6483424864726333523[77] = 0;
   out_6483424864726333523[78] = 0;
   out_6483424864726333523[79] = 0;
   out_6483424864726333523[80] = 1;
}
void h_25(double *state, double *unused, double *out_31592248190272271) {
   out_31592248190272271[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4277874600420364511) {
   out_4277874600420364511[0] = 0;
   out_4277874600420364511[1] = 0;
   out_4277874600420364511[2] = 0;
   out_4277874600420364511[3] = 0;
   out_4277874600420364511[4] = 0;
   out_4277874600420364511[5] = 0;
   out_4277874600420364511[6] = 1;
   out_4277874600420364511[7] = 0;
   out_4277874600420364511[8] = 0;
}
void h_24(double *state, double *unused, double *out_4691068814801759937) {
   out_4691068814801759937[0] = state[4];
   out_4691068814801759937[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2105225001414864945) {
   out_2105225001414864945[0] = 0;
   out_2105225001414864945[1] = 0;
   out_2105225001414864945[2] = 0;
   out_2105225001414864945[3] = 0;
   out_2105225001414864945[4] = 1;
   out_2105225001414864945[5] = 0;
   out_2105225001414864945[6] = 0;
   out_2105225001414864945[7] = 0;
   out_2105225001414864945[8] = 0;
   out_2105225001414864945[9] = 0;
   out_2105225001414864945[10] = 0;
   out_2105225001414864945[11] = 0;
   out_2105225001414864945[12] = 0;
   out_2105225001414864945[13] = 0;
   out_2105225001414864945[14] = 1;
   out_2105225001414864945[15] = 0;
   out_2105225001414864945[16] = 0;
   out_2105225001414864945[17] = 0;
}
void h_30(double *state, double *unused, double *out_6619420174666567434) {
   out_6619420174666567434[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7252179131797570350) {
   out_7252179131797570350[0] = 0;
   out_7252179131797570350[1] = 0;
   out_7252179131797570350[2] = 0;
   out_7252179131797570350[3] = 0;
   out_7252179131797570350[4] = 1;
   out_7252179131797570350[5] = 0;
   out_7252179131797570350[6] = 0;
   out_7252179131797570350[7] = 0;
   out_7252179131797570350[8] = 0;
}
void h_26(double *state, double *unused, double *out_1617746907005721789) {
   out_1617746907005721789[0] = state[7];
}
void H_26(double *state, double *unused, double *out_536371281546308287) {
   out_536371281546308287[0] = 0;
   out_536371281546308287[1] = 0;
   out_536371281546308287[2] = 0;
   out_536371281546308287[3] = 0;
   out_536371281546308287[4] = 0;
   out_536371281546308287[5] = 0;
   out_536371281546308287[6] = 0;
   out_536371281546308287[7] = 1;
   out_536371281546308287[8] = 0;
}
void h_27(double *state, double *unused, double *out_6746936507428314206) {
   out_6746936507428314206[0] = state[3];
}
void H_27(double *state, double *unused, double *out_9019801630111556355) {
   out_9019801630111556355[0] = 0;
   out_9019801630111556355[1] = 0;
   out_9019801630111556355[2] = 0;
   out_9019801630111556355[3] = 1;
   out_9019801630111556355[4] = 0;
   out_9019801630111556355[5] = 0;
   out_9019801630111556355[6] = 0;
   out_9019801630111556355[7] = 0;
   out_9019801630111556355[8] = 0;
}
void h_29(double *state, double *unused, double *out_6825650631382138537) {
   out_6825650631382138537[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7306438903242005322) {
   out_7306438903242005322[0] = 0;
   out_7306438903242005322[1] = 1;
   out_7306438903242005322[2] = 0;
   out_7306438903242005322[3] = 0;
   out_7306438903242005322[4] = 0;
   out_7306438903242005322[5] = 0;
   out_7306438903242005322[6] = 0;
   out_7306438903242005322[7] = 0;
   out_7306438903242005322[8] = 0;
}
void h_28(double *state, double *unused, double *out_6737427568599944178) {
   out_6737427568599944178[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2224039886172474748) {
   out_2224039886172474748[0] = 1;
   out_2224039886172474748[1] = 0;
   out_2224039886172474748[2] = 0;
   out_2224039886172474748[3] = 0;
   out_2224039886172474748[4] = 0;
   out_2224039886172474748[5] = 0;
   out_2224039886172474748[6] = 0;
   out_2224039886172474748[7] = 0;
   out_2224039886172474748[8] = 0;
}
void h_31(double *state, double *unused, double *out_3605857280063750079) {
   out_3605857280063750079[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4308520562297324939) {
   out_4308520562297324939[0] = 0;
   out_4308520562297324939[1] = 0;
   out_4308520562297324939[2] = 0;
   out_4308520562297324939[3] = 0;
   out_4308520562297324939[4] = 0;
   out_4308520562297324939[5] = 0;
   out_4308520562297324939[6] = 0;
   out_4308520562297324939[7] = 0;
   out_4308520562297324939[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_1628263207268497143) {
  err_fun(nom_x, delta_x, out_1628263207268497143);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3105845108139388897) {
  inv_err_fun(nom_x, true_x, out_3105845108139388897);
}
void car_H_mod_fun(double *state, double *out_4975681247376949868) {
  H_mod_fun(state, out_4975681247376949868);
}
void car_f_fun(double *state, double dt, double *out_1358740456813391901) {
  f_fun(state,  dt, out_1358740456813391901);
}
void car_F_fun(double *state, double dt, double *out_6483424864726333523) {
  F_fun(state,  dt, out_6483424864726333523);
}
void car_h_25(double *state, double *unused, double *out_31592248190272271) {
  h_25(state, unused, out_31592248190272271);
}
void car_H_25(double *state, double *unused, double *out_4277874600420364511) {
  H_25(state, unused, out_4277874600420364511);
}
void car_h_24(double *state, double *unused, double *out_4691068814801759937) {
  h_24(state, unused, out_4691068814801759937);
}
void car_H_24(double *state, double *unused, double *out_2105225001414864945) {
  H_24(state, unused, out_2105225001414864945);
}
void car_h_30(double *state, double *unused, double *out_6619420174666567434) {
  h_30(state, unused, out_6619420174666567434);
}
void car_H_30(double *state, double *unused, double *out_7252179131797570350) {
  H_30(state, unused, out_7252179131797570350);
}
void car_h_26(double *state, double *unused, double *out_1617746907005721789) {
  h_26(state, unused, out_1617746907005721789);
}
void car_H_26(double *state, double *unused, double *out_536371281546308287) {
  H_26(state, unused, out_536371281546308287);
}
void car_h_27(double *state, double *unused, double *out_6746936507428314206) {
  h_27(state, unused, out_6746936507428314206);
}
void car_H_27(double *state, double *unused, double *out_9019801630111556355) {
  H_27(state, unused, out_9019801630111556355);
}
void car_h_29(double *state, double *unused, double *out_6825650631382138537) {
  h_29(state, unused, out_6825650631382138537);
}
void car_H_29(double *state, double *unused, double *out_7306438903242005322) {
  H_29(state, unused, out_7306438903242005322);
}
void car_h_28(double *state, double *unused, double *out_6737427568599944178) {
  h_28(state, unused, out_6737427568599944178);
}
void car_H_28(double *state, double *unused, double *out_2224039886172474748) {
  H_28(state, unused, out_2224039886172474748);
}
void car_h_31(double *state, double *unused, double *out_3605857280063750079) {
  h_31(state, unused, out_3605857280063750079);
}
void car_H_31(double *state, double *unused, double *out_4308520562297324939) {
  H_31(state, unused, out_4308520562297324939);
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
