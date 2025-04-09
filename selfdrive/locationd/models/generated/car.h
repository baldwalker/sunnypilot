#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_1628263207268497143);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3105845108139388897);
void car_H_mod_fun(double *state, double *out_4975681247376949868);
void car_f_fun(double *state, double dt, double *out_1358740456813391901);
void car_F_fun(double *state, double dt, double *out_6483424864726333523);
void car_h_25(double *state, double *unused, double *out_31592248190272271);
void car_H_25(double *state, double *unused, double *out_4277874600420364511);
void car_h_24(double *state, double *unused, double *out_4691068814801759937);
void car_H_24(double *state, double *unused, double *out_2105225001414864945);
void car_h_30(double *state, double *unused, double *out_6619420174666567434);
void car_H_30(double *state, double *unused, double *out_7252179131797570350);
void car_h_26(double *state, double *unused, double *out_1617746907005721789);
void car_H_26(double *state, double *unused, double *out_536371281546308287);
void car_h_27(double *state, double *unused, double *out_6746936507428314206);
void car_H_27(double *state, double *unused, double *out_9019801630111556355);
void car_h_29(double *state, double *unused, double *out_6825650631382138537);
void car_H_29(double *state, double *unused, double *out_7306438903242005322);
void car_h_28(double *state, double *unused, double *out_6737427568599944178);
void car_H_28(double *state, double *unused, double *out_2224039886172474748);
void car_h_31(double *state, double *unused, double *out_3605857280063750079);
void car_H_31(double *state, double *unused, double *out_4308520562297324939);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}