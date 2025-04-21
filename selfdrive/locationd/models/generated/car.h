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
void car_err_fun(double *nom_x, double *delta_x, double *out_6381148626922471769);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8278583823519988565);
void car_H_mod_fun(double *state, double *out_2438603045394859286);
void car_f_fun(double *state, double dt, double *out_7546549020743744747);
void car_F_fun(double *state, double dt, double *out_2986487306094983042);
void car_h_25(double *state, double *unused, double *out_8162556459827044702);
void car_H_25(double *state, double *unused, double *out_2465088779789805554);
void car_h_24(double *state, double *unused, double *out_6375974745677310448);
void car_H_24(double *state, double *unused, double *out_4642303203396955527);
void car_h_30(double *state, double *unused, double *out_4064253161547672838);
void car_H_30(double *state, double *unused, double *out_4983421738297054181);
void car_h_26(double *state, double *unused, double *out_6957822417537203631);
void car_H_26(double *state, double *unused, double *out_1276414539084250670);
void car_h_27(double *state, double *unused, double *out_5282514346221661269);
void car_H_27(double *state, double *unused, double *out_7207015809480997398);
void car_h_29(double *state, double *unused, double *out_1420830869793159617);
void car_H_29(double *state, double *unused, double *out_5493653082611446365);
void car_h_28(double *state, double *unused, double *out_4300430252886363157);
void car_H_28(double *state, double *unused, double *out_411254065541915791);
void car_h_31(double *state, double *unused, double *out_7887362397542538813);
void car_H_31(double *state, double *unused, double *out_1902622641317602146);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}