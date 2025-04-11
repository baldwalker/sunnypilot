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
void car_err_fun(double *nom_x, double *delta_x, double *out_8098387024111253733);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8197590140558714354);
void car_H_mod_fun(double *state, double *out_7985016218246684398);
void car_f_fun(double *state, double dt, double *out_2546879206310887771);
void car_F_fun(double *state, double dt, double *out_6865742471245780773);
void car_h_25(double *state, double *unused, double *out_409840628699075303);
void car_H_25(double *state, double *unused, double *out_6938747205473375541);
void car_h_24(double *state, double *unused, double *out_8835857786613779772);
void car_H_24(double *state, double *unused, double *out_9111396804478875107);
void car_h_30(double *state, double *unused, double *out_8173105885519747744);
void car_H_30(double *state, double *unused, double *out_4420414246966126914);
void car_h_26(double *state, double *unused, double *out_5568700564711050093);
void car_H_26(double *state, double *unused, double *out_7766493549362119851);
void car_h_27(double *state, double *unused, double *out_9054934273003635548);
void car_H_27(double *state, double *unused, double *out_2196820175782183697);
void car_h_29(double *state, double *unused, double *out_6822852019009251702);
void car_H_29(double *state, double *unused, double *out_3910182902651734730);
void car_h_28(double *state, double *unused, double *out_8788493916029438551);
void car_H_28(double *state, double *unused, double *out_8992581919721265304);
void car_h_31(double *state, double *unused, double *out_191577528914288251);
void car_H_31(double *state, double *unused, double *out_6908101243596415113);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}