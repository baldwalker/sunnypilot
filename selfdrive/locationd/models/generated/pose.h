#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_7863920857934757474);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1293085161672310543);
void pose_H_mod_fun(double *state, double *out_271066960830845225);
void pose_f_fun(double *state, double dt, double *out_3845547857628505078);
void pose_F_fun(double *state, double dt, double *out_1631823746137781300);
void pose_h_4(double *state, double *unused, double *out_4955052215047598515);
void pose_H_4(double *state, double *unused, double *out_4190822113574178604);
void pose_h_10(double *state, double *unused, double *out_4337291095685463138);
void pose_H_10(double *state, double *unused, double *out_1798816701295712907);
void pose_h_13(double *state, double *unused, double *out_2119852544367272368);
void pose_H_13(double *state, double *unused, double *out_978548288241845803);
void pose_h_14(double *state, double *unused, double *out_178331072993193741);
void pose_H_14(double *state, double *unused, double *out_227581257234694075);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}