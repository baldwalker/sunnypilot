#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_2258393448586014737);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_150977374342042324);
void pose_H_mod_fun(double *state, double *out_3607427627586762493);
void pose_f_fun(double *state, double dt, double *out_961891625828540708);
void pose_F_fun(double *state, double dt, double *out_3085087175776285280);
void pose_h_4(double *state, double *unused, double *out_2716213592127562262);
void pose_H_4(double *state, double *unused, double *out_1545090757452318242);
void pose_h_10(double *state, double *unused, double *out_4417817159878647707);
void pose_H_10(double *state, double *unused, double *out_6890704536492367764);
void pose_h_13(double *state, double *unused, double *out_8627506231444867801);
void pose_H_13(double *state, double *unused, double *out_4757364582784651043);
void pose_h_14(double *state, double *unused, double *out_44121701262927063);
void pose_H_14(double *state, double *unused, double *out_5508331613791802771);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}