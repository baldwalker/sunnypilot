#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_3545767590363888806);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6258502841145403309);
void pose_H_mod_fun(double *state, double *out_8724642416341409662);
void pose_f_fun(double *state, double dt, double *out_3607734036645673030);
void pose_F_fun(double *state, double dt, double *out_2129463630195495818);
void pose_h_4(double *state, double *unused, double *out_6529939596965481659);
void pose_H_4(double *state, double *unused, double *out_8967940655313429347);
void pose_h_10(double *state, double *unused, double *out_4206098625503837771);
void pose_H_10(double *state, double *unused, double *out_5859467736028250385);
void pose_h_13(double *state, double *unused, double *out_948693449030853671);
void pose_H_13(double *state, double *unused, double *out_1357309446996728418);
void pose_h_14(double *state, double *unused, double *out_3564627351806134308);
void pose_H_14(double *state, double *unused, double *out_5004699798973944818);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}