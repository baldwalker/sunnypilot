#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_3068681635687019270);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6108564159138725733);
void pose_H_mod_fun(double *state, double *out_8310243639589261475);
void pose_f_fun(double *state, double dt, double *out_7527557940114523882);
void pose_F_fun(double *state, double dt, double *out_8348964164843467013);
void pose_h_4(double *state, double *unused, double *out_5762446430048339267);
void pose_H_4(double *state, double *unused, double *out_5036021560798312222);
void pose_h_10(double *state, double *unused, double *out_3165249718128103071);
void pose_H_10(double *state, double *unused, double *out_697369030236560030);
void pose_h_13(double *state, double *unused, double *out_2411777720830163540);
void pose_H_13(double *state, double *unused, double *out_8248295386130645023);
void pose_h_14(double *state, double *unused, double *out_5831462969446676566);
void pose_H_14(double *state, double *unused, double *out_8999262417137796751);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}