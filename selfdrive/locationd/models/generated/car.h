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
void car_err_fun(double *nom_x, double *delta_x, double *out_7717500870753027025);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5756825933590615713);
void car_H_mod_fun(double *state, double *out_8047934900049386876);
void car_f_fun(double *state, double dt, double *out_1531082340704195775);
void car_F_fun(double *state, double dt, double *out_5639704107982325358);
void car_h_25(double *state, double *unused, double *out_6733206396346686402);
void car_H_25(double *state, double *unused, double *out_1835251734303790686);
void car_h_24(double *state, double *unused, double *out_6326469331191467479);
void car_H_24(double *state, double *unused, double *out_337397864701708880);
void car_h_30(double *state, double *unused, double *out_7008400458631192291);
void car_H_30(double *state, double *unused, double *out_4353584692811039313);
void car_h_26(double *state, double *unused, double *out_2006727190970346646);
void car_H_26(double *state, double *unused, double *out_1906251584570265538);
void car_h_27(double *state, double *unused, double *out_5042758561611005442);
void car_H_27(double *state, double *unused, double *out_2178821381010614402);
void car_h_29(double *state, double *unused, double *out_6212436399243034378);
void car_H_29(double *state, double *unused, double *out_4863816037125431497);
void car_h_28(double *state, double *unused, double *out_8402312244437700529);
void car_H_28(double *state, double *unused, double *out_218582979944099077);
void car_h_31(double *state, double *unused, double *out_3216876996099125222);
void car_H_31(double *state, double *unused, double *out_2532459686803617014);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}