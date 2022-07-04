#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_3695708287462369075);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5497721190925856545);
void car_H_mod_fun(double *state, double *out_3741989862932459432);
void car_f_fun(double *state, double dt, double *out_5295389726812860391);
void car_F_fun(double *state, double dt, double *out_6749094151876649186);
void car_h_25(double *state, double *unused, double *out_5488032525607789861);
void car_H_25(double *state, double *unused, double *out_4647198212667562955);
void car_h_24(double *state, double *unused, double *out_3590326092945540893);
void car_H_24(double *state, double *unused, double *out_1669240464361133365);
void car_h_30(double *state, double *unused, double *out_5077438897347712570);
void car_H_30(double *state, double *unused, double *out_4185646950092805381);
void car_h_26(double *state, double *unused, double *out_5719960003050504704);
void car_H_26(double *state, double *unused, double *out_1057394942271646318);
void car_h_27(double *state, double *unused, double *out_7957038280440916110);
void car_H_27(double *state, double *unused, double *out_2898064962256180069);
void car_h_29(double *state, double *unused, double *out_4963648356717612393);
void car_H_29(double *state, double *unused, double *out_361848180963973428);
void car_h_28(double *state, double *unused, double *out_3874669749310933570);
void car_H_28(double *state, double *unused, double *out_1653765388214965162);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}