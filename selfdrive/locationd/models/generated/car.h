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
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_4279848910899479370);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4328374563733165579);
void car_H_mod_fun(double *state, double *out_3923237233234693522);
void car_f_fun(double *state, double dt, double *out_2379384011398423039);
void car_F_fun(double *state, double dt, double *out_3041465841500109542);
void car_h_25(double *state, double *unused, double *out_5343999116540519203);
void car_H_25(double *state, double *unused, double *out_8401926576797338066);
void car_h_24(double *state, double *unused, double *out_6923024055147605927);
void car_H_24(double *state, double *unused, double *out_6057929520912795245);
void car_h_30(double *state, double *unused, double *out_2419266140280153238);
void car_H_30(double *state, double *unused, double *out_3874230246669729868);
void car_h_26(double *state, double *unused, double *out_3979775377133695908);
void car_H_26(double *state, double *unused, double *out_4660423257923281842);
void car_h_27(double *state, double *unused, double *out_8036327438450689690);
void car_H_27(double *state, double *unused, double *out_1699466934869304957);
void car_h_29(double *state, double *unused, double *out_2202919889175955076);
void car_H_29(double *state, double *unused, double *out_4384461590984122052);
void car_h_28(double *state, double *unused, double *out_7806011314450164275);
void car_H_28(double *state, double *unused, double *out_697937426085408522);
void car_h_31(double *state, double *unused, double *out_5198479292319864232);
void car_H_31(double *state, double *unused, double *out_4034215155689930366);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}