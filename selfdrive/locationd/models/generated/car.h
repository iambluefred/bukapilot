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
void car_err_fun(double *nom_x, double *delta_x, double *out_575770558294804495);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5727502109067883687);
void car_H_mod_fun(double *state, double *out_6921934919768022397);
void car_f_fun(double *state, double dt, double *out_247457335877544533);
void car_F_fun(double *state, double dt, double *out_8800351135451747642);
void car_h_25(double *state, double *unused, double *out_1962729859178462780);
void car_H_25(double *state, double *unused, double *out_5440771516178847649);
void car_h_24(double *state, double *unused, double *out_4975792545761529740);
void car_H_24(double *state, double *unused, double *out_5890114518268132700);
void car_h_30(double *state, double *unused, double *out_6061033157457834644);
void car_H_30(double *state, double *unused, double *out_3653955642053336138);
void car_h_26(double *state, double *unused, double *out_3589401451985719832);
void car_H_26(double *state, double *unused, double *out_6782207649874495201);
void car_h_27(double *state, double *unused, double *out_4999515910622795465);
void car_H_27(double *state, double *unused, double *out_4941537629889961450);
void car_h_29(double *state, double *unused, double *out_1059359889796988999);
void car_H_29(double *state, double *unused, double *out_3966632260437056862);
void car_h_28(double *state, double *unused, double *out_3984807951886966422);
void car_H_28(double *state, double *unused, double *out_5538797077204924050);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}