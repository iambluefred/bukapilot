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
void car_err_fun(double *nom_x, double *delta_x, double *out_8977861693128698216);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6981307297054337874);
void car_H_mod_fun(double *state, double *out_609338320281026580);
void car_f_fun(double *state, double dt, double *out_5067090323671698306);
void car_F_fun(double *state, double dt, double *out_8920164968435431145);
void car_h_25(double *state, double *unused, double *out_8518928414088315027);
void car_H_25(double *state, double *unused, double *out_4448681048656224898);
void car_h_24(double *state, double *unused, double *out_6286710017891584073);
void car_H_24(double *state, double *unused, double *out_2222973264677356336);
void car_h_30(double *state, double *unused, double *out_8243734351803809138);
void car_H_30(double *state, double *unused, double *out_4578019995799464968);
void car_h_26(double *state, double *unused, double *out_2758650320462976683);
void car_H_26(double *state, double *unused, double *out_8190184367530281122);
void car_h_27(double *state, double *unused, double *out_2640642926529599939);
void car_H_27(double *state, double *unused, double *out_6752783307599889879);
void car_h_29(double *state, double *unused, double *out_7309440465916211685);
void car_H_29(double *state, double *unused, double *out_4067788651485072784);
void car_h_28(double *state, double *unused, double *out_2361030341131245706);
void car_H_28(double *state, double *unused, double *out_6502515762904114661);
void car_h_31(double *state, double *unused, double *out_120949062630226857);
void car_H_31(double *state, double *unused, double *out_4418035086779264470);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}