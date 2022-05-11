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
void car_err_fun(double *nom_x, double *delta_x, double *out_2080459397635649486);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7549402271710703544);
void car_H_mod_fun(double *state, double *out_7998804296790963776);
void car_f_fun(double *state, double dt, double *out_1007093853574867969);
void car_F_fun(double *state, double dt, double *out_3972400413518286592);
void car_h_25(double *state, double *unused, double *out_6186081259470436288);
void car_H_25(double *state, double *unused, double *out_7277562395580263808);
void car_h_24(double *state, double *unused, double *out_2163392311181086579);
void car_H_24(double *state, double *unused, double *out_7159240284580830777);
void car_h_30(double *state, double *unused, double *out_6135121145360409701);
void car_H_30(double *state, double *unused, double *out_2336336515368919472);
void car_h_26(double *state, double *unused, double *out_3530715824551712050);
void car_H_26(double *state, double *unused, double *out_8583798167535104953);
void car_h_27(double *state, double *unused, double *out_307795771911721200);
void car_H_27(double *state, double *unused, double *out_3623918503205544784);
void car_h_29(double *state, double *unused, double *out_8947673248630263084);
void car_H_29(double *state, double *unused, double *out_6160135284497751425);
void car_h_28(double *state, double *unused, double *out_691852136549595211);
void car_H_28(double *state, double *unused, double *out_4269671310892225973);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}