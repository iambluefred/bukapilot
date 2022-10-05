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
void car_err_fun(double *nom_x, double *delta_x, double *out_8062980869779089220);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_508088850886154557);
void car_H_mod_fun(double *state, double *out_3427232086650733882);
void car_f_fun(double *state, double dt, double *out_2337273054858271848);
void car_F_fun(double *state, double dt, double *out_435201181327936292);
void car_h_25(double *state, double *unused, double *out_2760927816007907288);
void car_H_25(double *state, double *unused, double *out_8305718187882283659);
void car_h_24(double *state, double *unused, double *out_6700003583289376697);
void car_H_24(double *state, double *unused, double *out_9150714796665979058);
void car_h_30(double *state, double *unused, double *out_8914981031351293392);
void car_H_30(double *state, double *unused, double *out_3224335544335651202);
void car_h_26(double *state, double *unused, double *out_3324605254877637169);
void car_H_26(double *state, double *unused, double *out_4564214869008227435);
void car_h_27(double *state, double *unused, double *out_3206597860944260425);
void car_H_27(double *state, double *unused, double *out_5399098856136076113);
void car_h_29(double *state, double *unused, double *out_6102428928081440009);
void car_H_29(double *state, double *unused, double *out_2714104200021259018);
void car_h_28(double *state, double *unused, double *out_7488161702187920933);
void car_H_28(double *state, double *unused, double *out_7796503217090789592);
void car_h_31(double *state, double *unused, double *out_7671597222049239594);
void car_H_31(double *state, double *unused, double *out_8336364149759244087);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}