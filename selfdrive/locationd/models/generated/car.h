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
void car_err_fun(double *nom_x, double *delta_x, double *out_5647470276735826301);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1373347932731707161);
void car_H_mod_fun(double *state, double *out_3198577004452224537);
void car_f_fun(double *state, double dt, double *out_2545790947865816023);
void car_F_fun(double *state, double dt, double *out_1912985182909371113);
void car_h_25(double *state, double *unused, double *out_4456422403562273665);
void car_H_25(double *state, double *unused, double *out_4929918955097233612);
void car_h_24(double *state, double *unused, double *out_248722503563105437);
void car_H_24(double *state, double *unused, double *out_7102568554102733178);
void car_h_30(double *state, double *unused, double *out_1907171574739542196);
void car_H_30(double *state, double *unused, double *out_1986771386394383143);
void car_h_26(double *state, double *unused, double *out_2490780506542086816);
void car_H_26(double *state, double *unused, double *out_8671422273971289836);
void car_h_27(double *state, double *unused, double *out_7734496948188230697);
void car_H_27(double *state, double *unused, double *out_7234021214040898593);
void car_h_29(double *state, double *unused, double *out_905380528530311187);
void car_H_29(double *state, double *unused, double *out_8947383940910449626);
void car_h_28(double *state, double *unused, double *out_5710573857860475274);
void car_H_28(double *state, double *unused, double *out_6983753669345123375);
void car_h_31(double *state, double *unused, double *out_3278691893586295999);
void car_H_31(double *state, double *unused, double *out_4899272993220273184);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}