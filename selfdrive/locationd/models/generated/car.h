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
void car_err_fun(double *nom_x, double *delta_x, double *out_6707971512290491444);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7853994834193184287);
void car_H_mod_fun(double *state, double *out_4010081315495760417);
void car_f_fun(double *state, double dt, double *out_4685496284796261050);
void car_F_fun(double *state, double dt, double *out_8622935208793728795);
void car_h_25(double *state, double *unused, double *out_1413316984438352955);
void car_H_25(double *state, double *unused, double *out_7359072756142844022);
void car_h_24(double *state, double *unused, double *out_1193875120514303785);
void car_H_24(double *state, double *unused, double *out_6341161990600198560);
void car_h_30(double *state, double *unused, double *out_1256130316087223810);
void car_H_30(double *state, double *unused, double *out_4840739797635595395);
void car_h_26(double *state, double *unused, double *out_2944996651036528421);
void car_H_26(double *state, double *unused, double *out_4054546786382043421);
void car_h_27(double *state, double *unused, double *out_6992339835205873698);
void car_H_27(double *state, double *unused, double *out_7015503109436020306);
void car_h_29(double *state, double *unused, double *out_8170070345181851364);
void car_H_29(double *state, double *unused, double *out_4330508453321203211);
void car_h_28(double *state, double *unused, double *out_3602438148829345217);
void car_H_28(double *state, double *unused, double *out_9033836603318817831);
void car_h_31(double *state, double *unused, double *out_1138122922153847066);
void car_H_31(double *state, double *unused, double *out_6719959896459299894);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}