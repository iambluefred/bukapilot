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
void car_err_fun(double *nom_x, double *delta_x, double *out_4005078601906462987);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6660233397225138441);
void car_H_mod_fun(double *state, double *out_6074328353129280536);
void car_f_fun(double *state, double dt, double *out_1156634792006458643);
void car_F_fun(double *state, double dt, double *out_6849697270777504597);
void car_h_25(double *state, double *unused, double *out_8290085571197364282);
void car_H_25(double *state, double *unused, double *out_690624191224379626);
void car_h_24(double *state, double *unused, double *out_2505692210727262820);
void car_H_24(double *state, double *unused, double *out_1087269495578239121);
void car_h_30(double *state, double *unused, double *out_6963440463520399393);
void car_H_30(double *state, double *unused, double *out_1827708767282869001);
void car_h_26(double *state, double *unused, double *out_2686994145923155083);
void car_H_26(double *state, double *unused, double *out_4432127510098435850);
void car_h_27(double *state, double *unused, double *out_5534472473056079020);
void car_H_27(double *state, double *unused, double *out_4051302838466812218);
void car_h_29(double *state, double *unused, double *out_1897022944802056670);
void car_H_29(double *state, double *unused, double *out_2337940111597261185);
void car_h_28(double *state, double *unused, double *out_532799205395233375);
void car_H_28(double *state, double *unused, double *out_2744458905472269389);
void car_h_31(double *state, double *unused, double *out_8447272239548493427);
void car_H_31(double *state, double *unused, double *out_5058335612331787326);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}