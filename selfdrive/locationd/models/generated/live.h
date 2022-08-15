#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_5866002633423505928);
void live_err_fun(double *nom_x, double *delta_x, double *out_8354761777587577496);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8317700417448784408);
void live_H_mod_fun(double *state, double *out_4964267863056697179);
void live_f_fun(double *state, double dt, double *out_1405658321643130987);
void live_F_fun(double *state, double dt, double *out_6609914163391936085);
void live_h_4(double *state, double *unused, double *out_6011813983550882776);
void live_H_4(double *state, double *unused, double *out_2098171024544885215);
void live_h_9(double *state, double *unused, double *out_3902237370978173728);
void live_H_9(double *state, double *unused, double *out_9061354113900218931);
void live_h_10(double *state, double *unused, double *out_8822121513169463888);
void live_H_10(double *state, double *unused, double *out_5660519162938753925);
void live_h_12(double *state, double *unused, double *out_9198692797862017000);
void live_H_12(double *state, double *unused, double *out_4283087352497847781);
void live_h_31(double *state, double *unused, double *out_4675503687168181330);
void live_H_31(double *state, double *unused, double *out_1537524320172834072);
void live_h_32(double *state, double *unused, double *out_7047972252560525334);
void live_H_32(double *state, double *unused, double *out_4263780052034778486);
void live_h_13(double *state, double *unused, double *out_3252000326958521153);
void live_H_13(double *state, double *unused, double *out_5915097730122734872);
void live_h_14(double *state, double *unused, double *out_3902237370978173728);
void live_H_14(double *state, double *unused, double *out_9061354113900218931);
void live_h_33(double *state, double *unused, double *out_3675521570661779962);
void live_H_33(double *state, double *unused, double *out_1613032684466023532);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}