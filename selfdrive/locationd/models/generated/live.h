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
void live_H(double *in_vec, double *out_54580470952589197);
void live_err_fun(double *nom_x, double *delta_x, double *out_2331509198973380424);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8821701542344926915);
void live_H_mod_fun(double *state, double *out_9124059286284755301);
void live_f_fun(double *state, double dt, double *out_2050961891249539931);
void live_F_fun(double *state, double dt, double *out_8407074663770280591);
void live_h_4(double *state, double *unused, double *out_3054322920255068772);
void live_H_4(double *state, double *unused, double *out_5193953318351477594);
void live_h_9(double *state, double *unused, double *out_2537871891519615598);
void live_H_9(double *state, double *unused, double *out_5965571820093626552);
void live_h_10(double *state, double *unused, double *out_1988757008616586788);
void live_H_10(double *state, double *unused, double *out_6693269377593792927);
void live_h_12(double *state, double *unused, double *out_8692366223190510689);
void live_H_12(double *state, double *unused, double *out_8233334347326112227);
void live_h_31(double *state, double *unused, double *out_1066015054762197490);
void live_H_31(double *state, double *unused, double *out_5487771315001098518);
void live_h_32(double *state, double *unused, double *out_2582007418172209400);
void live_H_32(double *state, double *unused, double *out_2782746802099348581);
void live_h_13(double *state, double *unused, double *out_5961185238884941771);
void live_H_13(double *state, double *unused, double *out_4536921181361021519);
void live_h_14(double *state, double *unused, double *out_2537871891519615598);
void live_H_14(double *state, double *unused, double *out_5965571820093626552);
void live_h_33(double *state, double *unused, double *out_1863919135657502523);
void live_H_33(double *state, double *unused, double *out_6735571693346609042);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}