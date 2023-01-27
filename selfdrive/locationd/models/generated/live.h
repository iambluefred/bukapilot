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
void live_H(double *in_vec, double *out_1967997356340866583);
void live_err_fun(double *nom_x, double *delta_x, double *out_5319645388126622084);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4147357239501958364);
void live_H_mod_fun(double *state, double *out_4013764754807942187);
void live_f_fun(double *state, double dt, double *out_6649622138097899585);
void live_F_fun(double *state, double dt, double *out_5292336351074770392);
void live_h_4(double *state, double *unused, double *out_7730102872744429728);
void live_H_4(double *state, double *unused, double *out_3258285380341575150);
void live_h_9(double *state, double *unused, double *out_2507810790711750711);
void live_H_9(double *state, double *unused, double *out_7901239758103528996);
void live_h_10(double *state, double *unused, double *out_5427455976156256445);
void live_H_10(double *state, double *unused, double *out_4292991088859467531);
void live_h_12(double *state, double *unused, double *out_2256952412241767232);
void live_H_12(double *state, double *unused, double *out_7521330379685525974);
void live_h_31(double *state, double *unused, double *out_216992504528642618);
void live_H_31(double *state, double *unused, double *out_4775767347360512265);
void live_h_32(double *state, double *unused, double *out_6575289537096574912);
void live_H_32(double *state, double *unused, double *out_5088389042247957293);
void live_h_13(double *state, double *unused, double *out_3557615268607894996);
void live_H_13(double *state, double *unused, double *out_7345515301927333450);
void live_h_14(double *state, double *unused, double *out_2507810790711750711);
void live_H_14(double *state, double *unused, double *out_7901239758103528996);
void live_h_33(double *state, double *unused, double *out_3788603540548372441);
void live_H_33(double *state, double *unused, double *out_1625210342721654661);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}