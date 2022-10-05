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
void live_H(double *in_vec, double *out_2508545050460247670);
void live_err_fun(double *nom_x, double *delta_x, double *out_8922224738887600792);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_9184353739647366519);
void live_H_mod_fun(double *state, double *out_8025929700250430649);
void live_f_fun(double *state, double dt, double *out_6546748259078843739);
void live_F_fun(double *state, double dt, double *out_820410862806496857);
void live_h_4(double *state, double *unused, double *out_7961884271916591371);
void live_H_4(double *state, double *unused, double *out_3591664531471880417);
void live_h_9(double *state, double *unused, double *out_9032137970758116100);
void live_H_9(double *state, double *unused, double *out_3695554403792567053);
void live_h_10(double *state, double *unused, double *out_3694327695067531867);
void live_H_10(double *state, double *unused, double *out_2496280317092357988);
void live_h_12(double *state, double *unused, double *out_7086267725388028713);
void live_H_12(double *state, double *unused, double *out_8473821165194938203);
void live_h_31(double *state, double *unused, double *out_4903278266110709116);
void live_H_31(double *state, double *unused, double *out_225002474099273041);
void live_h_32(double *state, double *unused, double *out_7636266217782030715);
void live_H_32(double *state, double *unused, double *out_3679515536202440975);
void live_h_13(double *state, double *unused, double *out_6202873061643521826);
void live_H_13(double *state, double *unused, double *out_2153824043286660686);
void live_h_14(double *state, double *unused, double *out_9032137970758116100);
void live_H_14(double *state, double *unused, double *out_3695554403792567053);
void live_h_33(double *state, double *unused, double *out_5544625881221946135);
void live_H_33(double *state, double *unused, double *out_2925554530539584563);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}