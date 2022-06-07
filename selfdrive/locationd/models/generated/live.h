#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_3(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_19(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_5694089642169818639);
void live_err_fun(double *nom_x, double *delta_x, double *out_1408589648636065867);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1069709250804381752);
void live_H_mod_fun(double *state, double *out_7387798961821370812);
void live_f_fun(double *state, double dt, double *out_8051823809517006440);
void live_F_fun(double *state, double dt, double *out_5261609552440221843);
void live_h_3(double *state, double *unused, double *out_1177032665715907738);
void live_H_3(double *state, double *unused, double *out_8812597426033739510);
void live_h_4(double *state, double *unused, double *out_308534515165949765);
void live_H_4(double *state, double *unused, double *out_1954526312791180633);
void live_h_9(double *state, double *unused, double *out_8793789087446410386);
void live_H_9(double *state, double *unused, double *out_7414008519714532162);
void live_h_10(double *state, double *unused, double *out_2055575067814280922);
void live_H_10(double *state, double *unused, double *out_3486589718129474225);
void live_h_12(double *state, double *unused, double *out_7573773643040648417);
void live_H_12(double *state, double *unused, double *out_152029198948028936);
void live_h_31(double *state, double *unused, double *out_7362136700080033137);
void live_H_31(double *state, double *unused, double *out_2523060576195390885);
void live_h_32(double *state, double *unused, double *out_2532236420724041484);
void live_H_32(double *state, double *unused, double *out_445179216997480672);
void live_h_13(double *state, double *unused, double *out_7928545197341766488);
void live_H_13(double *state, double *unused, double *out_295976250519396060);
void live_h_14(double *state, double *unused, double *out_8793789087446410386);
void live_H_14(double *state, double *unused, double *out_7414008519714532162);
void live_h_19(double *state, double *unused, double *out_4170488759116613439);
void live_H_19(double *state, double *unused, double *out_6600055702892588273);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}