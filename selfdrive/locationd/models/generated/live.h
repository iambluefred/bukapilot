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
void live_H(double *in_vec, double *out_9113312896467362739);
void live_err_fun(double *nom_x, double *delta_x, double *out_964857952506476578);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6597616937229980506);
void live_H_mod_fun(double *state, double *out_4945460196648548679);
void live_f_fun(double *state, double dt, double *out_7518527184276760821);
void live_F_fun(double *state, double dt, double *out_6522816774983372922);
void live_h_4(double *state, double *unused, double *out_4178579306779853840);
void live_H_4(double *state, double *unused, double *out_8217236990378880001);
void live_h_9(double *state, double *unused, double *out_6691784437994501899);
void live_H_9(double *state, double *unused, double *out_7976047343749289356);
void live_h_10(double *state, double *unused, double *out_2084879992312823116);
void live_H_10(double *state, double *unused, double *out_6592478223540510774);
void live_h_12(double *state, double *unused, double *out_7924202137488387519);
void live_H_12(double *state, double *unused, double *out_3197780582346918206);
void live_h_31(double *state, double *unused, double *out_2696252750839045524);
void live_H_31(double *state, double *unused, double *out_4850574933006272625);
void live_h_32(double *state, double *unused, double *out_7710427550752746331);
void live_H_32(double *state, double *unused, double *out_4719410148065532156);
void live_h_13(double *state, double *unused, double *out_4869146066304455894);
void live_H_13(double *state, double *unused, double *out_4751063952776432307);
void live_h_14(double *state, double *unused, double *out_6691784437994501899);
void live_H_14(double *state, double *unused, double *out_7976047343749289356);
void live_h_33(double *state, double *unused, double *out_929856056033232204);
void live_H_33(double *state, double *unused, double *out_1700017928367415021);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}