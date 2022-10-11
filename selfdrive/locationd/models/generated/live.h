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
void live_H(double *in_vec, double *out_5017127939989894028);
void live_err_fun(double *nom_x, double *delta_x, double *out_6740750633202003938);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4003804904042118845);
void live_H_mod_fun(double *state, double *out_3147787631266448461);
void live_f_fun(double *state, double dt, double *out_400464903206441192);
void live_F_fun(double *state, double dt, double *out_1608606016964525974);
void live_h_4(double *state, double *unused, double *out_5810520985716428538);
void live_H_4(double *state, double *unused, double *out_8101259356736239473);
void live_h_9(double *state, double *unused, double *out_2994849396655473689);
void live_H_9(double *state, double *unused, double *out_8342449003365830118);
void live_h_10(double *state, double *unused, double *out_6942493849852737304);
void live_H_10(double *state, double *unused, double *out_213825020102705639);
void live_h_12(double *state, double *unused, double *out_841890056352501808);
void live_H_12(double *state, double *unused, double *out_5326028308941350348);
void live_h_31(double *state, double *unused, double *out_570532822292067381);
void live_H_31(double *state, double *unused, double *out_6978822659600704767);
void live_h_32(double *state, double *unused, double *out_3795146207836411375);
void live_H_32(double *state, double *unused, double *out_4553056910414730493);
void live_h_13(double *state, double *unused, double *out_309080432191951178);
void live_H_13(double *state, double *unused, double *out_7608928095959949541);
void live_h_14(double *state, double *unused, double *out_2994849396655473689);
void live_H_14(double *state, double *unused, double *out_8342449003365830118);
void live_h_33(double *state, double *unused, double *out_2775908019009971139);
void live_H_33(double *state, double *unused, double *out_3828265654961847163);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}