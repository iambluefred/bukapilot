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
void live_H(double *in_vec, double *out_7992627687795914890);
void live_err_fun(double *nom_x, double *delta_x, double *out_3696685892434487592);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2519379675693431560);
void live_H_mod_fun(double *state, double *out_223824781137620440);
void live_f_fun(double *state, double dt, double *out_2774259741538367402);
void live_F_fun(double *state, double dt, double *out_5215984573198861592);
void live_h_3(double *state, double *unused, double *out_6374464597301679621);
void live_H_3(double *state, double *unused, double *out_6228146732953956280);
void live_h_4(double *state, double *unused, double *out_3510840487962717367);
void live_H_4(double *state, double *unused, double *out_8720116550257947860);
void live_h_9(double *state, double *unused, double *out_8212180715317447269);
void live_H_9(double *state, double *unused, double *out_358092690945890961);
void live_h_10(double *state, double *unused, double *out_389257673982922167);
void live_H_10(double *state, double *unused, double *out_8087727374197752058);
void live_h_12(double *state, double *unused, double *out_4459191647121467328);
void live_H_12(double *state, double *unused, double *out_7620072011712394187);
void live_h_31(double *state, double *unused, double *out_3511632025541620439);
void live_H_31(double *state, double *unused, double *out_8151582286853737608);
void live_h_32(double *state, double *unused, double *out_4930517077760155346);
void live_H_32(double *state, double *unused, double *out_2340614530331382204);
void live_h_13(double *state, double *unused, double *out_6788607034453464411);
void live_H_13(double *state, double *unused, double *out_4534418185260871932);
void live_h_14(double *state, double *unused, double *out_8212180715317447269);
void live_H_14(double *state, double *unused, double *out_358092690945890961);
void live_h_19(double *state, double *unused, double *out_986408198655348522);
void live_H_19(double *state, double *unused, double *out_4074587160156540220);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}