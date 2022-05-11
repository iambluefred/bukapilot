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
void live_H(double *in_vec, double *out_1926352033161799347);
void live_err_fun(double *nom_x, double *delta_x, double *out_5851487932107165381);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2054499611789129529);
void live_H_mod_fun(double *state, double *out_6233201449860091651);
void live_f_fun(double *state, double dt, double *out_3410304083448560036);
void live_F_fun(double *state, double dt, double *out_5654863995262940941);
void live_h_3(double *state, double *unused, double *out_5100590768938223208);
void live_H_3(double *state, double *unused, double *out_2407292730647848631);
void live_h_4(double *state, double *unused, double *out_4147124164089837445);
void live_H_4(double *state, double *unused, double *out_6605365188952738618);
void live_h_9(double *state, double *unused, double *out_2823378222322644005);
void live_H_9(double *state, double *unused, double *out_4282859645081882648);
void live_h_10(double *state, double *unused, double *out_6553802494680310299);
void live_H_10(double *state, double *unused, double *out_5449479205520640347);
void live_h_12(double *state, double *unused, double *out_6360644131885373515);
void live_H_12(double *state, double *unused, double *out_4498809677213529049);
void live_h_31(double *state, double *unused, double *out_7182486647620733571);
void live_H_31(double *state, double *unused, double *out_7173899452356948870);
void live_h_32(double *state, double *unused, double *out_7130992700841575480);
void live_H_32(double *state, double *unused, double *out_9124401174004700517);
void live_h_13(double *state, double *unused, double *out_8264355870367097605);
void live_H_13(double *state, double *unused, double *out_1468246092605740017);
void live_h_14(double *state, double *unused, double *out_2823378222322644005);
void live_H_14(double *state, double *unused, double *out_4282859645081882648);
void live_h_19(double *state, double *unused, double *out_5444115565338791622);
void live_H_19(double *state, double *unused, double *out_149820206020548533);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}