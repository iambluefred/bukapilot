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
void live_H(double *in_vec, double *out_8263339834962583163);
void live_err_fun(double *nom_x, double *delta_x, double *out_374402640541161811);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1712409039458462772);
void live_H_mod_fun(double *state, double *out_7904389955720533837);
void live_f_fun(double *state, double dt, double *out_3969113685218421091);
void live_F_fun(double *state, double dt, double *out_3394291750586310402);
void live_h_4(double *state, double *unused, double *out_2715917266781275174);
void live_H_4(double *state, double *unused, double *out_8196307344783457385);
void live_h_9(double *state, double *unused, double *out_5061189772138543239);
void live_H_9(double *state, double *unused, double *out_8437496991413048030);
void live_h_10(double *state, double *unused, double *out_5099401147393741763);
void live_H_10(double *state, double *unused, double *out_7955627714739753803);
void live_h_12(double *state, double *unused, double *out_1436772025734485824);
void live_H_12(double *state, double *unused, double *out_5230980320894132436);
void live_h_31(double *state, double *unused, double *out_5658326879621691075);
void live_H_31(double *state, double *unused, double *out_2485417288569118727);
void live_h_32(double *state, double *unused, double *out_9019943801864809423);
void live_H_32(double *state, double *unused, double *out_1062427051418040002);
void live_h_13(double *state, double *unused, double *out_5083526561539254523);
void live_H_13(double *state, double *unused, double *out_6425517206559759720);
void live_h_14(double *state, double *unused, double *out_5061189772138543239);
void live_H_14(double *state, double *unused, double *out_8437496991413048030);
void live_h_33(double *state, double *unused, double *out_5283518884697574127);
void live_H_33(double *state, double *unused, double *out_3733217666914629251);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}