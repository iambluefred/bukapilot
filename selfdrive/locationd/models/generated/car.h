#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_5906788765990308544);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1726895774273068326);
void car_H_mod_fun(double *state, double *out_5542936510909561841);
void car_f_fun(double *state, double dt, double *out_1065943893522122862);
void car_F_fun(double *state, double dt, double *out_6663051540238902053);
void car_h_25(double *state, double *unused, double *out_4227256557803292572);
void car_H_25(double *state, double *unused, double *out_1405888400492708987);
void car_h_24(double *state, double *unused, double *out_7727802183161571239);
void car_H_24(double *state, double *unused, double *out_565456616310663995);
void car_h_30(double *state, double *unused, double *out_4502450620087798461);
void car_H_30(double *state, double *unused, double *out_3121807929634899211);
void car_h_26(double *state, double *unused, double *out_853130833811377404);
void car_H_26(double *state, double *unused, double *out_2335614918381347237);
void car_h_27(double *state, double *unused, double *out_9217814248245413399);
void car_H_27(double *state, double *unused, double *out_5296571241435324122);
void car_h_29(double *state, double *unused, double *out_5580364719991391049);
void car_H_29(double *state, double *unused, double *out_2611576585320507027);
void car_h_28(double *state, double *unused, double *out_8285419972562475782);
void car_H_28(double *state, double *unused, double *out_647946313755180776);
void car_h_31(double *state, double *unused, double *out_1279530567447807611);
void car_H_31(double *state, double *unused, double *out_2961823020614698713);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}