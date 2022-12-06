#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4279848910899479370) {
   out_4279848910899479370[0] = delta_x[0] + nom_x[0];
   out_4279848910899479370[1] = delta_x[1] + nom_x[1];
   out_4279848910899479370[2] = delta_x[2] + nom_x[2];
   out_4279848910899479370[3] = delta_x[3] + nom_x[3];
   out_4279848910899479370[4] = delta_x[4] + nom_x[4];
   out_4279848910899479370[5] = delta_x[5] + nom_x[5];
   out_4279848910899479370[6] = delta_x[6] + nom_x[6];
   out_4279848910899479370[7] = delta_x[7] + nom_x[7];
   out_4279848910899479370[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4328374563733165579) {
   out_4328374563733165579[0] = -nom_x[0] + true_x[0];
   out_4328374563733165579[1] = -nom_x[1] + true_x[1];
   out_4328374563733165579[2] = -nom_x[2] + true_x[2];
   out_4328374563733165579[3] = -nom_x[3] + true_x[3];
   out_4328374563733165579[4] = -nom_x[4] + true_x[4];
   out_4328374563733165579[5] = -nom_x[5] + true_x[5];
   out_4328374563733165579[6] = -nom_x[6] + true_x[6];
   out_4328374563733165579[7] = -nom_x[7] + true_x[7];
   out_4328374563733165579[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3923237233234693522) {
   out_3923237233234693522[0] = 1.0;
   out_3923237233234693522[1] = 0;
   out_3923237233234693522[2] = 0;
   out_3923237233234693522[3] = 0;
   out_3923237233234693522[4] = 0;
   out_3923237233234693522[5] = 0;
   out_3923237233234693522[6] = 0;
   out_3923237233234693522[7] = 0;
   out_3923237233234693522[8] = 0;
   out_3923237233234693522[9] = 0;
   out_3923237233234693522[10] = 1.0;
   out_3923237233234693522[11] = 0;
   out_3923237233234693522[12] = 0;
   out_3923237233234693522[13] = 0;
   out_3923237233234693522[14] = 0;
   out_3923237233234693522[15] = 0;
   out_3923237233234693522[16] = 0;
   out_3923237233234693522[17] = 0;
   out_3923237233234693522[18] = 0;
   out_3923237233234693522[19] = 0;
   out_3923237233234693522[20] = 1.0;
   out_3923237233234693522[21] = 0;
   out_3923237233234693522[22] = 0;
   out_3923237233234693522[23] = 0;
   out_3923237233234693522[24] = 0;
   out_3923237233234693522[25] = 0;
   out_3923237233234693522[26] = 0;
   out_3923237233234693522[27] = 0;
   out_3923237233234693522[28] = 0;
   out_3923237233234693522[29] = 0;
   out_3923237233234693522[30] = 1.0;
   out_3923237233234693522[31] = 0;
   out_3923237233234693522[32] = 0;
   out_3923237233234693522[33] = 0;
   out_3923237233234693522[34] = 0;
   out_3923237233234693522[35] = 0;
   out_3923237233234693522[36] = 0;
   out_3923237233234693522[37] = 0;
   out_3923237233234693522[38] = 0;
   out_3923237233234693522[39] = 0;
   out_3923237233234693522[40] = 1.0;
   out_3923237233234693522[41] = 0;
   out_3923237233234693522[42] = 0;
   out_3923237233234693522[43] = 0;
   out_3923237233234693522[44] = 0;
   out_3923237233234693522[45] = 0;
   out_3923237233234693522[46] = 0;
   out_3923237233234693522[47] = 0;
   out_3923237233234693522[48] = 0;
   out_3923237233234693522[49] = 0;
   out_3923237233234693522[50] = 1.0;
   out_3923237233234693522[51] = 0;
   out_3923237233234693522[52] = 0;
   out_3923237233234693522[53] = 0;
   out_3923237233234693522[54] = 0;
   out_3923237233234693522[55] = 0;
   out_3923237233234693522[56] = 0;
   out_3923237233234693522[57] = 0;
   out_3923237233234693522[58] = 0;
   out_3923237233234693522[59] = 0;
   out_3923237233234693522[60] = 1.0;
   out_3923237233234693522[61] = 0;
   out_3923237233234693522[62] = 0;
   out_3923237233234693522[63] = 0;
   out_3923237233234693522[64] = 0;
   out_3923237233234693522[65] = 0;
   out_3923237233234693522[66] = 0;
   out_3923237233234693522[67] = 0;
   out_3923237233234693522[68] = 0;
   out_3923237233234693522[69] = 0;
   out_3923237233234693522[70] = 1.0;
   out_3923237233234693522[71] = 0;
   out_3923237233234693522[72] = 0;
   out_3923237233234693522[73] = 0;
   out_3923237233234693522[74] = 0;
   out_3923237233234693522[75] = 0;
   out_3923237233234693522[76] = 0;
   out_3923237233234693522[77] = 0;
   out_3923237233234693522[78] = 0;
   out_3923237233234693522[79] = 0;
   out_3923237233234693522[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2379384011398423039) {
   out_2379384011398423039[0] = state[0];
   out_2379384011398423039[1] = state[1];
   out_2379384011398423039[2] = state[2];
   out_2379384011398423039[3] = state[3];
   out_2379384011398423039[4] = state[4];
   out_2379384011398423039[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2379384011398423039[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2379384011398423039[7] = state[7];
   out_2379384011398423039[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3041465841500109542) {
   out_3041465841500109542[0] = 1;
   out_3041465841500109542[1] = 0;
   out_3041465841500109542[2] = 0;
   out_3041465841500109542[3] = 0;
   out_3041465841500109542[4] = 0;
   out_3041465841500109542[5] = 0;
   out_3041465841500109542[6] = 0;
   out_3041465841500109542[7] = 0;
   out_3041465841500109542[8] = 0;
   out_3041465841500109542[9] = 0;
   out_3041465841500109542[10] = 1;
   out_3041465841500109542[11] = 0;
   out_3041465841500109542[12] = 0;
   out_3041465841500109542[13] = 0;
   out_3041465841500109542[14] = 0;
   out_3041465841500109542[15] = 0;
   out_3041465841500109542[16] = 0;
   out_3041465841500109542[17] = 0;
   out_3041465841500109542[18] = 0;
   out_3041465841500109542[19] = 0;
   out_3041465841500109542[20] = 1;
   out_3041465841500109542[21] = 0;
   out_3041465841500109542[22] = 0;
   out_3041465841500109542[23] = 0;
   out_3041465841500109542[24] = 0;
   out_3041465841500109542[25] = 0;
   out_3041465841500109542[26] = 0;
   out_3041465841500109542[27] = 0;
   out_3041465841500109542[28] = 0;
   out_3041465841500109542[29] = 0;
   out_3041465841500109542[30] = 1;
   out_3041465841500109542[31] = 0;
   out_3041465841500109542[32] = 0;
   out_3041465841500109542[33] = 0;
   out_3041465841500109542[34] = 0;
   out_3041465841500109542[35] = 0;
   out_3041465841500109542[36] = 0;
   out_3041465841500109542[37] = 0;
   out_3041465841500109542[38] = 0;
   out_3041465841500109542[39] = 0;
   out_3041465841500109542[40] = 1;
   out_3041465841500109542[41] = 0;
   out_3041465841500109542[42] = 0;
   out_3041465841500109542[43] = 0;
   out_3041465841500109542[44] = 0;
   out_3041465841500109542[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3041465841500109542[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3041465841500109542[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3041465841500109542[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3041465841500109542[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3041465841500109542[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3041465841500109542[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3041465841500109542[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3041465841500109542[53] = -9.8000000000000007*dt;
   out_3041465841500109542[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3041465841500109542[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3041465841500109542[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3041465841500109542[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3041465841500109542[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3041465841500109542[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3041465841500109542[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3041465841500109542[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3041465841500109542[62] = 0;
   out_3041465841500109542[63] = 0;
   out_3041465841500109542[64] = 0;
   out_3041465841500109542[65] = 0;
   out_3041465841500109542[66] = 0;
   out_3041465841500109542[67] = 0;
   out_3041465841500109542[68] = 0;
   out_3041465841500109542[69] = 0;
   out_3041465841500109542[70] = 1;
   out_3041465841500109542[71] = 0;
   out_3041465841500109542[72] = 0;
   out_3041465841500109542[73] = 0;
   out_3041465841500109542[74] = 0;
   out_3041465841500109542[75] = 0;
   out_3041465841500109542[76] = 0;
   out_3041465841500109542[77] = 0;
   out_3041465841500109542[78] = 0;
   out_3041465841500109542[79] = 0;
   out_3041465841500109542[80] = 1;
}
void h_25(double *state, double *unused, double *out_5343999116540519203) {
   out_5343999116540519203[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8401926576797338066) {
   out_8401926576797338066[0] = 0;
   out_8401926576797338066[1] = 0;
   out_8401926576797338066[2] = 0;
   out_8401926576797338066[3] = 0;
   out_8401926576797338066[4] = 0;
   out_8401926576797338066[5] = 0;
   out_8401926576797338066[6] = 1;
   out_8401926576797338066[7] = 0;
   out_8401926576797338066[8] = 0;
}
void h_24(double *state, double *unused, double *out_6923024055147605927) {
   out_6923024055147605927[0] = state[4];
   out_6923024055147605927[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6057929520912795245) {
   out_6057929520912795245[0] = 0;
   out_6057929520912795245[1] = 0;
   out_6057929520912795245[2] = 0;
   out_6057929520912795245[3] = 0;
   out_6057929520912795245[4] = 1;
   out_6057929520912795245[5] = 0;
   out_6057929520912795245[6] = 0;
   out_6057929520912795245[7] = 0;
   out_6057929520912795245[8] = 0;
   out_6057929520912795245[9] = 0;
   out_6057929520912795245[10] = 0;
   out_6057929520912795245[11] = 0;
   out_6057929520912795245[12] = 0;
   out_6057929520912795245[13] = 0;
   out_6057929520912795245[14] = 1;
   out_6057929520912795245[15] = 0;
   out_6057929520912795245[16] = 0;
   out_6057929520912795245[17] = 0;
}
void h_30(double *state, double *unused, double *out_2419266140280153238) {
   out_2419266140280153238[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3874230246669729868) {
   out_3874230246669729868[0] = 0;
   out_3874230246669729868[1] = 0;
   out_3874230246669729868[2] = 0;
   out_3874230246669729868[3] = 0;
   out_3874230246669729868[4] = 1;
   out_3874230246669729868[5] = 0;
   out_3874230246669729868[6] = 0;
   out_3874230246669729868[7] = 0;
   out_3874230246669729868[8] = 0;
}
void h_26(double *state, double *unused, double *out_3979775377133695908) {
   out_3979775377133695908[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4660423257923281842) {
   out_4660423257923281842[0] = 0;
   out_4660423257923281842[1] = 0;
   out_4660423257923281842[2] = 0;
   out_4660423257923281842[3] = 0;
   out_4660423257923281842[4] = 0;
   out_4660423257923281842[5] = 0;
   out_4660423257923281842[6] = 0;
   out_4660423257923281842[7] = 1;
   out_4660423257923281842[8] = 0;
}
void h_27(double *state, double *unused, double *out_8036327438450689690) {
   out_8036327438450689690[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1699466934869304957) {
   out_1699466934869304957[0] = 0;
   out_1699466934869304957[1] = 0;
   out_1699466934869304957[2] = 0;
   out_1699466934869304957[3] = 1;
   out_1699466934869304957[4] = 0;
   out_1699466934869304957[5] = 0;
   out_1699466934869304957[6] = 0;
   out_1699466934869304957[7] = 0;
   out_1699466934869304957[8] = 0;
}
void h_29(double *state, double *unused, double *out_2202919889175955076) {
   out_2202919889175955076[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4384461590984122052) {
   out_4384461590984122052[0] = 0;
   out_4384461590984122052[1] = 1;
   out_4384461590984122052[2] = 0;
   out_4384461590984122052[3] = 0;
   out_4384461590984122052[4] = 0;
   out_4384461590984122052[5] = 0;
   out_4384461590984122052[6] = 0;
   out_4384461590984122052[7] = 0;
   out_4384461590984122052[8] = 0;
}
void h_28(double *state, double *unused, double *out_7806011314450164275) {
   out_7806011314450164275[0] = state[0];
}
void H_28(double *state, double *unused, double *out_697937426085408522) {
   out_697937426085408522[0] = 1;
   out_697937426085408522[1] = 0;
   out_697937426085408522[2] = 0;
   out_697937426085408522[3] = 0;
   out_697937426085408522[4] = 0;
   out_697937426085408522[5] = 0;
   out_697937426085408522[6] = 0;
   out_697937426085408522[7] = 0;
   out_697937426085408522[8] = 0;
}
void h_31(double *state, double *unused, double *out_5198479292319864232) {
   out_5198479292319864232[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4034215155689930366) {
   out_4034215155689930366[0] = 0;
   out_4034215155689930366[1] = 0;
   out_4034215155689930366[2] = 0;
   out_4034215155689930366[3] = 0;
   out_4034215155689930366[4] = 0;
   out_4034215155689930366[5] = 0;
   out_4034215155689930366[6] = 0;
   out_4034215155689930366[7] = 0;
   out_4034215155689930366[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_4279848910899479370) {
  err_fun(nom_x, delta_x, out_4279848910899479370);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4328374563733165579) {
  inv_err_fun(nom_x, true_x, out_4328374563733165579);
}
void car_H_mod_fun(double *state, double *out_3923237233234693522) {
  H_mod_fun(state, out_3923237233234693522);
}
void car_f_fun(double *state, double dt, double *out_2379384011398423039) {
  f_fun(state,  dt, out_2379384011398423039);
}
void car_F_fun(double *state, double dt, double *out_3041465841500109542) {
  F_fun(state,  dt, out_3041465841500109542);
}
void car_h_25(double *state, double *unused, double *out_5343999116540519203) {
  h_25(state, unused, out_5343999116540519203);
}
void car_H_25(double *state, double *unused, double *out_8401926576797338066) {
  H_25(state, unused, out_8401926576797338066);
}
void car_h_24(double *state, double *unused, double *out_6923024055147605927) {
  h_24(state, unused, out_6923024055147605927);
}
void car_H_24(double *state, double *unused, double *out_6057929520912795245) {
  H_24(state, unused, out_6057929520912795245);
}
void car_h_30(double *state, double *unused, double *out_2419266140280153238) {
  h_30(state, unused, out_2419266140280153238);
}
void car_H_30(double *state, double *unused, double *out_3874230246669729868) {
  H_30(state, unused, out_3874230246669729868);
}
void car_h_26(double *state, double *unused, double *out_3979775377133695908) {
  h_26(state, unused, out_3979775377133695908);
}
void car_H_26(double *state, double *unused, double *out_4660423257923281842) {
  H_26(state, unused, out_4660423257923281842);
}
void car_h_27(double *state, double *unused, double *out_8036327438450689690) {
  h_27(state, unused, out_8036327438450689690);
}
void car_H_27(double *state, double *unused, double *out_1699466934869304957) {
  H_27(state, unused, out_1699466934869304957);
}
void car_h_29(double *state, double *unused, double *out_2202919889175955076) {
  h_29(state, unused, out_2202919889175955076);
}
void car_H_29(double *state, double *unused, double *out_4384461590984122052) {
  H_29(state, unused, out_4384461590984122052);
}
void car_h_28(double *state, double *unused, double *out_7806011314450164275) {
  h_28(state, unused, out_7806011314450164275);
}
void car_H_28(double *state, double *unused, double *out_697937426085408522) {
  H_28(state, unused, out_697937426085408522);
}
void car_h_31(double *state, double *unused, double *out_5198479292319864232) {
  h_31(state, unused, out_5198479292319864232);
}
void car_H_31(double *state, double *unused, double *out_4034215155689930366) {
  H_31(state, unused, out_4034215155689930366);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
