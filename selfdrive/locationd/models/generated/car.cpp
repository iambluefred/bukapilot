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
void err_fun(double *nom_x, double *delta_x, double *out_8062980869779089220) {
   out_8062980869779089220[0] = delta_x[0] + nom_x[0];
   out_8062980869779089220[1] = delta_x[1] + nom_x[1];
   out_8062980869779089220[2] = delta_x[2] + nom_x[2];
   out_8062980869779089220[3] = delta_x[3] + nom_x[3];
   out_8062980869779089220[4] = delta_x[4] + nom_x[4];
   out_8062980869779089220[5] = delta_x[5] + nom_x[5];
   out_8062980869779089220[6] = delta_x[6] + nom_x[6];
   out_8062980869779089220[7] = delta_x[7] + nom_x[7];
   out_8062980869779089220[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_508088850886154557) {
   out_508088850886154557[0] = -nom_x[0] + true_x[0];
   out_508088850886154557[1] = -nom_x[1] + true_x[1];
   out_508088850886154557[2] = -nom_x[2] + true_x[2];
   out_508088850886154557[3] = -nom_x[3] + true_x[3];
   out_508088850886154557[4] = -nom_x[4] + true_x[4];
   out_508088850886154557[5] = -nom_x[5] + true_x[5];
   out_508088850886154557[6] = -nom_x[6] + true_x[6];
   out_508088850886154557[7] = -nom_x[7] + true_x[7];
   out_508088850886154557[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3427232086650733882) {
   out_3427232086650733882[0] = 1.0;
   out_3427232086650733882[1] = 0;
   out_3427232086650733882[2] = 0;
   out_3427232086650733882[3] = 0;
   out_3427232086650733882[4] = 0;
   out_3427232086650733882[5] = 0;
   out_3427232086650733882[6] = 0;
   out_3427232086650733882[7] = 0;
   out_3427232086650733882[8] = 0;
   out_3427232086650733882[9] = 0;
   out_3427232086650733882[10] = 1.0;
   out_3427232086650733882[11] = 0;
   out_3427232086650733882[12] = 0;
   out_3427232086650733882[13] = 0;
   out_3427232086650733882[14] = 0;
   out_3427232086650733882[15] = 0;
   out_3427232086650733882[16] = 0;
   out_3427232086650733882[17] = 0;
   out_3427232086650733882[18] = 0;
   out_3427232086650733882[19] = 0;
   out_3427232086650733882[20] = 1.0;
   out_3427232086650733882[21] = 0;
   out_3427232086650733882[22] = 0;
   out_3427232086650733882[23] = 0;
   out_3427232086650733882[24] = 0;
   out_3427232086650733882[25] = 0;
   out_3427232086650733882[26] = 0;
   out_3427232086650733882[27] = 0;
   out_3427232086650733882[28] = 0;
   out_3427232086650733882[29] = 0;
   out_3427232086650733882[30] = 1.0;
   out_3427232086650733882[31] = 0;
   out_3427232086650733882[32] = 0;
   out_3427232086650733882[33] = 0;
   out_3427232086650733882[34] = 0;
   out_3427232086650733882[35] = 0;
   out_3427232086650733882[36] = 0;
   out_3427232086650733882[37] = 0;
   out_3427232086650733882[38] = 0;
   out_3427232086650733882[39] = 0;
   out_3427232086650733882[40] = 1.0;
   out_3427232086650733882[41] = 0;
   out_3427232086650733882[42] = 0;
   out_3427232086650733882[43] = 0;
   out_3427232086650733882[44] = 0;
   out_3427232086650733882[45] = 0;
   out_3427232086650733882[46] = 0;
   out_3427232086650733882[47] = 0;
   out_3427232086650733882[48] = 0;
   out_3427232086650733882[49] = 0;
   out_3427232086650733882[50] = 1.0;
   out_3427232086650733882[51] = 0;
   out_3427232086650733882[52] = 0;
   out_3427232086650733882[53] = 0;
   out_3427232086650733882[54] = 0;
   out_3427232086650733882[55] = 0;
   out_3427232086650733882[56] = 0;
   out_3427232086650733882[57] = 0;
   out_3427232086650733882[58] = 0;
   out_3427232086650733882[59] = 0;
   out_3427232086650733882[60] = 1.0;
   out_3427232086650733882[61] = 0;
   out_3427232086650733882[62] = 0;
   out_3427232086650733882[63] = 0;
   out_3427232086650733882[64] = 0;
   out_3427232086650733882[65] = 0;
   out_3427232086650733882[66] = 0;
   out_3427232086650733882[67] = 0;
   out_3427232086650733882[68] = 0;
   out_3427232086650733882[69] = 0;
   out_3427232086650733882[70] = 1.0;
   out_3427232086650733882[71] = 0;
   out_3427232086650733882[72] = 0;
   out_3427232086650733882[73] = 0;
   out_3427232086650733882[74] = 0;
   out_3427232086650733882[75] = 0;
   out_3427232086650733882[76] = 0;
   out_3427232086650733882[77] = 0;
   out_3427232086650733882[78] = 0;
   out_3427232086650733882[79] = 0;
   out_3427232086650733882[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2337273054858271848) {
   out_2337273054858271848[0] = state[0];
   out_2337273054858271848[1] = state[1];
   out_2337273054858271848[2] = state[2];
   out_2337273054858271848[3] = state[3];
   out_2337273054858271848[4] = state[4];
   out_2337273054858271848[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2337273054858271848[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2337273054858271848[7] = state[7];
   out_2337273054858271848[8] = state[8];
}
void F_fun(double *state, double dt, double *out_435201181327936292) {
   out_435201181327936292[0] = 1;
   out_435201181327936292[1] = 0;
   out_435201181327936292[2] = 0;
   out_435201181327936292[3] = 0;
   out_435201181327936292[4] = 0;
   out_435201181327936292[5] = 0;
   out_435201181327936292[6] = 0;
   out_435201181327936292[7] = 0;
   out_435201181327936292[8] = 0;
   out_435201181327936292[9] = 0;
   out_435201181327936292[10] = 1;
   out_435201181327936292[11] = 0;
   out_435201181327936292[12] = 0;
   out_435201181327936292[13] = 0;
   out_435201181327936292[14] = 0;
   out_435201181327936292[15] = 0;
   out_435201181327936292[16] = 0;
   out_435201181327936292[17] = 0;
   out_435201181327936292[18] = 0;
   out_435201181327936292[19] = 0;
   out_435201181327936292[20] = 1;
   out_435201181327936292[21] = 0;
   out_435201181327936292[22] = 0;
   out_435201181327936292[23] = 0;
   out_435201181327936292[24] = 0;
   out_435201181327936292[25] = 0;
   out_435201181327936292[26] = 0;
   out_435201181327936292[27] = 0;
   out_435201181327936292[28] = 0;
   out_435201181327936292[29] = 0;
   out_435201181327936292[30] = 1;
   out_435201181327936292[31] = 0;
   out_435201181327936292[32] = 0;
   out_435201181327936292[33] = 0;
   out_435201181327936292[34] = 0;
   out_435201181327936292[35] = 0;
   out_435201181327936292[36] = 0;
   out_435201181327936292[37] = 0;
   out_435201181327936292[38] = 0;
   out_435201181327936292[39] = 0;
   out_435201181327936292[40] = 1;
   out_435201181327936292[41] = 0;
   out_435201181327936292[42] = 0;
   out_435201181327936292[43] = 0;
   out_435201181327936292[44] = 0;
   out_435201181327936292[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_435201181327936292[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_435201181327936292[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_435201181327936292[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_435201181327936292[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_435201181327936292[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_435201181327936292[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_435201181327936292[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_435201181327936292[53] = -9.8000000000000007*dt;
   out_435201181327936292[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_435201181327936292[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_435201181327936292[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_435201181327936292[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_435201181327936292[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_435201181327936292[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_435201181327936292[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_435201181327936292[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_435201181327936292[62] = 0;
   out_435201181327936292[63] = 0;
   out_435201181327936292[64] = 0;
   out_435201181327936292[65] = 0;
   out_435201181327936292[66] = 0;
   out_435201181327936292[67] = 0;
   out_435201181327936292[68] = 0;
   out_435201181327936292[69] = 0;
   out_435201181327936292[70] = 1;
   out_435201181327936292[71] = 0;
   out_435201181327936292[72] = 0;
   out_435201181327936292[73] = 0;
   out_435201181327936292[74] = 0;
   out_435201181327936292[75] = 0;
   out_435201181327936292[76] = 0;
   out_435201181327936292[77] = 0;
   out_435201181327936292[78] = 0;
   out_435201181327936292[79] = 0;
   out_435201181327936292[80] = 1;
}
void h_25(double *state, double *unused, double *out_2760927816007907288) {
   out_2760927816007907288[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8305718187882283659) {
   out_8305718187882283659[0] = 0;
   out_8305718187882283659[1] = 0;
   out_8305718187882283659[2] = 0;
   out_8305718187882283659[3] = 0;
   out_8305718187882283659[4] = 0;
   out_8305718187882283659[5] = 0;
   out_8305718187882283659[6] = 1;
   out_8305718187882283659[7] = 0;
   out_8305718187882283659[8] = 0;
}
void h_24(double *state, double *unused, double *out_6700003583289376697) {
   out_6700003583289376697[0] = state[4];
   out_6700003583289376697[1] = state[5];
}
void H_24(double *state, double *unused, double *out_9150714796665979058) {
   out_9150714796665979058[0] = 0;
   out_9150714796665979058[1] = 0;
   out_9150714796665979058[2] = 0;
   out_9150714796665979058[3] = 0;
   out_9150714796665979058[4] = 1;
   out_9150714796665979058[5] = 0;
   out_9150714796665979058[6] = 0;
   out_9150714796665979058[7] = 0;
   out_9150714796665979058[8] = 0;
   out_9150714796665979058[9] = 0;
   out_9150714796665979058[10] = 0;
   out_9150714796665979058[11] = 0;
   out_9150714796665979058[12] = 0;
   out_9150714796665979058[13] = 0;
   out_9150714796665979058[14] = 1;
   out_9150714796665979058[15] = 0;
   out_9150714796665979058[16] = 0;
   out_9150714796665979058[17] = 0;
}
void h_30(double *state, double *unused, double *out_8914981031351293392) {
   out_8914981031351293392[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3224335544335651202) {
   out_3224335544335651202[0] = 0;
   out_3224335544335651202[1] = 0;
   out_3224335544335651202[2] = 0;
   out_3224335544335651202[3] = 0;
   out_3224335544335651202[4] = 1;
   out_3224335544335651202[5] = 0;
   out_3224335544335651202[6] = 0;
   out_3224335544335651202[7] = 0;
   out_3224335544335651202[8] = 0;
}
void h_26(double *state, double *unused, double *out_3324605254877637169) {
   out_3324605254877637169[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4564214869008227435) {
   out_4564214869008227435[0] = 0;
   out_4564214869008227435[1] = 0;
   out_4564214869008227435[2] = 0;
   out_4564214869008227435[3] = 0;
   out_4564214869008227435[4] = 0;
   out_4564214869008227435[5] = 0;
   out_4564214869008227435[6] = 0;
   out_4564214869008227435[7] = 1;
   out_4564214869008227435[8] = 0;
}
void h_27(double *state, double *unused, double *out_3206597860944260425) {
   out_3206597860944260425[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5399098856136076113) {
   out_5399098856136076113[0] = 0;
   out_5399098856136076113[1] = 0;
   out_5399098856136076113[2] = 0;
   out_5399098856136076113[3] = 1;
   out_5399098856136076113[4] = 0;
   out_5399098856136076113[5] = 0;
   out_5399098856136076113[6] = 0;
   out_5399098856136076113[7] = 0;
   out_5399098856136076113[8] = 0;
}
void h_29(double *state, double *unused, double *out_6102428928081440009) {
   out_6102428928081440009[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2714104200021259018) {
   out_2714104200021259018[0] = 0;
   out_2714104200021259018[1] = 1;
   out_2714104200021259018[2] = 0;
   out_2714104200021259018[3] = 0;
   out_2714104200021259018[4] = 0;
   out_2714104200021259018[5] = 0;
   out_2714104200021259018[6] = 0;
   out_2714104200021259018[7] = 0;
   out_2714104200021259018[8] = 0;
}
void h_28(double *state, double *unused, double *out_7488161702187920933) {
   out_7488161702187920933[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7796503217090789592) {
   out_7796503217090789592[0] = 1;
   out_7796503217090789592[1] = 0;
   out_7796503217090789592[2] = 0;
   out_7796503217090789592[3] = 0;
   out_7796503217090789592[4] = 0;
   out_7796503217090789592[5] = 0;
   out_7796503217090789592[6] = 0;
   out_7796503217090789592[7] = 0;
   out_7796503217090789592[8] = 0;
}
void h_31(double *state, double *unused, double *out_7671597222049239594) {
   out_7671597222049239594[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8336364149759244087) {
   out_8336364149759244087[0] = 0;
   out_8336364149759244087[1] = 0;
   out_8336364149759244087[2] = 0;
   out_8336364149759244087[3] = 0;
   out_8336364149759244087[4] = 0;
   out_8336364149759244087[5] = 0;
   out_8336364149759244087[6] = 0;
   out_8336364149759244087[7] = 0;
   out_8336364149759244087[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8062980869779089220) {
  err_fun(nom_x, delta_x, out_8062980869779089220);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_508088850886154557) {
  inv_err_fun(nom_x, true_x, out_508088850886154557);
}
void car_H_mod_fun(double *state, double *out_3427232086650733882) {
  H_mod_fun(state, out_3427232086650733882);
}
void car_f_fun(double *state, double dt, double *out_2337273054858271848) {
  f_fun(state,  dt, out_2337273054858271848);
}
void car_F_fun(double *state, double dt, double *out_435201181327936292) {
  F_fun(state,  dt, out_435201181327936292);
}
void car_h_25(double *state, double *unused, double *out_2760927816007907288) {
  h_25(state, unused, out_2760927816007907288);
}
void car_H_25(double *state, double *unused, double *out_8305718187882283659) {
  H_25(state, unused, out_8305718187882283659);
}
void car_h_24(double *state, double *unused, double *out_6700003583289376697) {
  h_24(state, unused, out_6700003583289376697);
}
void car_H_24(double *state, double *unused, double *out_9150714796665979058) {
  H_24(state, unused, out_9150714796665979058);
}
void car_h_30(double *state, double *unused, double *out_8914981031351293392) {
  h_30(state, unused, out_8914981031351293392);
}
void car_H_30(double *state, double *unused, double *out_3224335544335651202) {
  H_30(state, unused, out_3224335544335651202);
}
void car_h_26(double *state, double *unused, double *out_3324605254877637169) {
  h_26(state, unused, out_3324605254877637169);
}
void car_H_26(double *state, double *unused, double *out_4564214869008227435) {
  H_26(state, unused, out_4564214869008227435);
}
void car_h_27(double *state, double *unused, double *out_3206597860944260425) {
  h_27(state, unused, out_3206597860944260425);
}
void car_H_27(double *state, double *unused, double *out_5399098856136076113) {
  H_27(state, unused, out_5399098856136076113);
}
void car_h_29(double *state, double *unused, double *out_6102428928081440009) {
  h_29(state, unused, out_6102428928081440009);
}
void car_H_29(double *state, double *unused, double *out_2714104200021259018) {
  H_29(state, unused, out_2714104200021259018);
}
void car_h_28(double *state, double *unused, double *out_7488161702187920933) {
  h_28(state, unused, out_7488161702187920933);
}
void car_H_28(double *state, double *unused, double *out_7796503217090789592) {
  H_28(state, unused, out_7796503217090789592);
}
void car_h_31(double *state, double *unused, double *out_7671597222049239594) {
  h_31(state, unused, out_7671597222049239594);
}
void car_H_31(double *state, double *unused, double *out_8336364149759244087) {
  H_31(state, unused, out_8336364149759244087);
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
