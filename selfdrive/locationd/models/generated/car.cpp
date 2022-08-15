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
void err_fun(double *nom_x, double *delta_x, double *out_5906788765990308544) {
   out_5906788765990308544[0] = delta_x[0] + nom_x[0];
   out_5906788765990308544[1] = delta_x[1] + nom_x[1];
   out_5906788765990308544[2] = delta_x[2] + nom_x[2];
   out_5906788765990308544[3] = delta_x[3] + nom_x[3];
   out_5906788765990308544[4] = delta_x[4] + nom_x[4];
   out_5906788765990308544[5] = delta_x[5] + nom_x[5];
   out_5906788765990308544[6] = delta_x[6] + nom_x[6];
   out_5906788765990308544[7] = delta_x[7] + nom_x[7];
   out_5906788765990308544[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1726895774273068326) {
   out_1726895774273068326[0] = -nom_x[0] + true_x[0];
   out_1726895774273068326[1] = -nom_x[1] + true_x[1];
   out_1726895774273068326[2] = -nom_x[2] + true_x[2];
   out_1726895774273068326[3] = -nom_x[3] + true_x[3];
   out_1726895774273068326[4] = -nom_x[4] + true_x[4];
   out_1726895774273068326[5] = -nom_x[5] + true_x[5];
   out_1726895774273068326[6] = -nom_x[6] + true_x[6];
   out_1726895774273068326[7] = -nom_x[7] + true_x[7];
   out_1726895774273068326[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5542936510909561841) {
   out_5542936510909561841[0] = 1.0;
   out_5542936510909561841[1] = 0;
   out_5542936510909561841[2] = 0;
   out_5542936510909561841[3] = 0;
   out_5542936510909561841[4] = 0;
   out_5542936510909561841[5] = 0;
   out_5542936510909561841[6] = 0;
   out_5542936510909561841[7] = 0;
   out_5542936510909561841[8] = 0;
   out_5542936510909561841[9] = 0;
   out_5542936510909561841[10] = 1.0;
   out_5542936510909561841[11] = 0;
   out_5542936510909561841[12] = 0;
   out_5542936510909561841[13] = 0;
   out_5542936510909561841[14] = 0;
   out_5542936510909561841[15] = 0;
   out_5542936510909561841[16] = 0;
   out_5542936510909561841[17] = 0;
   out_5542936510909561841[18] = 0;
   out_5542936510909561841[19] = 0;
   out_5542936510909561841[20] = 1.0;
   out_5542936510909561841[21] = 0;
   out_5542936510909561841[22] = 0;
   out_5542936510909561841[23] = 0;
   out_5542936510909561841[24] = 0;
   out_5542936510909561841[25] = 0;
   out_5542936510909561841[26] = 0;
   out_5542936510909561841[27] = 0;
   out_5542936510909561841[28] = 0;
   out_5542936510909561841[29] = 0;
   out_5542936510909561841[30] = 1.0;
   out_5542936510909561841[31] = 0;
   out_5542936510909561841[32] = 0;
   out_5542936510909561841[33] = 0;
   out_5542936510909561841[34] = 0;
   out_5542936510909561841[35] = 0;
   out_5542936510909561841[36] = 0;
   out_5542936510909561841[37] = 0;
   out_5542936510909561841[38] = 0;
   out_5542936510909561841[39] = 0;
   out_5542936510909561841[40] = 1.0;
   out_5542936510909561841[41] = 0;
   out_5542936510909561841[42] = 0;
   out_5542936510909561841[43] = 0;
   out_5542936510909561841[44] = 0;
   out_5542936510909561841[45] = 0;
   out_5542936510909561841[46] = 0;
   out_5542936510909561841[47] = 0;
   out_5542936510909561841[48] = 0;
   out_5542936510909561841[49] = 0;
   out_5542936510909561841[50] = 1.0;
   out_5542936510909561841[51] = 0;
   out_5542936510909561841[52] = 0;
   out_5542936510909561841[53] = 0;
   out_5542936510909561841[54] = 0;
   out_5542936510909561841[55] = 0;
   out_5542936510909561841[56] = 0;
   out_5542936510909561841[57] = 0;
   out_5542936510909561841[58] = 0;
   out_5542936510909561841[59] = 0;
   out_5542936510909561841[60] = 1.0;
   out_5542936510909561841[61] = 0;
   out_5542936510909561841[62] = 0;
   out_5542936510909561841[63] = 0;
   out_5542936510909561841[64] = 0;
   out_5542936510909561841[65] = 0;
   out_5542936510909561841[66] = 0;
   out_5542936510909561841[67] = 0;
   out_5542936510909561841[68] = 0;
   out_5542936510909561841[69] = 0;
   out_5542936510909561841[70] = 1.0;
   out_5542936510909561841[71] = 0;
   out_5542936510909561841[72] = 0;
   out_5542936510909561841[73] = 0;
   out_5542936510909561841[74] = 0;
   out_5542936510909561841[75] = 0;
   out_5542936510909561841[76] = 0;
   out_5542936510909561841[77] = 0;
   out_5542936510909561841[78] = 0;
   out_5542936510909561841[79] = 0;
   out_5542936510909561841[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1065943893522122862) {
   out_1065943893522122862[0] = state[0];
   out_1065943893522122862[1] = state[1];
   out_1065943893522122862[2] = state[2];
   out_1065943893522122862[3] = state[3];
   out_1065943893522122862[4] = state[4];
   out_1065943893522122862[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1065943893522122862[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1065943893522122862[7] = state[7];
   out_1065943893522122862[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6663051540238902053) {
   out_6663051540238902053[0] = 1;
   out_6663051540238902053[1] = 0;
   out_6663051540238902053[2] = 0;
   out_6663051540238902053[3] = 0;
   out_6663051540238902053[4] = 0;
   out_6663051540238902053[5] = 0;
   out_6663051540238902053[6] = 0;
   out_6663051540238902053[7] = 0;
   out_6663051540238902053[8] = 0;
   out_6663051540238902053[9] = 0;
   out_6663051540238902053[10] = 1;
   out_6663051540238902053[11] = 0;
   out_6663051540238902053[12] = 0;
   out_6663051540238902053[13] = 0;
   out_6663051540238902053[14] = 0;
   out_6663051540238902053[15] = 0;
   out_6663051540238902053[16] = 0;
   out_6663051540238902053[17] = 0;
   out_6663051540238902053[18] = 0;
   out_6663051540238902053[19] = 0;
   out_6663051540238902053[20] = 1;
   out_6663051540238902053[21] = 0;
   out_6663051540238902053[22] = 0;
   out_6663051540238902053[23] = 0;
   out_6663051540238902053[24] = 0;
   out_6663051540238902053[25] = 0;
   out_6663051540238902053[26] = 0;
   out_6663051540238902053[27] = 0;
   out_6663051540238902053[28] = 0;
   out_6663051540238902053[29] = 0;
   out_6663051540238902053[30] = 1;
   out_6663051540238902053[31] = 0;
   out_6663051540238902053[32] = 0;
   out_6663051540238902053[33] = 0;
   out_6663051540238902053[34] = 0;
   out_6663051540238902053[35] = 0;
   out_6663051540238902053[36] = 0;
   out_6663051540238902053[37] = 0;
   out_6663051540238902053[38] = 0;
   out_6663051540238902053[39] = 0;
   out_6663051540238902053[40] = 1;
   out_6663051540238902053[41] = 0;
   out_6663051540238902053[42] = 0;
   out_6663051540238902053[43] = 0;
   out_6663051540238902053[44] = 0;
   out_6663051540238902053[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6663051540238902053[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6663051540238902053[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6663051540238902053[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6663051540238902053[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6663051540238902053[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6663051540238902053[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6663051540238902053[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6663051540238902053[53] = -9.8000000000000007*dt;
   out_6663051540238902053[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6663051540238902053[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6663051540238902053[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6663051540238902053[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6663051540238902053[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6663051540238902053[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6663051540238902053[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6663051540238902053[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6663051540238902053[62] = 0;
   out_6663051540238902053[63] = 0;
   out_6663051540238902053[64] = 0;
   out_6663051540238902053[65] = 0;
   out_6663051540238902053[66] = 0;
   out_6663051540238902053[67] = 0;
   out_6663051540238902053[68] = 0;
   out_6663051540238902053[69] = 0;
   out_6663051540238902053[70] = 1;
   out_6663051540238902053[71] = 0;
   out_6663051540238902053[72] = 0;
   out_6663051540238902053[73] = 0;
   out_6663051540238902053[74] = 0;
   out_6663051540238902053[75] = 0;
   out_6663051540238902053[76] = 0;
   out_6663051540238902053[77] = 0;
   out_6663051540238902053[78] = 0;
   out_6663051540238902053[79] = 0;
   out_6663051540238902053[80] = 1;
}
void h_25(double *state, double *unused, double *out_4227256557803292572) {
   out_4227256557803292572[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1405888400492708987) {
   out_1405888400492708987[0] = 0;
   out_1405888400492708987[1] = 0;
   out_1405888400492708987[2] = 0;
   out_1405888400492708987[3] = 0;
   out_1405888400492708987[4] = 0;
   out_1405888400492708987[5] = 0;
   out_1405888400492708987[6] = 1;
   out_1405888400492708987[7] = 0;
   out_1405888400492708987[8] = 0;
}
void h_24(double *state, double *unused, double *out_7727802183161571239) {
   out_7727802183161571239[0] = state[4];
   out_7727802183161571239[1] = state[5];
}
void H_24(double *state, double *unused, double *out_565456616310663995) {
   out_565456616310663995[0] = 0;
   out_565456616310663995[1] = 0;
   out_565456616310663995[2] = 0;
   out_565456616310663995[3] = 0;
   out_565456616310663995[4] = 1;
   out_565456616310663995[5] = 0;
   out_565456616310663995[6] = 0;
   out_565456616310663995[7] = 0;
   out_565456616310663995[8] = 0;
   out_565456616310663995[9] = 0;
   out_565456616310663995[10] = 0;
   out_565456616310663995[11] = 0;
   out_565456616310663995[12] = 0;
   out_565456616310663995[13] = 0;
   out_565456616310663995[14] = 1;
   out_565456616310663995[15] = 0;
   out_565456616310663995[16] = 0;
   out_565456616310663995[17] = 0;
}
void h_30(double *state, double *unused, double *out_4502450620087798461) {
   out_4502450620087798461[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3121807929634899211) {
   out_3121807929634899211[0] = 0;
   out_3121807929634899211[1] = 0;
   out_3121807929634899211[2] = 0;
   out_3121807929634899211[3] = 0;
   out_3121807929634899211[4] = 1;
   out_3121807929634899211[5] = 0;
   out_3121807929634899211[6] = 0;
   out_3121807929634899211[7] = 0;
   out_3121807929634899211[8] = 0;
}
void h_26(double *state, double *unused, double *out_853130833811377404) {
   out_853130833811377404[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2335614918381347237) {
   out_2335614918381347237[0] = 0;
   out_2335614918381347237[1] = 0;
   out_2335614918381347237[2] = 0;
   out_2335614918381347237[3] = 0;
   out_2335614918381347237[4] = 0;
   out_2335614918381347237[5] = 0;
   out_2335614918381347237[6] = 0;
   out_2335614918381347237[7] = 1;
   out_2335614918381347237[8] = 0;
}
void h_27(double *state, double *unused, double *out_9217814248245413399) {
   out_9217814248245413399[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5296571241435324122) {
   out_5296571241435324122[0] = 0;
   out_5296571241435324122[1] = 0;
   out_5296571241435324122[2] = 0;
   out_5296571241435324122[3] = 1;
   out_5296571241435324122[4] = 0;
   out_5296571241435324122[5] = 0;
   out_5296571241435324122[6] = 0;
   out_5296571241435324122[7] = 0;
   out_5296571241435324122[8] = 0;
}
void h_29(double *state, double *unused, double *out_5580364719991391049) {
   out_5580364719991391049[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2611576585320507027) {
   out_2611576585320507027[0] = 0;
   out_2611576585320507027[1] = 1;
   out_2611576585320507027[2] = 0;
   out_2611576585320507027[3] = 0;
   out_2611576585320507027[4] = 0;
   out_2611576585320507027[5] = 0;
   out_2611576585320507027[6] = 0;
   out_2611576585320507027[7] = 0;
   out_2611576585320507027[8] = 0;
}
void h_28(double *state, double *unused, double *out_8285419972562475782) {
   out_8285419972562475782[0] = state[0];
}
void H_28(double *state, double *unused, double *out_647946313755180776) {
   out_647946313755180776[0] = 1;
   out_647946313755180776[1] = 0;
   out_647946313755180776[2] = 0;
   out_647946313755180776[3] = 0;
   out_647946313755180776[4] = 0;
   out_647946313755180776[5] = 0;
   out_647946313755180776[6] = 0;
   out_647946313755180776[7] = 0;
   out_647946313755180776[8] = 0;
}
void h_31(double *state, double *unused, double *out_1279530567447807611) {
   out_1279530567447807611[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2961823020614698713) {
   out_2961823020614698713[0] = 0;
   out_2961823020614698713[1] = 0;
   out_2961823020614698713[2] = 0;
   out_2961823020614698713[3] = 0;
   out_2961823020614698713[4] = 0;
   out_2961823020614698713[5] = 0;
   out_2961823020614698713[6] = 0;
   out_2961823020614698713[7] = 0;
   out_2961823020614698713[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5906788765990308544) {
  err_fun(nom_x, delta_x, out_5906788765990308544);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1726895774273068326) {
  inv_err_fun(nom_x, true_x, out_1726895774273068326);
}
void car_H_mod_fun(double *state, double *out_5542936510909561841) {
  H_mod_fun(state, out_5542936510909561841);
}
void car_f_fun(double *state, double dt, double *out_1065943893522122862) {
  f_fun(state,  dt, out_1065943893522122862);
}
void car_F_fun(double *state, double dt, double *out_6663051540238902053) {
  F_fun(state,  dt, out_6663051540238902053);
}
void car_h_25(double *state, double *unused, double *out_4227256557803292572) {
  h_25(state, unused, out_4227256557803292572);
}
void car_H_25(double *state, double *unused, double *out_1405888400492708987) {
  H_25(state, unused, out_1405888400492708987);
}
void car_h_24(double *state, double *unused, double *out_7727802183161571239) {
  h_24(state, unused, out_7727802183161571239);
}
void car_H_24(double *state, double *unused, double *out_565456616310663995) {
  H_24(state, unused, out_565456616310663995);
}
void car_h_30(double *state, double *unused, double *out_4502450620087798461) {
  h_30(state, unused, out_4502450620087798461);
}
void car_H_30(double *state, double *unused, double *out_3121807929634899211) {
  H_30(state, unused, out_3121807929634899211);
}
void car_h_26(double *state, double *unused, double *out_853130833811377404) {
  h_26(state, unused, out_853130833811377404);
}
void car_H_26(double *state, double *unused, double *out_2335614918381347237) {
  H_26(state, unused, out_2335614918381347237);
}
void car_h_27(double *state, double *unused, double *out_9217814248245413399) {
  h_27(state, unused, out_9217814248245413399);
}
void car_H_27(double *state, double *unused, double *out_5296571241435324122) {
  H_27(state, unused, out_5296571241435324122);
}
void car_h_29(double *state, double *unused, double *out_5580364719991391049) {
  h_29(state, unused, out_5580364719991391049);
}
void car_H_29(double *state, double *unused, double *out_2611576585320507027) {
  H_29(state, unused, out_2611576585320507027);
}
void car_h_28(double *state, double *unused, double *out_8285419972562475782) {
  h_28(state, unused, out_8285419972562475782);
}
void car_H_28(double *state, double *unused, double *out_647946313755180776) {
  H_28(state, unused, out_647946313755180776);
}
void car_h_31(double *state, double *unused, double *out_1279530567447807611) {
  h_31(state, unused, out_1279530567447807611);
}
void car_H_31(double *state, double *unused, double *out_2961823020614698713) {
  H_31(state, unused, out_2961823020614698713);
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
