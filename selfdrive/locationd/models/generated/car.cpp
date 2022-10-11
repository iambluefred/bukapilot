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
void err_fun(double *nom_x, double *delta_x, double *out_4005078601906462987) {
   out_4005078601906462987[0] = delta_x[0] + nom_x[0];
   out_4005078601906462987[1] = delta_x[1] + nom_x[1];
   out_4005078601906462987[2] = delta_x[2] + nom_x[2];
   out_4005078601906462987[3] = delta_x[3] + nom_x[3];
   out_4005078601906462987[4] = delta_x[4] + nom_x[4];
   out_4005078601906462987[5] = delta_x[5] + nom_x[5];
   out_4005078601906462987[6] = delta_x[6] + nom_x[6];
   out_4005078601906462987[7] = delta_x[7] + nom_x[7];
   out_4005078601906462987[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6660233397225138441) {
   out_6660233397225138441[0] = -nom_x[0] + true_x[0];
   out_6660233397225138441[1] = -nom_x[1] + true_x[1];
   out_6660233397225138441[2] = -nom_x[2] + true_x[2];
   out_6660233397225138441[3] = -nom_x[3] + true_x[3];
   out_6660233397225138441[4] = -nom_x[4] + true_x[4];
   out_6660233397225138441[5] = -nom_x[5] + true_x[5];
   out_6660233397225138441[6] = -nom_x[6] + true_x[6];
   out_6660233397225138441[7] = -nom_x[7] + true_x[7];
   out_6660233397225138441[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6074328353129280536) {
   out_6074328353129280536[0] = 1.0;
   out_6074328353129280536[1] = 0;
   out_6074328353129280536[2] = 0;
   out_6074328353129280536[3] = 0;
   out_6074328353129280536[4] = 0;
   out_6074328353129280536[5] = 0;
   out_6074328353129280536[6] = 0;
   out_6074328353129280536[7] = 0;
   out_6074328353129280536[8] = 0;
   out_6074328353129280536[9] = 0;
   out_6074328353129280536[10] = 1.0;
   out_6074328353129280536[11] = 0;
   out_6074328353129280536[12] = 0;
   out_6074328353129280536[13] = 0;
   out_6074328353129280536[14] = 0;
   out_6074328353129280536[15] = 0;
   out_6074328353129280536[16] = 0;
   out_6074328353129280536[17] = 0;
   out_6074328353129280536[18] = 0;
   out_6074328353129280536[19] = 0;
   out_6074328353129280536[20] = 1.0;
   out_6074328353129280536[21] = 0;
   out_6074328353129280536[22] = 0;
   out_6074328353129280536[23] = 0;
   out_6074328353129280536[24] = 0;
   out_6074328353129280536[25] = 0;
   out_6074328353129280536[26] = 0;
   out_6074328353129280536[27] = 0;
   out_6074328353129280536[28] = 0;
   out_6074328353129280536[29] = 0;
   out_6074328353129280536[30] = 1.0;
   out_6074328353129280536[31] = 0;
   out_6074328353129280536[32] = 0;
   out_6074328353129280536[33] = 0;
   out_6074328353129280536[34] = 0;
   out_6074328353129280536[35] = 0;
   out_6074328353129280536[36] = 0;
   out_6074328353129280536[37] = 0;
   out_6074328353129280536[38] = 0;
   out_6074328353129280536[39] = 0;
   out_6074328353129280536[40] = 1.0;
   out_6074328353129280536[41] = 0;
   out_6074328353129280536[42] = 0;
   out_6074328353129280536[43] = 0;
   out_6074328353129280536[44] = 0;
   out_6074328353129280536[45] = 0;
   out_6074328353129280536[46] = 0;
   out_6074328353129280536[47] = 0;
   out_6074328353129280536[48] = 0;
   out_6074328353129280536[49] = 0;
   out_6074328353129280536[50] = 1.0;
   out_6074328353129280536[51] = 0;
   out_6074328353129280536[52] = 0;
   out_6074328353129280536[53] = 0;
   out_6074328353129280536[54] = 0;
   out_6074328353129280536[55] = 0;
   out_6074328353129280536[56] = 0;
   out_6074328353129280536[57] = 0;
   out_6074328353129280536[58] = 0;
   out_6074328353129280536[59] = 0;
   out_6074328353129280536[60] = 1.0;
   out_6074328353129280536[61] = 0;
   out_6074328353129280536[62] = 0;
   out_6074328353129280536[63] = 0;
   out_6074328353129280536[64] = 0;
   out_6074328353129280536[65] = 0;
   out_6074328353129280536[66] = 0;
   out_6074328353129280536[67] = 0;
   out_6074328353129280536[68] = 0;
   out_6074328353129280536[69] = 0;
   out_6074328353129280536[70] = 1.0;
   out_6074328353129280536[71] = 0;
   out_6074328353129280536[72] = 0;
   out_6074328353129280536[73] = 0;
   out_6074328353129280536[74] = 0;
   out_6074328353129280536[75] = 0;
   out_6074328353129280536[76] = 0;
   out_6074328353129280536[77] = 0;
   out_6074328353129280536[78] = 0;
   out_6074328353129280536[79] = 0;
   out_6074328353129280536[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1156634792006458643) {
   out_1156634792006458643[0] = state[0];
   out_1156634792006458643[1] = state[1];
   out_1156634792006458643[2] = state[2];
   out_1156634792006458643[3] = state[3];
   out_1156634792006458643[4] = state[4];
   out_1156634792006458643[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1156634792006458643[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1156634792006458643[7] = state[7];
   out_1156634792006458643[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6849697270777504597) {
   out_6849697270777504597[0] = 1;
   out_6849697270777504597[1] = 0;
   out_6849697270777504597[2] = 0;
   out_6849697270777504597[3] = 0;
   out_6849697270777504597[4] = 0;
   out_6849697270777504597[5] = 0;
   out_6849697270777504597[6] = 0;
   out_6849697270777504597[7] = 0;
   out_6849697270777504597[8] = 0;
   out_6849697270777504597[9] = 0;
   out_6849697270777504597[10] = 1;
   out_6849697270777504597[11] = 0;
   out_6849697270777504597[12] = 0;
   out_6849697270777504597[13] = 0;
   out_6849697270777504597[14] = 0;
   out_6849697270777504597[15] = 0;
   out_6849697270777504597[16] = 0;
   out_6849697270777504597[17] = 0;
   out_6849697270777504597[18] = 0;
   out_6849697270777504597[19] = 0;
   out_6849697270777504597[20] = 1;
   out_6849697270777504597[21] = 0;
   out_6849697270777504597[22] = 0;
   out_6849697270777504597[23] = 0;
   out_6849697270777504597[24] = 0;
   out_6849697270777504597[25] = 0;
   out_6849697270777504597[26] = 0;
   out_6849697270777504597[27] = 0;
   out_6849697270777504597[28] = 0;
   out_6849697270777504597[29] = 0;
   out_6849697270777504597[30] = 1;
   out_6849697270777504597[31] = 0;
   out_6849697270777504597[32] = 0;
   out_6849697270777504597[33] = 0;
   out_6849697270777504597[34] = 0;
   out_6849697270777504597[35] = 0;
   out_6849697270777504597[36] = 0;
   out_6849697270777504597[37] = 0;
   out_6849697270777504597[38] = 0;
   out_6849697270777504597[39] = 0;
   out_6849697270777504597[40] = 1;
   out_6849697270777504597[41] = 0;
   out_6849697270777504597[42] = 0;
   out_6849697270777504597[43] = 0;
   out_6849697270777504597[44] = 0;
   out_6849697270777504597[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6849697270777504597[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6849697270777504597[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6849697270777504597[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6849697270777504597[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6849697270777504597[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6849697270777504597[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6849697270777504597[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6849697270777504597[53] = -9.8000000000000007*dt;
   out_6849697270777504597[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6849697270777504597[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6849697270777504597[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6849697270777504597[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6849697270777504597[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6849697270777504597[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6849697270777504597[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6849697270777504597[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6849697270777504597[62] = 0;
   out_6849697270777504597[63] = 0;
   out_6849697270777504597[64] = 0;
   out_6849697270777504597[65] = 0;
   out_6849697270777504597[66] = 0;
   out_6849697270777504597[67] = 0;
   out_6849697270777504597[68] = 0;
   out_6849697270777504597[69] = 0;
   out_6849697270777504597[70] = 1;
   out_6849697270777504597[71] = 0;
   out_6849697270777504597[72] = 0;
   out_6849697270777504597[73] = 0;
   out_6849697270777504597[74] = 0;
   out_6849697270777504597[75] = 0;
   out_6849697270777504597[76] = 0;
   out_6849697270777504597[77] = 0;
   out_6849697270777504597[78] = 0;
   out_6849697270777504597[79] = 0;
   out_6849697270777504597[80] = 1;
}
void h_25(double *state, double *unused, double *out_8290085571197364282) {
   out_8290085571197364282[0] = state[6];
}
void H_25(double *state, double *unused, double *out_690624191224379626) {
   out_690624191224379626[0] = 0;
   out_690624191224379626[1] = 0;
   out_690624191224379626[2] = 0;
   out_690624191224379626[3] = 0;
   out_690624191224379626[4] = 0;
   out_690624191224379626[5] = 0;
   out_690624191224379626[6] = 1;
   out_690624191224379626[7] = 0;
   out_690624191224379626[8] = 0;
}
void h_24(double *state, double *unused, double *out_2505692210727262820) {
   out_2505692210727262820[0] = state[4];
   out_2505692210727262820[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1087269495578239121) {
   out_1087269495578239121[0] = 0;
   out_1087269495578239121[1] = 0;
   out_1087269495578239121[2] = 0;
   out_1087269495578239121[3] = 0;
   out_1087269495578239121[4] = 1;
   out_1087269495578239121[5] = 0;
   out_1087269495578239121[6] = 0;
   out_1087269495578239121[7] = 0;
   out_1087269495578239121[8] = 0;
   out_1087269495578239121[9] = 0;
   out_1087269495578239121[10] = 0;
   out_1087269495578239121[11] = 0;
   out_1087269495578239121[12] = 0;
   out_1087269495578239121[13] = 0;
   out_1087269495578239121[14] = 1;
   out_1087269495578239121[15] = 0;
   out_1087269495578239121[16] = 0;
   out_1087269495578239121[17] = 0;
}
void h_30(double *state, double *unused, double *out_6963440463520399393) {
   out_6963440463520399393[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1827708767282869001) {
   out_1827708767282869001[0] = 0;
   out_1827708767282869001[1] = 0;
   out_1827708767282869001[2] = 0;
   out_1827708767282869001[3] = 0;
   out_1827708767282869001[4] = 1;
   out_1827708767282869001[5] = 0;
   out_1827708767282869001[6] = 0;
   out_1827708767282869001[7] = 0;
   out_1827708767282869001[8] = 0;
}
void h_26(double *state, double *unused, double *out_2686994145923155083) {
   out_2686994145923155083[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4432127510098435850) {
   out_4432127510098435850[0] = 0;
   out_4432127510098435850[1] = 0;
   out_4432127510098435850[2] = 0;
   out_4432127510098435850[3] = 0;
   out_4432127510098435850[4] = 0;
   out_4432127510098435850[5] = 0;
   out_4432127510098435850[6] = 0;
   out_4432127510098435850[7] = 1;
   out_4432127510098435850[8] = 0;
}
void h_27(double *state, double *unused, double *out_5534472473056079020) {
   out_5534472473056079020[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4051302838466812218) {
   out_4051302838466812218[0] = 0;
   out_4051302838466812218[1] = 0;
   out_4051302838466812218[2] = 0;
   out_4051302838466812218[3] = 1;
   out_4051302838466812218[4] = 0;
   out_4051302838466812218[5] = 0;
   out_4051302838466812218[6] = 0;
   out_4051302838466812218[7] = 0;
   out_4051302838466812218[8] = 0;
}
void h_29(double *state, double *unused, double *out_1897022944802056670) {
   out_1897022944802056670[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2337940111597261185) {
   out_2337940111597261185[0] = 0;
   out_2337940111597261185[1] = 1;
   out_2337940111597261185[2] = 0;
   out_2337940111597261185[3] = 0;
   out_2337940111597261185[4] = 0;
   out_2337940111597261185[5] = 0;
   out_2337940111597261185[6] = 0;
   out_2337940111597261185[7] = 0;
   out_2337940111597261185[8] = 0;
}
void h_28(double *state, double *unused, double *out_532799205395233375) {
   out_532799205395233375[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2744458905472269389) {
   out_2744458905472269389[0] = 1;
   out_2744458905472269389[1] = 0;
   out_2744458905472269389[2] = 0;
   out_2744458905472269389[3] = 0;
   out_2744458905472269389[4] = 0;
   out_2744458905472269389[5] = 0;
   out_2744458905472269389[6] = 0;
   out_2744458905472269389[7] = 0;
   out_2744458905472269389[8] = 0;
}
void h_31(double *state, double *unused, double *out_8447272239548493427) {
   out_8447272239548493427[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5058335612331787326) {
   out_5058335612331787326[0] = 0;
   out_5058335612331787326[1] = 0;
   out_5058335612331787326[2] = 0;
   out_5058335612331787326[3] = 0;
   out_5058335612331787326[4] = 0;
   out_5058335612331787326[5] = 0;
   out_5058335612331787326[6] = 0;
   out_5058335612331787326[7] = 0;
   out_5058335612331787326[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4005078601906462987) {
  err_fun(nom_x, delta_x, out_4005078601906462987);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6660233397225138441) {
  inv_err_fun(nom_x, true_x, out_6660233397225138441);
}
void car_H_mod_fun(double *state, double *out_6074328353129280536) {
  H_mod_fun(state, out_6074328353129280536);
}
void car_f_fun(double *state, double dt, double *out_1156634792006458643) {
  f_fun(state,  dt, out_1156634792006458643);
}
void car_F_fun(double *state, double dt, double *out_6849697270777504597) {
  F_fun(state,  dt, out_6849697270777504597);
}
void car_h_25(double *state, double *unused, double *out_8290085571197364282) {
  h_25(state, unused, out_8290085571197364282);
}
void car_H_25(double *state, double *unused, double *out_690624191224379626) {
  H_25(state, unused, out_690624191224379626);
}
void car_h_24(double *state, double *unused, double *out_2505692210727262820) {
  h_24(state, unused, out_2505692210727262820);
}
void car_H_24(double *state, double *unused, double *out_1087269495578239121) {
  H_24(state, unused, out_1087269495578239121);
}
void car_h_30(double *state, double *unused, double *out_6963440463520399393) {
  h_30(state, unused, out_6963440463520399393);
}
void car_H_30(double *state, double *unused, double *out_1827708767282869001) {
  H_30(state, unused, out_1827708767282869001);
}
void car_h_26(double *state, double *unused, double *out_2686994145923155083) {
  h_26(state, unused, out_2686994145923155083);
}
void car_H_26(double *state, double *unused, double *out_4432127510098435850) {
  H_26(state, unused, out_4432127510098435850);
}
void car_h_27(double *state, double *unused, double *out_5534472473056079020) {
  h_27(state, unused, out_5534472473056079020);
}
void car_H_27(double *state, double *unused, double *out_4051302838466812218) {
  H_27(state, unused, out_4051302838466812218);
}
void car_h_29(double *state, double *unused, double *out_1897022944802056670) {
  h_29(state, unused, out_1897022944802056670);
}
void car_H_29(double *state, double *unused, double *out_2337940111597261185) {
  H_29(state, unused, out_2337940111597261185);
}
void car_h_28(double *state, double *unused, double *out_532799205395233375) {
  h_28(state, unused, out_532799205395233375);
}
void car_H_28(double *state, double *unused, double *out_2744458905472269389) {
  H_28(state, unused, out_2744458905472269389);
}
void car_h_31(double *state, double *unused, double *out_8447272239548493427) {
  h_31(state, unused, out_8447272239548493427);
}
void car_H_31(double *state, double *unused, double *out_5058335612331787326) {
  H_31(state, unused, out_5058335612331787326);
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
