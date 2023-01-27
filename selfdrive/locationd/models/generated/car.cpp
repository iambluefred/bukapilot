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
void err_fun(double *nom_x, double *delta_x, double *out_5647470276735826301) {
   out_5647470276735826301[0] = delta_x[0] + nom_x[0];
   out_5647470276735826301[1] = delta_x[1] + nom_x[1];
   out_5647470276735826301[2] = delta_x[2] + nom_x[2];
   out_5647470276735826301[3] = delta_x[3] + nom_x[3];
   out_5647470276735826301[4] = delta_x[4] + nom_x[4];
   out_5647470276735826301[5] = delta_x[5] + nom_x[5];
   out_5647470276735826301[6] = delta_x[6] + nom_x[6];
   out_5647470276735826301[7] = delta_x[7] + nom_x[7];
   out_5647470276735826301[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1373347932731707161) {
   out_1373347932731707161[0] = -nom_x[0] + true_x[0];
   out_1373347932731707161[1] = -nom_x[1] + true_x[1];
   out_1373347932731707161[2] = -nom_x[2] + true_x[2];
   out_1373347932731707161[3] = -nom_x[3] + true_x[3];
   out_1373347932731707161[4] = -nom_x[4] + true_x[4];
   out_1373347932731707161[5] = -nom_x[5] + true_x[5];
   out_1373347932731707161[6] = -nom_x[6] + true_x[6];
   out_1373347932731707161[7] = -nom_x[7] + true_x[7];
   out_1373347932731707161[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3198577004452224537) {
   out_3198577004452224537[0] = 1.0;
   out_3198577004452224537[1] = 0;
   out_3198577004452224537[2] = 0;
   out_3198577004452224537[3] = 0;
   out_3198577004452224537[4] = 0;
   out_3198577004452224537[5] = 0;
   out_3198577004452224537[6] = 0;
   out_3198577004452224537[7] = 0;
   out_3198577004452224537[8] = 0;
   out_3198577004452224537[9] = 0;
   out_3198577004452224537[10] = 1.0;
   out_3198577004452224537[11] = 0;
   out_3198577004452224537[12] = 0;
   out_3198577004452224537[13] = 0;
   out_3198577004452224537[14] = 0;
   out_3198577004452224537[15] = 0;
   out_3198577004452224537[16] = 0;
   out_3198577004452224537[17] = 0;
   out_3198577004452224537[18] = 0;
   out_3198577004452224537[19] = 0;
   out_3198577004452224537[20] = 1.0;
   out_3198577004452224537[21] = 0;
   out_3198577004452224537[22] = 0;
   out_3198577004452224537[23] = 0;
   out_3198577004452224537[24] = 0;
   out_3198577004452224537[25] = 0;
   out_3198577004452224537[26] = 0;
   out_3198577004452224537[27] = 0;
   out_3198577004452224537[28] = 0;
   out_3198577004452224537[29] = 0;
   out_3198577004452224537[30] = 1.0;
   out_3198577004452224537[31] = 0;
   out_3198577004452224537[32] = 0;
   out_3198577004452224537[33] = 0;
   out_3198577004452224537[34] = 0;
   out_3198577004452224537[35] = 0;
   out_3198577004452224537[36] = 0;
   out_3198577004452224537[37] = 0;
   out_3198577004452224537[38] = 0;
   out_3198577004452224537[39] = 0;
   out_3198577004452224537[40] = 1.0;
   out_3198577004452224537[41] = 0;
   out_3198577004452224537[42] = 0;
   out_3198577004452224537[43] = 0;
   out_3198577004452224537[44] = 0;
   out_3198577004452224537[45] = 0;
   out_3198577004452224537[46] = 0;
   out_3198577004452224537[47] = 0;
   out_3198577004452224537[48] = 0;
   out_3198577004452224537[49] = 0;
   out_3198577004452224537[50] = 1.0;
   out_3198577004452224537[51] = 0;
   out_3198577004452224537[52] = 0;
   out_3198577004452224537[53] = 0;
   out_3198577004452224537[54] = 0;
   out_3198577004452224537[55] = 0;
   out_3198577004452224537[56] = 0;
   out_3198577004452224537[57] = 0;
   out_3198577004452224537[58] = 0;
   out_3198577004452224537[59] = 0;
   out_3198577004452224537[60] = 1.0;
   out_3198577004452224537[61] = 0;
   out_3198577004452224537[62] = 0;
   out_3198577004452224537[63] = 0;
   out_3198577004452224537[64] = 0;
   out_3198577004452224537[65] = 0;
   out_3198577004452224537[66] = 0;
   out_3198577004452224537[67] = 0;
   out_3198577004452224537[68] = 0;
   out_3198577004452224537[69] = 0;
   out_3198577004452224537[70] = 1.0;
   out_3198577004452224537[71] = 0;
   out_3198577004452224537[72] = 0;
   out_3198577004452224537[73] = 0;
   out_3198577004452224537[74] = 0;
   out_3198577004452224537[75] = 0;
   out_3198577004452224537[76] = 0;
   out_3198577004452224537[77] = 0;
   out_3198577004452224537[78] = 0;
   out_3198577004452224537[79] = 0;
   out_3198577004452224537[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2545790947865816023) {
   out_2545790947865816023[0] = state[0];
   out_2545790947865816023[1] = state[1];
   out_2545790947865816023[2] = state[2];
   out_2545790947865816023[3] = state[3];
   out_2545790947865816023[4] = state[4];
   out_2545790947865816023[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2545790947865816023[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2545790947865816023[7] = state[7];
   out_2545790947865816023[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1912985182909371113) {
   out_1912985182909371113[0] = 1;
   out_1912985182909371113[1] = 0;
   out_1912985182909371113[2] = 0;
   out_1912985182909371113[3] = 0;
   out_1912985182909371113[4] = 0;
   out_1912985182909371113[5] = 0;
   out_1912985182909371113[6] = 0;
   out_1912985182909371113[7] = 0;
   out_1912985182909371113[8] = 0;
   out_1912985182909371113[9] = 0;
   out_1912985182909371113[10] = 1;
   out_1912985182909371113[11] = 0;
   out_1912985182909371113[12] = 0;
   out_1912985182909371113[13] = 0;
   out_1912985182909371113[14] = 0;
   out_1912985182909371113[15] = 0;
   out_1912985182909371113[16] = 0;
   out_1912985182909371113[17] = 0;
   out_1912985182909371113[18] = 0;
   out_1912985182909371113[19] = 0;
   out_1912985182909371113[20] = 1;
   out_1912985182909371113[21] = 0;
   out_1912985182909371113[22] = 0;
   out_1912985182909371113[23] = 0;
   out_1912985182909371113[24] = 0;
   out_1912985182909371113[25] = 0;
   out_1912985182909371113[26] = 0;
   out_1912985182909371113[27] = 0;
   out_1912985182909371113[28] = 0;
   out_1912985182909371113[29] = 0;
   out_1912985182909371113[30] = 1;
   out_1912985182909371113[31] = 0;
   out_1912985182909371113[32] = 0;
   out_1912985182909371113[33] = 0;
   out_1912985182909371113[34] = 0;
   out_1912985182909371113[35] = 0;
   out_1912985182909371113[36] = 0;
   out_1912985182909371113[37] = 0;
   out_1912985182909371113[38] = 0;
   out_1912985182909371113[39] = 0;
   out_1912985182909371113[40] = 1;
   out_1912985182909371113[41] = 0;
   out_1912985182909371113[42] = 0;
   out_1912985182909371113[43] = 0;
   out_1912985182909371113[44] = 0;
   out_1912985182909371113[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1912985182909371113[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1912985182909371113[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1912985182909371113[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1912985182909371113[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1912985182909371113[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1912985182909371113[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1912985182909371113[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1912985182909371113[53] = -9.8000000000000007*dt;
   out_1912985182909371113[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1912985182909371113[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1912985182909371113[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1912985182909371113[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1912985182909371113[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1912985182909371113[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1912985182909371113[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1912985182909371113[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1912985182909371113[62] = 0;
   out_1912985182909371113[63] = 0;
   out_1912985182909371113[64] = 0;
   out_1912985182909371113[65] = 0;
   out_1912985182909371113[66] = 0;
   out_1912985182909371113[67] = 0;
   out_1912985182909371113[68] = 0;
   out_1912985182909371113[69] = 0;
   out_1912985182909371113[70] = 1;
   out_1912985182909371113[71] = 0;
   out_1912985182909371113[72] = 0;
   out_1912985182909371113[73] = 0;
   out_1912985182909371113[74] = 0;
   out_1912985182909371113[75] = 0;
   out_1912985182909371113[76] = 0;
   out_1912985182909371113[77] = 0;
   out_1912985182909371113[78] = 0;
   out_1912985182909371113[79] = 0;
   out_1912985182909371113[80] = 1;
}
void h_25(double *state, double *unused, double *out_4456422403562273665) {
   out_4456422403562273665[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4929918955097233612) {
   out_4929918955097233612[0] = 0;
   out_4929918955097233612[1] = 0;
   out_4929918955097233612[2] = 0;
   out_4929918955097233612[3] = 0;
   out_4929918955097233612[4] = 0;
   out_4929918955097233612[5] = 0;
   out_4929918955097233612[6] = 1;
   out_4929918955097233612[7] = 0;
   out_4929918955097233612[8] = 0;
}
void h_24(double *state, double *unused, double *out_248722503563105437) {
   out_248722503563105437[0] = state[4];
   out_248722503563105437[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7102568554102733178) {
   out_7102568554102733178[0] = 0;
   out_7102568554102733178[1] = 0;
   out_7102568554102733178[2] = 0;
   out_7102568554102733178[3] = 0;
   out_7102568554102733178[4] = 1;
   out_7102568554102733178[5] = 0;
   out_7102568554102733178[6] = 0;
   out_7102568554102733178[7] = 0;
   out_7102568554102733178[8] = 0;
   out_7102568554102733178[9] = 0;
   out_7102568554102733178[10] = 0;
   out_7102568554102733178[11] = 0;
   out_7102568554102733178[12] = 0;
   out_7102568554102733178[13] = 0;
   out_7102568554102733178[14] = 1;
   out_7102568554102733178[15] = 0;
   out_7102568554102733178[16] = 0;
   out_7102568554102733178[17] = 0;
}
void h_30(double *state, double *unused, double *out_1907171574739542196) {
   out_1907171574739542196[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1986771386394383143) {
   out_1986771386394383143[0] = 0;
   out_1986771386394383143[1] = 0;
   out_1986771386394383143[2] = 0;
   out_1986771386394383143[3] = 0;
   out_1986771386394383143[4] = 1;
   out_1986771386394383143[5] = 0;
   out_1986771386394383143[6] = 0;
   out_1986771386394383143[7] = 0;
   out_1986771386394383143[8] = 0;
}
void h_26(double *state, double *unused, double *out_2490780506542086816) {
   out_2490780506542086816[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8671422273971289836) {
   out_8671422273971289836[0] = 0;
   out_8671422273971289836[1] = 0;
   out_8671422273971289836[2] = 0;
   out_8671422273971289836[3] = 0;
   out_8671422273971289836[4] = 0;
   out_8671422273971289836[5] = 0;
   out_8671422273971289836[6] = 0;
   out_8671422273971289836[7] = 1;
   out_8671422273971289836[8] = 0;
}
void h_27(double *state, double *unused, double *out_7734496948188230697) {
   out_7734496948188230697[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7234021214040898593) {
   out_7234021214040898593[0] = 0;
   out_7234021214040898593[1] = 0;
   out_7234021214040898593[2] = 0;
   out_7234021214040898593[3] = 1;
   out_7234021214040898593[4] = 0;
   out_7234021214040898593[5] = 0;
   out_7234021214040898593[6] = 0;
   out_7234021214040898593[7] = 0;
   out_7234021214040898593[8] = 0;
}
void h_29(double *state, double *unused, double *out_905380528530311187) {
   out_905380528530311187[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8947383940910449626) {
   out_8947383940910449626[0] = 0;
   out_8947383940910449626[1] = 1;
   out_8947383940910449626[2] = 0;
   out_8947383940910449626[3] = 0;
   out_8947383940910449626[4] = 0;
   out_8947383940910449626[5] = 0;
   out_8947383940910449626[6] = 0;
   out_8947383940910449626[7] = 0;
   out_8947383940910449626[8] = 0;
}
void h_28(double *state, double *unused, double *out_5710573857860475274) {
   out_5710573857860475274[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6983753669345123375) {
   out_6983753669345123375[0] = 1;
   out_6983753669345123375[1] = 0;
   out_6983753669345123375[2] = 0;
   out_6983753669345123375[3] = 0;
   out_6983753669345123375[4] = 0;
   out_6983753669345123375[5] = 0;
   out_6983753669345123375[6] = 0;
   out_6983753669345123375[7] = 0;
   out_6983753669345123375[8] = 0;
}
void h_31(double *state, double *unused, double *out_3278691893586295999) {
   out_3278691893586295999[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4899272993220273184) {
   out_4899272993220273184[0] = 0;
   out_4899272993220273184[1] = 0;
   out_4899272993220273184[2] = 0;
   out_4899272993220273184[3] = 0;
   out_4899272993220273184[4] = 0;
   out_4899272993220273184[5] = 0;
   out_4899272993220273184[6] = 0;
   out_4899272993220273184[7] = 0;
   out_4899272993220273184[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5647470276735826301) {
  err_fun(nom_x, delta_x, out_5647470276735826301);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1373347932731707161) {
  inv_err_fun(nom_x, true_x, out_1373347932731707161);
}
void car_H_mod_fun(double *state, double *out_3198577004452224537) {
  H_mod_fun(state, out_3198577004452224537);
}
void car_f_fun(double *state, double dt, double *out_2545790947865816023) {
  f_fun(state,  dt, out_2545790947865816023);
}
void car_F_fun(double *state, double dt, double *out_1912985182909371113) {
  F_fun(state,  dt, out_1912985182909371113);
}
void car_h_25(double *state, double *unused, double *out_4456422403562273665) {
  h_25(state, unused, out_4456422403562273665);
}
void car_H_25(double *state, double *unused, double *out_4929918955097233612) {
  H_25(state, unused, out_4929918955097233612);
}
void car_h_24(double *state, double *unused, double *out_248722503563105437) {
  h_24(state, unused, out_248722503563105437);
}
void car_H_24(double *state, double *unused, double *out_7102568554102733178) {
  H_24(state, unused, out_7102568554102733178);
}
void car_h_30(double *state, double *unused, double *out_1907171574739542196) {
  h_30(state, unused, out_1907171574739542196);
}
void car_H_30(double *state, double *unused, double *out_1986771386394383143) {
  H_30(state, unused, out_1986771386394383143);
}
void car_h_26(double *state, double *unused, double *out_2490780506542086816) {
  h_26(state, unused, out_2490780506542086816);
}
void car_H_26(double *state, double *unused, double *out_8671422273971289836) {
  H_26(state, unused, out_8671422273971289836);
}
void car_h_27(double *state, double *unused, double *out_7734496948188230697) {
  h_27(state, unused, out_7734496948188230697);
}
void car_H_27(double *state, double *unused, double *out_7234021214040898593) {
  H_27(state, unused, out_7234021214040898593);
}
void car_h_29(double *state, double *unused, double *out_905380528530311187) {
  h_29(state, unused, out_905380528530311187);
}
void car_H_29(double *state, double *unused, double *out_8947383940910449626) {
  H_29(state, unused, out_8947383940910449626);
}
void car_h_28(double *state, double *unused, double *out_5710573857860475274) {
  h_28(state, unused, out_5710573857860475274);
}
void car_H_28(double *state, double *unused, double *out_6983753669345123375) {
  H_28(state, unused, out_6983753669345123375);
}
void car_h_31(double *state, double *unused, double *out_3278691893586295999) {
  h_31(state, unused, out_3278691893586295999);
}
void car_H_31(double *state, double *unused, double *out_4899272993220273184) {
  H_31(state, unused, out_4899272993220273184);
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
