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
void err_fun(double *nom_x, double *delta_x, double *out_6707971512290491444) {
   out_6707971512290491444[0] = delta_x[0] + nom_x[0];
   out_6707971512290491444[1] = delta_x[1] + nom_x[1];
   out_6707971512290491444[2] = delta_x[2] + nom_x[2];
   out_6707971512290491444[3] = delta_x[3] + nom_x[3];
   out_6707971512290491444[4] = delta_x[4] + nom_x[4];
   out_6707971512290491444[5] = delta_x[5] + nom_x[5];
   out_6707971512290491444[6] = delta_x[6] + nom_x[6];
   out_6707971512290491444[7] = delta_x[7] + nom_x[7];
   out_6707971512290491444[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7853994834193184287) {
   out_7853994834193184287[0] = -nom_x[0] + true_x[0];
   out_7853994834193184287[1] = -nom_x[1] + true_x[1];
   out_7853994834193184287[2] = -nom_x[2] + true_x[2];
   out_7853994834193184287[3] = -nom_x[3] + true_x[3];
   out_7853994834193184287[4] = -nom_x[4] + true_x[4];
   out_7853994834193184287[5] = -nom_x[5] + true_x[5];
   out_7853994834193184287[6] = -nom_x[6] + true_x[6];
   out_7853994834193184287[7] = -nom_x[7] + true_x[7];
   out_7853994834193184287[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4010081315495760417) {
   out_4010081315495760417[0] = 1.0;
   out_4010081315495760417[1] = 0;
   out_4010081315495760417[2] = 0;
   out_4010081315495760417[3] = 0;
   out_4010081315495760417[4] = 0;
   out_4010081315495760417[5] = 0;
   out_4010081315495760417[6] = 0;
   out_4010081315495760417[7] = 0;
   out_4010081315495760417[8] = 0;
   out_4010081315495760417[9] = 0;
   out_4010081315495760417[10] = 1.0;
   out_4010081315495760417[11] = 0;
   out_4010081315495760417[12] = 0;
   out_4010081315495760417[13] = 0;
   out_4010081315495760417[14] = 0;
   out_4010081315495760417[15] = 0;
   out_4010081315495760417[16] = 0;
   out_4010081315495760417[17] = 0;
   out_4010081315495760417[18] = 0;
   out_4010081315495760417[19] = 0;
   out_4010081315495760417[20] = 1.0;
   out_4010081315495760417[21] = 0;
   out_4010081315495760417[22] = 0;
   out_4010081315495760417[23] = 0;
   out_4010081315495760417[24] = 0;
   out_4010081315495760417[25] = 0;
   out_4010081315495760417[26] = 0;
   out_4010081315495760417[27] = 0;
   out_4010081315495760417[28] = 0;
   out_4010081315495760417[29] = 0;
   out_4010081315495760417[30] = 1.0;
   out_4010081315495760417[31] = 0;
   out_4010081315495760417[32] = 0;
   out_4010081315495760417[33] = 0;
   out_4010081315495760417[34] = 0;
   out_4010081315495760417[35] = 0;
   out_4010081315495760417[36] = 0;
   out_4010081315495760417[37] = 0;
   out_4010081315495760417[38] = 0;
   out_4010081315495760417[39] = 0;
   out_4010081315495760417[40] = 1.0;
   out_4010081315495760417[41] = 0;
   out_4010081315495760417[42] = 0;
   out_4010081315495760417[43] = 0;
   out_4010081315495760417[44] = 0;
   out_4010081315495760417[45] = 0;
   out_4010081315495760417[46] = 0;
   out_4010081315495760417[47] = 0;
   out_4010081315495760417[48] = 0;
   out_4010081315495760417[49] = 0;
   out_4010081315495760417[50] = 1.0;
   out_4010081315495760417[51] = 0;
   out_4010081315495760417[52] = 0;
   out_4010081315495760417[53] = 0;
   out_4010081315495760417[54] = 0;
   out_4010081315495760417[55] = 0;
   out_4010081315495760417[56] = 0;
   out_4010081315495760417[57] = 0;
   out_4010081315495760417[58] = 0;
   out_4010081315495760417[59] = 0;
   out_4010081315495760417[60] = 1.0;
   out_4010081315495760417[61] = 0;
   out_4010081315495760417[62] = 0;
   out_4010081315495760417[63] = 0;
   out_4010081315495760417[64] = 0;
   out_4010081315495760417[65] = 0;
   out_4010081315495760417[66] = 0;
   out_4010081315495760417[67] = 0;
   out_4010081315495760417[68] = 0;
   out_4010081315495760417[69] = 0;
   out_4010081315495760417[70] = 1.0;
   out_4010081315495760417[71] = 0;
   out_4010081315495760417[72] = 0;
   out_4010081315495760417[73] = 0;
   out_4010081315495760417[74] = 0;
   out_4010081315495760417[75] = 0;
   out_4010081315495760417[76] = 0;
   out_4010081315495760417[77] = 0;
   out_4010081315495760417[78] = 0;
   out_4010081315495760417[79] = 0;
   out_4010081315495760417[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4685496284796261050) {
   out_4685496284796261050[0] = state[0];
   out_4685496284796261050[1] = state[1];
   out_4685496284796261050[2] = state[2];
   out_4685496284796261050[3] = state[3];
   out_4685496284796261050[4] = state[4];
   out_4685496284796261050[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4685496284796261050[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4685496284796261050[7] = state[7];
   out_4685496284796261050[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8622935208793728795) {
   out_8622935208793728795[0] = 1;
   out_8622935208793728795[1] = 0;
   out_8622935208793728795[2] = 0;
   out_8622935208793728795[3] = 0;
   out_8622935208793728795[4] = 0;
   out_8622935208793728795[5] = 0;
   out_8622935208793728795[6] = 0;
   out_8622935208793728795[7] = 0;
   out_8622935208793728795[8] = 0;
   out_8622935208793728795[9] = 0;
   out_8622935208793728795[10] = 1;
   out_8622935208793728795[11] = 0;
   out_8622935208793728795[12] = 0;
   out_8622935208793728795[13] = 0;
   out_8622935208793728795[14] = 0;
   out_8622935208793728795[15] = 0;
   out_8622935208793728795[16] = 0;
   out_8622935208793728795[17] = 0;
   out_8622935208793728795[18] = 0;
   out_8622935208793728795[19] = 0;
   out_8622935208793728795[20] = 1;
   out_8622935208793728795[21] = 0;
   out_8622935208793728795[22] = 0;
   out_8622935208793728795[23] = 0;
   out_8622935208793728795[24] = 0;
   out_8622935208793728795[25] = 0;
   out_8622935208793728795[26] = 0;
   out_8622935208793728795[27] = 0;
   out_8622935208793728795[28] = 0;
   out_8622935208793728795[29] = 0;
   out_8622935208793728795[30] = 1;
   out_8622935208793728795[31] = 0;
   out_8622935208793728795[32] = 0;
   out_8622935208793728795[33] = 0;
   out_8622935208793728795[34] = 0;
   out_8622935208793728795[35] = 0;
   out_8622935208793728795[36] = 0;
   out_8622935208793728795[37] = 0;
   out_8622935208793728795[38] = 0;
   out_8622935208793728795[39] = 0;
   out_8622935208793728795[40] = 1;
   out_8622935208793728795[41] = 0;
   out_8622935208793728795[42] = 0;
   out_8622935208793728795[43] = 0;
   out_8622935208793728795[44] = 0;
   out_8622935208793728795[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8622935208793728795[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8622935208793728795[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8622935208793728795[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8622935208793728795[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8622935208793728795[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8622935208793728795[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8622935208793728795[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8622935208793728795[53] = -9.8000000000000007*dt;
   out_8622935208793728795[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8622935208793728795[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8622935208793728795[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8622935208793728795[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8622935208793728795[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8622935208793728795[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8622935208793728795[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8622935208793728795[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8622935208793728795[62] = 0;
   out_8622935208793728795[63] = 0;
   out_8622935208793728795[64] = 0;
   out_8622935208793728795[65] = 0;
   out_8622935208793728795[66] = 0;
   out_8622935208793728795[67] = 0;
   out_8622935208793728795[68] = 0;
   out_8622935208793728795[69] = 0;
   out_8622935208793728795[70] = 1;
   out_8622935208793728795[71] = 0;
   out_8622935208793728795[72] = 0;
   out_8622935208793728795[73] = 0;
   out_8622935208793728795[74] = 0;
   out_8622935208793728795[75] = 0;
   out_8622935208793728795[76] = 0;
   out_8622935208793728795[77] = 0;
   out_8622935208793728795[78] = 0;
   out_8622935208793728795[79] = 0;
   out_8622935208793728795[80] = 1;
}
void h_25(double *state, double *unused, double *out_1413316984438352955) {
   out_1413316984438352955[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7359072756142844022) {
   out_7359072756142844022[0] = 0;
   out_7359072756142844022[1] = 0;
   out_7359072756142844022[2] = 0;
   out_7359072756142844022[3] = 0;
   out_7359072756142844022[4] = 0;
   out_7359072756142844022[5] = 0;
   out_7359072756142844022[6] = 1;
   out_7359072756142844022[7] = 0;
   out_7359072756142844022[8] = 0;
}
void h_24(double *state, double *unused, double *out_1193875120514303785) {
   out_1193875120514303785[0] = state[4];
   out_1193875120514303785[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6341161990600198560) {
   out_6341161990600198560[0] = 0;
   out_6341161990600198560[1] = 0;
   out_6341161990600198560[2] = 0;
   out_6341161990600198560[3] = 0;
   out_6341161990600198560[4] = 1;
   out_6341161990600198560[5] = 0;
   out_6341161990600198560[6] = 0;
   out_6341161990600198560[7] = 0;
   out_6341161990600198560[8] = 0;
   out_6341161990600198560[9] = 0;
   out_6341161990600198560[10] = 0;
   out_6341161990600198560[11] = 0;
   out_6341161990600198560[12] = 0;
   out_6341161990600198560[13] = 0;
   out_6341161990600198560[14] = 1;
   out_6341161990600198560[15] = 0;
   out_6341161990600198560[16] = 0;
   out_6341161990600198560[17] = 0;
}
void h_30(double *state, double *unused, double *out_1256130316087223810) {
   out_1256130316087223810[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4840739797635595395) {
   out_4840739797635595395[0] = 0;
   out_4840739797635595395[1] = 0;
   out_4840739797635595395[2] = 0;
   out_4840739797635595395[3] = 0;
   out_4840739797635595395[4] = 1;
   out_4840739797635595395[5] = 0;
   out_4840739797635595395[6] = 0;
   out_4840739797635595395[7] = 0;
   out_4840739797635595395[8] = 0;
}
void h_26(double *state, double *unused, double *out_2944996651036528421) {
   out_2944996651036528421[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4054546786382043421) {
   out_4054546786382043421[0] = 0;
   out_4054546786382043421[1] = 0;
   out_4054546786382043421[2] = 0;
   out_4054546786382043421[3] = 0;
   out_4054546786382043421[4] = 0;
   out_4054546786382043421[5] = 0;
   out_4054546786382043421[6] = 0;
   out_4054546786382043421[7] = 1;
   out_4054546786382043421[8] = 0;
}
void h_27(double *state, double *unused, double *out_6992339835205873698) {
   out_6992339835205873698[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7015503109436020306) {
   out_7015503109436020306[0] = 0;
   out_7015503109436020306[1] = 0;
   out_7015503109436020306[2] = 0;
   out_7015503109436020306[3] = 1;
   out_7015503109436020306[4] = 0;
   out_7015503109436020306[5] = 0;
   out_7015503109436020306[6] = 0;
   out_7015503109436020306[7] = 0;
   out_7015503109436020306[8] = 0;
}
void h_29(double *state, double *unused, double *out_8170070345181851364) {
   out_8170070345181851364[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4330508453321203211) {
   out_4330508453321203211[0] = 0;
   out_4330508453321203211[1] = 1;
   out_4330508453321203211[2] = 0;
   out_4330508453321203211[3] = 0;
   out_4330508453321203211[4] = 0;
   out_4330508453321203211[5] = 0;
   out_4330508453321203211[6] = 0;
   out_4330508453321203211[7] = 0;
   out_4330508453321203211[8] = 0;
}
void h_28(double *state, double *unused, double *out_3602438148829345217) {
   out_3602438148829345217[0] = state[0];
}
void H_28(double *state, double *unused, double *out_9033836603318817831) {
   out_9033836603318817831[0] = 1;
   out_9033836603318817831[1] = 0;
   out_9033836603318817831[2] = 0;
   out_9033836603318817831[3] = 0;
   out_9033836603318817831[4] = 0;
   out_9033836603318817831[5] = 0;
   out_9033836603318817831[6] = 0;
   out_9033836603318817831[7] = 0;
   out_9033836603318817831[8] = 0;
}
void h_31(double *state, double *unused, double *out_1138122922153847066) {
   out_1138122922153847066[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6719959896459299894) {
   out_6719959896459299894[0] = 0;
   out_6719959896459299894[1] = 0;
   out_6719959896459299894[2] = 0;
   out_6719959896459299894[3] = 0;
   out_6719959896459299894[4] = 0;
   out_6719959896459299894[5] = 0;
   out_6719959896459299894[6] = 0;
   out_6719959896459299894[7] = 0;
   out_6719959896459299894[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6707971512290491444) {
  err_fun(nom_x, delta_x, out_6707971512290491444);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7853994834193184287) {
  inv_err_fun(nom_x, true_x, out_7853994834193184287);
}
void car_H_mod_fun(double *state, double *out_4010081315495760417) {
  H_mod_fun(state, out_4010081315495760417);
}
void car_f_fun(double *state, double dt, double *out_4685496284796261050) {
  f_fun(state,  dt, out_4685496284796261050);
}
void car_F_fun(double *state, double dt, double *out_8622935208793728795) {
  F_fun(state,  dt, out_8622935208793728795);
}
void car_h_25(double *state, double *unused, double *out_1413316984438352955) {
  h_25(state, unused, out_1413316984438352955);
}
void car_H_25(double *state, double *unused, double *out_7359072756142844022) {
  H_25(state, unused, out_7359072756142844022);
}
void car_h_24(double *state, double *unused, double *out_1193875120514303785) {
  h_24(state, unused, out_1193875120514303785);
}
void car_H_24(double *state, double *unused, double *out_6341161990600198560) {
  H_24(state, unused, out_6341161990600198560);
}
void car_h_30(double *state, double *unused, double *out_1256130316087223810) {
  h_30(state, unused, out_1256130316087223810);
}
void car_H_30(double *state, double *unused, double *out_4840739797635595395) {
  H_30(state, unused, out_4840739797635595395);
}
void car_h_26(double *state, double *unused, double *out_2944996651036528421) {
  h_26(state, unused, out_2944996651036528421);
}
void car_H_26(double *state, double *unused, double *out_4054546786382043421) {
  H_26(state, unused, out_4054546786382043421);
}
void car_h_27(double *state, double *unused, double *out_6992339835205873698) {
  h_27(state, unused, out_6992339835205873698);
}
void car_H_27(double *state, double *unused, double *out_7015503109436020306) {
  H_27(state, unused, out_7015503109436020306);
}
void car_h_29(double *state, double *unused, double *out_8170070345181851364) {
  h_29(state, unused, out_8170070345181851364);
}
void car_H_29(double *state, double *unused, double *out_4330508453321203211) {
  H_29(state, unused, out_4330508453321203211);
}
void car_h_28(double *state, double *unused, double *out_3602438148829345217) {
  h_28(state, unused, out_3602438148829345217);
}
void car_H_28(double *state, double *unused, double *out_9033836603318817831) {
  H_28(state, unused, out_9033836603318817831);
}
void car_h_31(double *state, double *unused, double *out_1138122922153847066) {
  h_31(state, unused, out_1138122922153847066);
}
void car_H_31(double *state, double *unused, double *out_6719959896459299894) {
  H_31(state, unused, out_6719959896459299894);
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
