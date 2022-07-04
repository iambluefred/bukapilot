#include "car.h"

namespace {
#define DIM 8
#define EDIM 8
#define MEDIM 8
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
const static double MAHA_THRESH_28 = 5.991464547107981;

/******************************************************************************
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3695708287462369075) {
   out_3695708287462369075[0] = delta_x[0] + nom_x[0];
   out_3695708287462369075[1] = delta_x[1] + nom_x[1];
   out_3695708287462369075[2] = delta_x[2] + nom_x[2];
   out_3695708287462369075[3] = delta_x[3] + nom_x[3];
   out_3695708287462369075[4] = delta_x[4] + nom_x[4];
   out_3695708287462369075[5] = delta_x[5] + nom_x[5];
   out_3695708287462369075[6] = delta_x[6] + nom_x[6];
   out_3695708287462369075[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5497721190925856545) {
   out_5497721190925856545[0] = -nom_x[0] + true_x[0];
   out_5497721190925856545[1] = -nom_x[1] + true_x[1];
   out_5497721190925856545[2] = -nom_x[2] + true_x[2];
   out_5497721190925856545[3] = -nom_x[3] + true_x[3];
   out_5497721190925856545[4] = -nom_x[4] + true_x[4];
   out_5497721190925856545[5] = -nom_x[5] + true_x[5];
   out_5497721190925856545[6] = -nom_x[6] + true_x[6];
   out_5497721190925856545[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_3741989862932459432) {
   out_3741989862932459432[0] = 1.0;
   out_3741989862932459432[1] = 0;
   out_3741989862932459432[2] = 0;
   out_3741989862932459432[3] = 0;
   out_3741989862932459432[4] = 0;
   out_3741989862932459432[5] = 0;
   out_3741989862932459432[6] = 0;
   out_3741989862932459432[7] = 0;
   out_3741989862932459432[8] = 0;
   out_3741989862932459432[9] = 1.0;
   out_3741989862932459432[10] = 0;
   out_3741989862932459432[11] = 0;
   out_3741989862932459432[12] = 0;
   out_3741989862932459432[13] = 0;
   out_3741989862932459432[14] = 0;
   out_3741989862932459432[15] = 0;
   out_3741989862932459432[16] = 0;
   out_3741989862932459432[17] = 0;
   out_3741989862932459432[18] = 1.0;
   out_3741989862932459432[19] = 0;
   out_3741989862932459432[20] = 0;
   out_3741989862932459432[21] = 0;
   out_3741989862932459432[22] = 0;
   out_3741989862932459432[23] = 0;
   out_3741989862932459432[24] = 0;
   out_3741989862932459432[25] = 0;
   out_3741989862932459432[26] = 0;
   out_3741989862932459432[27] = 1.0;
   out_3741989862932459432[28] = 0;
   out_3741989862932459432[29] = 0;
   out_3741989862932459432[30] = 0;
   out_3741989862932459432[31] = 0;
   out_3741989862932459432[32] = 0;
   out_3741989862932459432[33] = 0;
   out_3741989862932459432[34] = 0;
   out_3741989862932459432[35] = 0;
   out_3741989862932459432[36] = 1.0;
   out_3741989862932459432[37] = 0;
   out_3741989862932459432[38] = 0;
   out_3741989862932459432[39] = 0;
   out_3741989862932459432[40] = 0;
   out_3741989862932459432[41] = 0;
   out_3741989862932459432[42] = 0;
   out_3741989862932459432[43] = 0;
   out_3741989862932459432[44] = 0;
   out_3741989862932459432[45] = 1.0;
   out_3741989862932459432[46] = 0;
   out_3741989862932459432[47] = 0;
   out_3741989862932459432[48] = 0;
   out_3741989862932459432[49] = 0;
   out_3741989862932459432[50] = 0;
   out_3741989862932459432[51] = 0;
   out_3741989862932459432[52] = 0;
   out_3741989862932459432[53] = 0;
   out_3741989862932459432[54] = 1.0;
   out_3741989862932459432[55] = 0;
   out_3741989862932459432[56] = 0;
   out_3741989862932459432[57] = 0;
   out_3741989862932459432[58] = 0;
   out_3741989862932459432[59] = 0;
   out_3741989862932459432[60] = 0;
   out_3741989862932459432[61] = 0;
   out_3741989862932459432[62] = 0;
   out_3741989862932459432[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_5295389726812860391) {
   out_5295389726812860391[0] = state[0];
   out_5295389726812860391[1] = state[1];
   out_5295389726812860391[2] = state[2];
   out_5295389726812860391[3] = state[3];
   out_5295389726812860391[4] = state[4];
   out_5295389726812860391[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5295389726812860391[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5295389726812860391[7] = state[7];
}
void F_fun(double *state, double dt, double *out_6749094151876649186) {
   out_6749094151876649186[0] = 1;
   out_6749094151876649186[1] = 0;
   out_6749094151876649186[2] = 0;
   out_6749094151876649186[3] = 0;
   out_6749094151876649186[4] = 0;
   out_6749094151876649186[5] = 0;
   out_6749094151876649186[6] = 0;
   out_6749094151876649186[7] = 0;
   out_6749094151876649186[8] = 0;
   out_6749094151876649186[9] = 1;
   out_6749094151876649186[10] = 0;
   out_6749094151876649186[11] = 0;
   out_6749094151876649186[12] = 0;
   out_6749094151876649186[13] = 0;
   out_6749094151876649186[14] = 0;
   out_6749094151876649186[15] = 0;
   out_6749094151876649186[16] = 0;
   out_6749094151876649186[17] = 0;
   out_6749094151876649186[18] = 1;
   out_6749094151876649186[19] = 0;
   out_6749094151876649186[20] = 0;
   out_6749094151876649186[21] = 0;
   out_6749094151876649186[22] = 0;
   out_6749094151876649186[23] = 0;
   out_6749094151876649186[24] = 0;
   out_6749094151876649186[25] = 0;
   out_6749094151876649186[26] = 0;
   out_6749094151876649186[27] = 1;
   out_6749094151876649186[28] = 0;
   out_6749094151876649186[29] = 0;
   out_6749094151876649186[30] = 0;
   out_6749094151876649186[31] = 0;
   out_6749094151876649186[32] = 0;
   out_6749094151876649186[33] = 0;
   out_6749094151876649186[34] = 0;
   out_6749094151876649186[35] = 0;
   out_6749094151876649186[36] = 1;
   out_6749094151876649186[37] = 0;
   out_6749094151876649186[38] = 0;
   out_6749094151876649186[39] = 0;
   out_6749094151876649186[40] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6749094151876649186[41] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6749094151876649186[42] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6749094151876649186[43] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6749094151876649186[44] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6749094151876649186[45] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6749094151876649186[46] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6749094151876649186[47] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6749094151876649186[48] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6749094151876649186[49] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6749094151876649186[50] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6749094151876649186[51] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6749094151876649186[52] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6749094151876649186[53] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6749094151876649186[54] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6749094151876649186[55] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6749094151876649186[56] = 0;
   out_6749094151876649186[57] = 0;
   out_6749094151876649186[58] = 0;
   out_6749094151876649186[59] = 0;
   out_6749094151876649186[60] = 0;
   out_6749094151876649186[61] = 0;
   out_6749094151876649186[62] = 0;
   out_6749094151876649186[63] = 1;
}
void h_25(double *state, double *unused, double *out_5488032525607789861) {
   out_5488032525607789861[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4647198212667562955) {
   out_4647198212667562955[0] = 0;
   out_4647198212667562955[1] = 0;
   out_4647198212667562955[2] = 0;
   out_4647198212667562955[3] = 0;
   out_4647198212667562955[4] = 0;
   out_4647198212667562955[5] = 0;
   out_4647198212667562955[6] = 1;
   out_4647198212667562955[7] = 0;
}
void h_24(double *state, double *unused, double *out_3590326092945540893) {
   out_3590326092945540893[0] = state[4];
   out_3590326092945540893[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1669240464361133365) {
   out_1669240464361133365[0] = 0;
   out_1669240464361133365[1] = 0;
   out_1669240464361133365[2] = 0;
   out_1669240464361133365[3] = 0;
   out_1669240464361133365[4] = 1;
   out_1669240464361133365[5] = 0;
   out_1669240464361133365[6] = 0;
   out_1669240464361133365[7] = 0;
   out_1669240464361133365[8] = 0;
   out_1669240464361133365[9] = 0;
   out_1669240464361133365[10] = 0;
   out_1669240464361133365[11] = 0;
   out_1669240464361133365[12] = 0;
   out_1669240464361133365[13] = 1;
   out_1669240464361133365[14] = 0;
   out_1669240464361133365[15] = 0;
}
void h_30(double *state, double *unused, double *out_5077438897347712570) {
   out_5077438897347712570[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4185646950092805381) {
   out_4185646950092805381[0] = 0;
   out_4185646950092805381[1] = 0;
   out_4185646950092805381[2] = 0;
   out_4185646950092805381[3] = 0;
   out_4185646950092805381[4] = 1;
   out_4185646950092805381[5] = 0;
   out_4185646950092805381[6] = 0;
   out_4185646950092805381[7] = 0;
}
void h_26(double *state, double *unused, double *out_5719960003050504704) {
   out_5719960003050504704[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1057394942271646318) {
   out_1057394942271646318[0] = 0;
   out_1057394942271646318[1] = 0;
   out_1057394942271646318[2] = 0;
   out_1057394942271646318[3] = 0;
   out_1057394942271646318[4] = 0;
   out_1057394942271646318[5] = 0;
   out_1057394942271646318[6] = 0;
   out_1057394942271646318[7] = 1;
}
void h_27(double *state, double *unused, double *out_7957038280440916110) {
   out_7957038280440916110[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2898064962256180069) {
   out_2898064962256180069[0] = 0;
   out_2898064962256180069[1] = 0;
   out_2898064962256180069[2] = 0;
   out_2898064962256180069[3] = 1;
   out_2898064962256180069[4] = 0;
   out_2898064962256180069[5] = 0;
   out_2898064962256180069[6] = 0;
   out_2898064962256180069[7] = 0;
}
void h_29(double *state, double *unused, double *out_4963648356717612393) {
   out_4963648356717612393[0] = state[1];
}
void H_29(double *state, double *unused, double *out_361848180963973428) {
   out_361848180963973428[0] = 0;
   out_361848180963973428[1] = 1;
   out_361848180963973428[2] = 0;
   out_361848180963973428[3] = 0;
   out_361848180963973428[4] = 0;
   out_361848180963973428[5] = 0;
   out_361848180963973428[6] = 0;
   out_361848180963973428[7] = 0;
}
void h_28(double *state, double *unused, double *out_3874669749310933570) {
   out_3874669749310933570[0] = state[5];
   out_3874669749310933570[1] = state[6];
}
void H_28(double *state, double *unused, double *out_1653765388214965162) {
   out_1653765388214965162[0] = 0;
   out_1653765388214965162[1] = 0;
   out_1653765388214965162[2] = 0;
   out_1653765388214965162[3] = 0;
   out_1653765388214965162[4] = 0;
   out_1653765388214965162[5] = 1;
   out_1653765388214965162[6] = 0;
   out_1653765388214965162[7] = 0;
   out_1653765388214965162[8] = 0;
   out_1653765388214965162[9] = 0;
   out_1653765388214965162[10] = 0;
   out_1653765388214965162[11] = 0;
   out_1653765388214965162[12] = 0;
   out_1653765388214965162[13] = 0;
   out_1653765388214965162[14] = 1;
   out_1653765388214965162[15] = 0;
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
  update<2, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_3695708287462369075) {
  err_fun(nom_x, delta_x, out_3695708287462369075);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5497721190925856545) {
  inv_err_fun(nom_x, true_x, out_5497721190925856545);
}
void car_H_mod_fun(double *state, double *out_3741989862932459432) {
  H_mod_fun(state, out_3741989862932459432);
}
void car_f_fun(double *state, double dt, double *out_5295389726812860391) {
  f_fun(state,  dt, out_5295389726812860391);
}
void car_F_fun(double *state, double dt, double *out_6749094151876649186) {
  F_fun(state,  dt, out_6749094151876649186);
}
void car_h_25(double *state, double *unused, double *out_5488032525607789861) {
  h_25(state, unused, out_5488032525607789861);
}
void car_H_25(double *state, double *unused, double *out_4647198212667562955) {
  H_25(state, unused, out_4647198212667562955);
}
void car_h_24(double *state, double *unused, double *out_3590326092945540893) {
  h_24(state, unused, out_3590326092945540893);
}
void car_H_24(double *state, double *unused, double *out_1669240464361133365) {
  H_24(state, unused, out_1669240464361133365);
}
void car_h_30(double *state, double *unused, double *out_5077438897347712570) {
  h_30(state, unused, out_5077438897347712570);
}
void car_H_30(double *state, double *unused, double *out_4185646950092805381) {
  H_30(state, unused, out_4185646950092805381);
}
void car_h_26(double *state, double *unused, double *out_5719960003050504704) {
  h_26(state, unused, out_5719960003050504704);
}
void car_H_26(double *state, double *unused, double *out_1057394942271646318) {
  H_26(state, unused, out_1057394942271646318);
}
void car_h_27(double *state, double *unused, double *out_7957038280440916110) {
  h_27(state, unused, out_7957038280440916110);
}
void car_H_27(double *state, double *unused, double *out_2898064962256180069) {
  H_27(state, unused, out_2898064962256180069);
}
void car_h_29(double *state, double *unused, double *out_4963648356717612393) {
  h_29(state, unused, out_4963648356717612393);
}
void car_H_29(double *state, double *unused, double *out_361848180963973428) {
  H_29(state, unused, out_361848180963973428);
}
void car_h_28(double *state, double *unused, double *out_3874669749310933570) {
  h_28(state, unused, out_3874669749310933570);
}
void car_H_28(double *state, double *unused, double *out_1653765388214965162) {
  H_28(state, unused, out_1653765388214965162);
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
  .kinds = { 25, 24, 30, 26, 27, 29, 28 },
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
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
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
