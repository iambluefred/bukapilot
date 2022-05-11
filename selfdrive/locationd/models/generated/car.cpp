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
void err_fun(double *nom_x, double *delta_x, double *out_2080459397635649486) {
   out_2080459397635649486[0] = delta_x[0] + nom_x[0];
   out_2080459397635649486[1] = delta_x[1] + nom_x[1];
   out_2080459397635649486[2] = delta_x[2] + nom_x[2];
   out_2080459397635649486[3] = delta_x[3] + nom_x[3];
   out_2080459397635649486[4] = delta_x[4] + nom_x[4];
   out_2080459397635649486[5] = delta_x[5] + nom_x[5];
   out_2080459397635649486[6] = delta_x[6] + nom_x[6];
   out_2080459397635649486[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7549402271710703544) {
   out_7549402271710703544[0] = -nom_x[0] + true_x[0];
   out_7549402271710703544[1] = -nom_x[1] + true_x[1];
   out_7549402271710703544[2] = -nom_x[2] + true_x[2];
   out_7549402271710703544[3] = -nom_x[3] + true_x[3];
   out_7549402271710703544[4] = -nom_x[4] + true_x[4];
   out_7549402271710703544[5] = -nom_x[5] + true_x[5];
   out_7549402271710703544[6] = -nom_x[6] + true_x[6];
   out_7549402271710703544[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_7998804296790963776) {
   out_7998804296790963776[0] = 1.0;
   out_7998804296790963776[1] = 0;
   out_7998804296790963776[2] = 0;
   out_7998804296790963776[3] = 0;
   out_7998804296790963776[4] = 0;
   out_7998804296790963776[5] = 0;
   out_7998804296790963776[6] = 0;
   out_7998804296790963776[7] = 0;
   out_7998804296790963776[8] = 0;
   out_7998804296790963776[9] = 1.0;
   out_7998804296790963776[10] = 0;
   out_7998804296790963776[11] = 0;
   out_7998804296790963776[12] = 0;
   out_7998804296790963776[13] = 0;
   out_7998804296790963776[14] = 0;
   out_7998804296790963776[15] = 0;
   out_7998804296790963776[16] = 0;
   out_7998804296790963776[17] = 0;
   out_7998804296790963776[18] = 1.0;
   out_7998804296790963776[19] = 0;
   out_7998804296790963776[20] = 0;
   out_7998804296790963776[21] = 0;
   out_7998804296790963776[22] = 0;
   out_7998804296790963776[23] = 0;
   out_7998804296790963776[24] = 0;
   out_7998804296790963776[25] = 0;
   out_7998804296790963776[26] = 0;
   out_7998804296790963776[27] = 1.0;
   out_7998804296790963776[28] = 0;
   out_7998804296790963776[29] = 0;
   out_7998804296790963776[30] = 0;
   out_7998804296790963776[31] = 0;
   out_7998804296790963776[32] = 0;
   out_7998804296790963776[33] = 0;
   out_7998804296790963776[34] = 0;
   out_7998804296790963776[35] = 0;
   out_7998804296790963776[36] = 1.0;
   out_7998804296790963776[37] = 0;
   out_7998804296790963776[38] = 0;
   out_7998804296790963776[39] = 0;
   out_7998804296790963776[40] = 0;
   out_7998804296790963776[41] = 0;
   out_7998804296790963776[42] = 0;
   out_7998804296790963776[43] = 0;
   out_7998804296790963776[44] = 0;
   out_7998804296790963776[45] = 1.0;
   out_7998804296790963776[46] = 0;
   out_7998804296790963776[47] = 0;
   out_7998804296790963776[48] = 0;
   out_7998804296790963776[49] = 0;
   out_7998804296790963776[50] = 0;
   out_7998804296790963776[51] = 0;
   out_7998804296790963776[52] = 0;
   out_7998804296790963776[53] = 0;
   out_7998804296790963776[54] = 1.0;
   out_7998804296790963776[55] = 0;
   out_7998804296790963776[56] = 0;
   out_7998804296790963776[57] = 0;
   out_7998804296790963776[58] = 0;
   out_7998804296790963776[59] = 0;
   out_7998804296790963776[60] = 0;
   out_7998804296790963776[61] = 0;
   out_7998804296790963776[62] = 0;
   out_7998804296790963776[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_1007093853574867969) {
   out_1007093853574867969[0] = state[0];
   out_1007093853574867969[1] = state[1];
   out_1007093853574867969[2] = state[2];
   out_1007093853574867969[3] = state[3];
   out_1007093853574867969[4] = state[4];
   out_1007093853574867969[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1007093853574867969[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1007093853574867969[7] = state[7];
}
void F_fun(double *state, double dt, double *out_3972400413518286592) {
   out_3972400413518286592[0] = 1;
   out_3972400413518286592[1] = 0;
   out_3972400413518286592[2] = 0;
   out_3972400413518286592[3] = 0;
   out_3972400413518286592[4] = 0;
   out_3972400413518286592[5] = 0;
   out_3972400413518286592[6] = 0;
   out_3972400413518286592[7] = 0;
   out_3972400413518286592[8] = 0;
   out_3972400413518286592[9] = 1;
   out_3972400413518286592[10] = 0;
   out_3972400413518286592[11] = 0;
   out_3972400413518286592[12] = 0;
   out_3972400413518286592[13] = 0;
   out_3972400413518286592[14] = 0;
   out_3972400413518286592[15] = 0;
   out_3972400413518286592[16] = 0;
   out_3972400413518286592[17] = 0;
   out_3972400413518286592[18] = 1;
   out_3972400413518286592[19] = 0;
   out_3972400413518286592[20] = 0;
   out_3972400413518286592[21] = 0;
   out_3972400413518286592[22] = 0;
   out_3972400413518286592[23] = 0;
   out_3972400413518286592[24] = 0;
   out_3972400413518286592[25] = 0;
   out_3972400413518286592[26] = 0;
   out_3972400413518286592[27] = 1;
   out_3972400413518286592[28] = 0;
   out_3972400413518286592[29] = 0;
   out_3972400413518286592[30] = 0;
   out_3972400413518286592[31] = 0;
   out_3972400413518286592[32] = 0;
   out_3972400413518286592[33] = 0;
   out_3972400413518286592[34] = 0;
   out_3972400413518286592[35] = 0;
   out_3972400413518286592[36] = 1;
   out_3972400413518286592[37] = 0;
   out_3972400413518286592[38] = 0;
   out_3972400413518286592[39] = 0;
   out_3972400413518286592[40] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3972400413518286592[41] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3972400413518286592[42] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3972400413518286592[43] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3972400413518286592[44] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3972400413518286592[45] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3972400413518286592[46] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3972400413518286592[47] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3972400413518286592[48] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3972400413518286592[49] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3972400413518286592[50] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3972400413518286592[51] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3972400413518286592[52] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3972400413518286592[53] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3972400413518286592[54] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3972400413518286592[55] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3972400413518286592[56] = 0;
   out_3972400413518286592[57] = 0;
   out_3972400413518286592[58] = 0;
   out_3972400413518286592[59] = 0;
   out_3972400413518286592[60] = 0;
   out_3972400413518286592[61] = 0;
   out_3972400413518286592[62] = 0;
   out_3972400413518286592[63] = 1;
}
void h_25(double *state, double *unused, double *out_6186081259470436288) {
   out_6186081259470436288[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7277562395580263808) {
   out_7277562395580263808[0] = 0;
   out_7277562395580263808[1] = 0;
   out_7277562395580263808[2] = 0;
   out_7277562395580263808[3] = 0;
   out_7277562395580263808[4] = 0;
   out_7277562395580263808[5] = 0;
   out_7277562395580263808[6] = 1;
   out_7277562395580263808[7] = 0;
}
void h_24(double *state, double *unused, double *out_2163392311181086579) {
   out_2163392311181086579[0] = state[4];
   out_2163392311181086579[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7159240284580830777) {
   out_7159240284580830777[0] = 0;
   out_7159240284580830777[1] = 0;
   out_7159240284580830777[2] = 0;
   out_7159240284580830777[3] = 0;
   out_7159240284580830777[4] = 1;
   out_7159240284580830777[5] = 0;
   out_7159240284580830777[6] = 0;
   out_7159240284580830777[7] = 0;
   out_7159240284580830777[8] = 0;
   out_7159240284580830777[9] = 0;
   out_7159240284580830777[10] = 0;
   out_7159240284580830777[11] = 0;
   out_7159240284580830777[12] = 0;
   out_7159240284580830777[13] = 1;
   out_7159240284580830777[14] = 0;
   out_7159240284580830777[15] = 0;
}
void h_30(double *state, double *unused, double *out_6135121145360409701) {
   out_6135121145360409701[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2336336515368919472) {
   out_2336336515368919472[0] = 0;
   out_2336336515368919472[1] = 0;
   out_2336336515368919472[2] = 0;
   out_2336336515368919472[3] = 0;
   out_2336336515368919472[4] = 1;
   out_2336336515368919472[5] = 0;
   out_2336336515368919472[6] = 0;
   out_2336336515368919472[7] = 0;
}
void h_26(double *state, double *unused, double *out_3530715824551712050) {
   out_3530715824551712050[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8583798167535104953) {
   out_8583798167535104953[0] = 0;
   out_8583798167535104953[1] = 0;
   out_8583798167535104953[2] = 0;
   out_8583798167535104953[3] = 0;
   out_8583798167535104953[4] = 0;
   out_8583798167535104953[5] = 0;
   out_8583798167535104953[6] = 0;
   out_8583798167535104953[7] = 1;
}
void h_27(double *state, double *unused, double *out_307795771911721200) {
   out_307795771911721200[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3623918503205544784) {
   out_3623918503205544784[0] = 0;
   out_3623918503205544784[1] = 0;
   out_3623918503205544784[2] = 0;
   out_3623918503205544784[3] = 1;
   out_3623918503205544784[4] = 0;
   out_3623918503205544784[5] = 0;
   out_3623918503205544784[6] = 0;
   out_3623918503205544784[7] = 0;
}
void h_29(double *state, double *unused, double *out_8947673248630263084) {
   out_8947673248630263084[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6160135284497751425) {
   out_6160135284497751425[0] = 0;
   out_6160135284497751425[1] = 1;
   out_6160135284497751425[2] = 0;
   out_6160135284497751425[3] = 0;
   out_6160135284497751425[4] = 0;
   out_6160135284497751425[5] = 0;
   out_6160135284497751425[6] = 0;
   out_6160135284497751425[7] = 0;
}
void h_28(double *state, double *unused, double *out_691852136549595211) {
   out_691852136549595211[0] = state[5];
   out_691852136549595211[1] = state[6];
}
void H_28(double *state, double *unused, double *out_4269671310892225973) {
   out_4269671310892225973[0] = 0;
   out_4269671310892225973[1] = 0;
   out_4269671310892225973[2] = 0;
   out_4269671310892225973[3] = 0;
   out_4269671310892225973[4] = 0;
   out_4269671310892225973[5] = 1;
   out_4269671310892225973[6] = 0;
   out_4269671310892225973[7] = 0;
   out_4269671310892225973[8] = 0;
   out_4269671310892225973[9] = 0;
   out_4269671310892225973[10] = 0;
   out_4269671310892225973[11] = 0;
   out_4269671310892225973[12] = 0;
   out_4269671310892225973[13] = 0;
   out_4269671310892225973[14] = 1;
   out_4269671310892225973[15] = 0;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2080459397635649486) {
  err_fun(nom_x, delta_x, out_2080459397635649486);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7549402271710703544) {
  inv_err_fun(nom_x, true_x, out_7549402271710703544);
}
void car_H_mod_fun(double *state, double *out_7998804296790963776) {
  H_mod_fun(state, out_7998804296790963776);
}
void car_f_fun(double *state, double dt, double *out_1007093853574867969) {
  f_fun(state,  dt, out_1007093853574867969);
}
void car_F_fun(double *state, double dt, double *out_3972400413518286592) {
  F_fun(state,  dt, out_3972400413518286592);
}
void car_h_25(double *state, double *unused, double *out_6186081259470436288) {
  h_25(state, unused, out_6186081259470436288);
}
void car_H_25(double *state, double *unused, double *out_7277562395580263808) {
  H_25(state, unused, out_7277562395580263808);
}
void car_h_24(double *state, double *unused, double *out_2163392311181086579) {
  h_24(state, unused, out_2163392311181086579);
}
void car_H_24(double *state, double *unused, double *out_7159240284580830777) {
  H_24(state, unused, out_7159240284580830777);
}
void car_h_30(double *state, double *unused, double *out_6135121145360409701) {
  h_30(state, unused, out_6135121145360409701);
}
void car_H_30(double *state, double *unused, double *out_2336336515368919472) {
  H_30(state, unused, out_2336336515368919472);
}
void car_h_26(double *state, double *unused, double *out_3530715824551712050) {
  h_26(state, unused, out_3530715824551712050);
}
void car_H_26(double *state, double *unused, double *out_8583798167535104953) {
  H_26(state, unused, out_8583798167535104953);
}
void car_h_27(double *state, double *unused, double *out_307795771911721200) {
  h_27(state, unused, out_307795771911721200);
}
void car_H_27(double *state, double *unused, double *out_3623918503205544784) {
  H_27(state, unused, out_3623918503205544784);
}
void car_h_29(double *state, double *unused, double *out_8947673248630263084) {
  h_29(state, unused, out_8947673248630263084);
}
void car_H_29(double *state, double *unused, double *out_6160135284497751425) {
  H_29(state, unused, out_6160135284497751425);
}
void car_h_28(double *state, double *unused, double *out_691852136549595211) {
  h_28(state, unused, out_691852136549595211);
}
void car_H_28(double *state, double *unused, double *out_4269671310892225973) {
  H_28(state, unused, out_4269671310892225973);
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
