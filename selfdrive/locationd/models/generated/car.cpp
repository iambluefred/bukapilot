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
void err_fun(double *nom_x, double *delta_x, double *out_575770558294804495) {
   out_575770558294804495[0] = delta_x[0] + nom_x[0];
   out_575770558294804495[1] = delta_x[1] + nom_x[1];
   out_575770558294804495[2] = delta_x[2] + nom_x[2];
   out_575770558294804495[3] = delta_x[3] + nom_x[3];
   out_575770558294804495[4] = delta_x[4] + nom_x[4];
   out_575770558294804495[5] = delta_x[5] + nom_x[5];
   out_575770558294804495[6] = delta_x[6] + nom_x[6];
   out_575770558294804495[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5727502109067883687) {
   out_5727502109067883687[0] = -nom_x[0] + true_x[0];
   out_5727502109067883687[1] = -nom_x[1] + true_x[1];
   out_5727502109067883687[2] = -nom_x[2] + true_x[2];
   out_5727502109067883687[3] = -nom_x[3] + true_x[3];
   out_5727502109067883687[4] = -nom_x[4] + true_x[4];
   out_5727502109067883687[5] = -nom_x[5] + true_x[5];
   out_5727502109067883687[6] = -nom_x[6] + true_x[6];
   out_5727502109067883687[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_6921934919768022397) {
   out_6921934919768022397[0] = 1.0;
   out_6921934919768022397[1] = 0;
   out_6921934919768022397[2] = 0;
   out_6921934919768022397[3] = 0;
   out_6921934919768022397[4] = 0;
   out_6921934919768022397[5] = 0;
   out_6921934919768022397[6] = 0;
   out_6921934919768022397[7] = 0;
   out_6921934919768022397[8] = 0;
   out_6921934919768022397[9] = 1.0;
   out_6921934919768022397[10] = 0;
   out_6921934919768022397[11] = 0;
   out_6921934919768022397[12] = 0;
   out_6921934919768022397[13] = 0;
   out_6921934919768022397[14] = 0;
   out_6921934919768022397[15] = 0;
   out_6921934919768022397[16] = 0;
   out_6921934919768022397[17] = 0;
   out_6921934919768022397[18] = 1.0;
   out_6921934919768022397[19] = 0;
   out_6921934919768022397[20] = 0;
   out_6921934919768022397[21] = 0;
   out_6921934919768022397[22] = 0;
   out_6921934919768022397[23] = 0;
   out_6921934919768022397[24] = 0;
   out_6921934919768022397[25] = 0;
   out_6921934919768022397[26] = 0;
   out_6921934919768022397[27] = 1.0;
   out_6921934919768022397[28] = 0;
   out_6921934919768022397[29] = 0;
   out_6921934919768022397[30] = 0;
   out_6921934919768022397[31] = 0;
   out_6921934919768022397[32] = 0;
   out_6921934919768022397[33] = 0;
   out_6921934919768022397[34] = 0;
   out_6921934919768022397[35] = 0;
   out_6921934919768022397[36] = 1.0;
   out_6921934919768022397[37] = 0;
   out_6921934919768022397[38] = 0;
   out_6921934919768022397[39] = 0;
   out_6921934919768022397[40] = 0;
   out_6921934919768022397[41] = 0;
   out_6921934919768022397[42] = 0;
   out_6921934919768022397[43] = 0;
   out_6921934919768022397[44] = 0;
   out_6921934919768022397[45] = 1.0;
   out_6921934919768022397[46] = 0;
   out_6921934919768022397[47] = 0;
   out_6921934919768022397[48] = 0;
   out_6921934919768022397[49] = 0;
   out_6921934919768022397[50] = 0;
   out_6921934919768022397[51] = 0;
   out_6921934919768022397[52] = 0;
   out_6921934919768022397[53] = 0;
   out_6921934919768022397[54] = 1.0;
   out_6921934919768022397[55] = 0;
   out_6921934919768022397[56] = 0;
   out_6921934919768022397[57] = 0;
   out_6921934919768022397[58] = 0;
   out_6921934919768022397[59] = 0;
   out_6921934919768022397[60] = 0;
   out_6921934919768022397[61] = 0;
   out_6921934919768022397[62] = 0;
   out_6921934919768022397[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_247457335877544533) {
   out_247457335877544533[0] = state[0];
   out_247457335877544533[1] = state[1];
   out_247457335877544533[2] = state[2];
   out_247457335877544533[3] = state[3];
   out_247457335877544533[4] = state[4];
   out_247457335877544533[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_247457335877544533[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_247457335877544533[7] = state[7];
}
void F_fun(double *state, double dt, double *out_8800351135451747642) {
   out_8800351135451747642[0] = 1;
   out_8800351135451747642[1] = 0;
   out_8800351135451747642[2] = 0;
   out_8800351135451747642[3] = 0;
   out_8800351135451747642[4] = 0;
   out_8800351135451747642[5] = 0;
   out_8800351135451747642[6] = 0;
   out_8800351135451747642[7] = 0;
   out_8800351135451747642[8] = 0;
   out_8800351135451747642[9] = 1;
   out_8800351135451747642[10] = 0;
   out_8800351135451747642[11] = 0;
   out_8800351135451747642[12] = 0;
   out_8800351135451747642[13] = 0;
   out_8800351135451747642[14] = 0;
   out_8800351135451747642[15] = 0;
   out_8800351135451747642[16] = 0;
   out_8800351135451747642[17] = 0;
   out_8800351135451747642[18] = 1;
   out_8800351135451747642[19] = 0;
   out_8800351135451747642[20] = 0;
   out_8800351135451747642[21] = 0;
   out_8800351135451747642[22] = 0;
   out_8800351135451747642[23] = 0;
   out_8800351135451747642[24] = 0;
   out_8800351135451747642[25] = 0;
   out_8800351135451747642[26] = 0;
   out_8800351135451747642[27] = 1;
   out_8800351135451747642[28] = 0;
   out_8800351135451747642[29] = 0;
   out_8800351135451747642[30] = 0;
   out_8800351135451747642[31] = 0;
   out_8800351135451747642[32] = 0;
   out_8800351135451747642[33] = 0;
   out_8800351135451747642[34] = 0;
   out_8800351135451747642[35] = 0;
   out_8800351135451747642[36] = 1;
   out_8800351135451747642[37] = 0;
   out_8800351135451747642[38] = 0;
   out_8800351135451747642[39] = 0;
   out_8800351135451747642[40] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8800351135451747642[41] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8800351135451747642[42] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8800351135451747642[43] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8800351135451747642[44] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8800351135451747642[45] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8800351135451747642[46] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8800351135451747642[47] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8800351135451747642[48] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8800351135451747642[49] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8800351135451747642[50] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8800351135451747642[51] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8800351135451747642[52] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8800351135451747642[53] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8800351135451747642[54] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8800351135451747642[55] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8800351135451747642[56] = 0;
   out_8800351135451747642[57] = 0;
   out_8800351135451747642[58] = 0;
   out_8800351135451747642[59] = 0;
   out_8800351135451747642[60] = 0;
   out_8800351135451747642[61] = 0;
   out_8800351135451747642[62] = 0;
   out_8800351135451747642[63] = 1;
}
void h_25(double *state, double *unused, double *out_1962729859178462780) {
   out_1962729859178462780[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5440771516178847649) {
   out_5440771516178847649[0] = 0;
   out_5440771516178847649[1] = 0;
   out_5440771516178847649[2] = 0;
   out_5440771516178847649[3] = 0;
   out_5440771516178847649[4] = 0;
   out_5440771516178847649[5] = 0;
   out_5440771516178847649[6] = 1;
   out_5440771516178847649[7] = 0;
}
void h_24(double *state, double *unused, double *out_4975792545761529740) {
   out_4975792545761529740[0] = state[4];
   out_4975792545761529740[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5890114518268132700) {
   out_5890114518268132700[0] = 0;
   out_5890114518268132700[1] = 0;
   out_5890114518268132700[2] = 0;
   out_5890114518268132700[3] = 0;
   out_5890114518268132700[4] = 1;
   out_5890114518268132700[5] = 0;
   out_5890114518268132700[6] = 0;
   out_5890114518268132700[7] = 0;
   out_5890114518268132700[8] = 0;
   out_5890114518268132700[9] = 0;
   out_5890114518268132700[10] = 0;
   out_5890114518268132700[11] = 0;
   out_5890114518268132700[12] = 0;
   out_5890114518268132700[13] = 1;
   out_5890114518268132700[14] = 0;
   out_5890114518268132700[15] = 0;
}
void h_30(double *state, double *unused, double *out_6061033157457834644) {
   out_6061033157457834644[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3653955642053336138) {
   out_3653955642053336138[0] = 0;
   out_3653955642053336138[1] = 0;
   out_3653955642053336138[2] = 0;
   out_3653955642053336138[3] = 0;
   out_3653955642053336138[4] = 1;
   out_3653955642053336138[5] = 0;
   out_3653955642053336138[6] = 0;
   out_3653955642053336138[7] = 0;
}
void h_26(double *state, double *unused, double *out_3589401451985719832) {
   out_3589401451985719832[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6782207649874495201) {
   out_6782207649874495201[0] = 0;
   out_6782207649874495201[1] = 0;
   out_6782207649874495201[2] = 0;
   out_6782207649874495201[3] = 0;
   out_6782207649874495201[4] = 0;
   out_6782207649874495201[5] = 0;
   out_6782207649874495201[6] = 0;
   out_6782207649874495201[7] = 1;
}
void h_27(double *state, double *unused, double *out_4999515910622795465) {
   out_4999515910622795465[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4941537629889961450) {
   out_4941537629889961450[0] = 0;
   out_4941537629889961450[1] = 0;
   out_4941537629889961450[2] = 0;
   out_4941537629889961450[3] = 1;
   out_4941537629889961450[4] = 0;
   out_4941537629889961450[5] = 0;
   out_4941537629889961450[6] = 0;
   out_4941537629889961450[7] = 0;
}
void h_29(double *state, double *unused, double *out_1059359889796988999) {
   out_1059359889796988999[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3966632260437056862) {
   out_3966632260437056862[0] = 0;
   out_3966632260437056862[1] = 1;
   out_3966632260437056862[2] = 0;
   out_3966632260437056862[3] = 0;
   out_3966632260437056862[4] = 0;
   out_3966632260437056862[5] = 0;
   out_3966632260437056862[6] = 0;
   out_3966632260437056862[7] = 0;
}
void h_28(double *state, double *unused, double *out_3984807951886966422) {
   out_3984807951886966422[0] = state[5];
   out_3984807951886966422[1] = state[6];
}
void H_28(double *state, double *unused, double *out_5538797077204924050) {
   out_5538797077204924050[0] = 0;
   out_5538797077204924050[1] = 0;
   out_5538797077204924050[2] = 0;
   out_5538797077204924050[3] = 0;
   out_5538797077204924050[4] = 0;
   out_5538797077204924050[5] = 1;
   out_5538797077204924050[6] = 0;
   out_5538797077204924050[7] = 0;
   out_5538797077204924050[8] = 0;
   out_5538797077204924050[9] = 0;
   out_5538797077204924050[10] = 0;
   out_5538797077204924050[11] = 0;
   out_5538797077204924050[12] = 0;
   out_5538797077204924050[13] = 0;
   out_5538797077204924050[14] = 1;
   out_5538797077204924050[15] = 0;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_575770558294804495) {
  err_fun(nom_x, delta_x, out_575770558294804495);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5727502109067883687) {
  inv_err_fun(nom_x, true_x, out_5727502109067883687);
}
void car_H_mod_fun(double *state, double *out_6921934919768022397) {
  H_mod_fun(state, out_6921934919768022397);
}
void car_f_fun(double *state, double dt, double *out_247457335877544533) {
  f_fun(state,  dt, out_247457335877544533);
}
void car_F_fun(double *state, double dt, double *out_8800351135451747642) {
  F_fun(state,  dt, out_8800351135451747642);
}
void car_h_25(double *state, double *unused, double *out_1962729859178462780) {
  h_25(state, unused, out_1962729859178462780);
}
void car_H_25(double *state, double *unused, double *out_5440771516178847649) {
  H_25(state, unused, out_5440771516178847649);
}
void car_h_24(double *state, double *unused, double *out_4975792545761529740) {
  h_24(state, unused, out_4975792545761529740);
}
void car_H_24(double *state, double *unused, double *out_5890114518268132700) {
  H_24(state, unused, out_5890114518268132700);
}
void car_h_30(double *state, double *unused, double *out_6061033157457834644) {
  h_30(state, unused, out_6061033157457834644);
}
void car_H_30(double *state, double *unused, double *out_3653955642053336138) {
  H_30(state, unused, out_3653955642053336138);
}
void car_h_26(double *state, double *unused, double *out_3589401451985719832) {
  h_26(state, unused, out_3589401451985719832);
}
void car_H_26(double *state, double *unused, double *out_6782207649874495201) {
  H_26(state, unused, out_6782207649874495201);
}
void car_h_27(double *state, double *unused, double *out_4999515910622795465) {
  h_27(state, unused, out_4999515910622795465);
}
void car_H_27(double *state, double *unused, double *out_4941537629889961450) {
  H_27(state, unused, out_4941537629889961450);
}
void car_h_29(double *state, double *unused, double *out_1059359889796988999) {
  h_29(state, unused, out_1059359889796988999);
}
void car_H_29(double *state, double *unused, double *out_3966632260437056862) {
  H_29(state, unused, out_3966632260437056862);
}
void car_h_28(double *state, double *unused, double *out_3984807951886966422) {
  h_28(state, unused, out_3984807951886966422);
}
void car_H_28(double *state, double *unused, double *out_5538797077204924050) {
  H_28(state, unused, out_5538797077204924050);
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
