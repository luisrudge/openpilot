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
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5261127957076131017) {
   out_5261127957076131017[0] = delta_x[0] + nom_x[0];
   out_5261127957076131017[1] = delta_x[1] + nom_x[1];
   out_5261127957076131017[2] = delta_x[2] + nom_x[2];
   out_5261127957076131017[3] = delta_x[3] + nom_x[3];
   out_5261127957076131017[4] = delta_x[4] + nom_x[4];
   out_5261127957076131017[5] = delta_x[5] + nom_x[5];
   out_5261127957076131017[6] = delta_x[6] + nom_x[6];
   out_5261127957076131017[7] = delta_x[7] + nom_x[7];
   out_5261127957076131017[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5573080380081729814) {
   out_5573080380081729814[0] = -nom_x[0] + true_x[0];
   out_5573080380081729814[1] = -nom_x[1] + true_x[1];
   out_5573080380081729814[2] = -nom_x[2] + true_x[2];
   out_5573080380081729814[3] = -nom_x[3] + true_x[3];
   out_5573080380081729814[4] = -nom_x[4] + true_x[4];
   out_5573080380081729814[5] = -nom_x[5] + true_x[5];
   out_5573080380081729814[6] = -nom_x[6] + true_x[6];
   out_5573080380081729814[7] = -nom_x[7] + true_x[7];
   out_5573080380081729814[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8497018896580574730) {
   out_8497018896580574730[0] = 1.0;
   out_8497018896580574730[1] = 0;
   out_8497018896580574730[2] = 0;
   out_8497018896580574730[3] = 0;
   out_8497018896580574730[4] = 0;
   out_8497018896580574730[5] = 0;
   out_8497018896580574730[6] = 0;
   out_8497018896580574730[7] = 0;
   out_8497018896580574730[8] = 0;
   out_8497018896580574730[9] = 0;
   out_8497018896580574730[10] = 1.0;
   out_8497018896580574730[11] = 0;
   out_8497018896580574730[12] = 0;
   out_8497018896580574730[13] = 0;
   out_8497018896580574730[14] = 0;
   out_8497018896580574730[15] = 0;
   out_8497018896580574730[16] = 0;
   out_8497018896580574730[17] = 0;
   out_8497018896580574730[18] = 0;
   out_8497018896580574730[19] = 0;
   out_8497018896580574730[20] = 1.0;
   out_8497018896580574730[21] = 0;
   out_8497018896580574730[22] = 0;
   out_8497018896580574730[23] = 0;
   out_8497018896580574730[24] = 0;
   out_8497018896580574730[25] = 0;
   out_8497018896580574730[26] = 0;
   out_8497018896580574730[27] = 0;
   out_8497018896580574730[28] = 0;
   out_8497018896580574730[29] = 0;
   out_8497018896580574730[30] = 1.0;
   out_8497018896580574730[31] = 0;
   out_8497018896580574730[32] = 0;
   out_8497018896580574730[33] = 0;
   out_8497018896580574730[34] = 0;
   out_8497018896580574730[35] = 0;
   out_8497018896580574730[36] = 0;
   out_8497018896580574730[37] = 0;
   out_8497018896580574730[38] = 0;
   out_8497018896580574730[39] = 0;
   out_8497018896580574730[40] = 1.0;
   out_8497018896580574730[41] = 0;
   out_8497018896580574730[42] = 0;
   out_8497018896580574730[43] = 0;
   out_8497018896580574730[44] = 0;
   out_8497018896580574730[45] = 0;
   out_8497018896580574730[46] = 0;
   out_8497018896580574730[47] = 0;
   out_8497018896580574730[48] = 0;
   out_8497018896580574730[49] = 0;
   out_8497018896580574730[50] = 1.0;
   out_8497018896580574730[51] = 0;
   out_8497018896580574730[52] = 0;
   out_8497018896580574730[53] = 0;
   out_8497018896580574730[54] = 0;
   out_8497018896580574730[55] = 0;
   out_8497018896580574730[56] = 0;
   out_8497018896580574730[57] = 0;
   out_8497018896580574730[58] = 0;
   out_8497018896580574730[59] = 0;
   out_8497018896580574730[60] = 1.0;
   out_8497018896580574730[61] = 0;
   out_8497018896580574730[62] = 0;
   out_8497018896580574730[63] = 0;
   out_8497018896580574730[64] = 0;
   out_8497018896580574730[65] = 0;
   out_8497018896580574730[66] = 0;
   out_8497018896580574730[67] = 0;
   out_8497018896580574730[68] = 0;
   out_8497018896580574730[69] = 0;
   out_8497018896580574730[70] = 1.0;
   out_8497018896580574730[71] = 0;
   out_8497018896580574730[72] = 0;
   out_8497018896580574730[73] = 0;
   out_8497018896580574730[74] = 0;
   out_8497018896580574730[75] = 0;
   out_8497018896580574730[76] = 0;
   out_8497018896580574730[77] = 0;
   out_8497018896580574730[78] = 0;
   out_8497018896580574730[79] = 0;
   out_8497018896580574730[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6343601988785741941) {
   out_6343601988785741941[0] = state[0];
   out_6343601988785741941[1] = state[1];
   out_6343601988785741941[2] = state[2];
   out_6343601988785741941[3] = state[3];
   out_6343601988785741941[4] = state[4];
   out_6343601988785741941[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6343601988785741941[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6343601988785741941[7] = state[7];
   out_6343601988785741941[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2898252461551912388) {
   out_2898252461551912388[0] = 1;
   out_2898252461551912388[1] = 0;
   out_2898252461551912388[2] = 0;
   out_2898252461551912388[3] = 0;
   out_2898252461551912388[4] = 0;
   out_2898252461551912388[5] = 0;
   out_2898252461551912388[6] = 0;
   out_2898252461551912388[7] = 0;
   out_2898252461551912388[8] = 0;
   out_2898252461551912388[9] = 0;
   out_2898252461551912388[10] = 1;
   out_2898252461551912388[11] = 0;
   out_2898252461551912388[12] = 0;
   out_2898252461551912388[13] = 0;
   out_2898252461551912388[14] = 0;
   out_2898252461551912388[15] = 0;
   out_2898252461551912388[16] = 0;
   out_2898252461551912388[17] = 0;
   out_2898252461551912388[18] = 0;
   out_2898252461551912388[19] = 0;
   out_2898252461551912388[20] = 1;
   out_2898252461551912388[21] = 0;
   out_2898252461551912388[22] = 0;
   out_2898252461551912388[23] = 0;
   out_2898252461551912388[24] = 0;
   out_2898252461551912388[25] = 0;
   out_2898252461551912388[26] = 0;
   out_2898252461551912388[27] = 0;
   out_2898252461551912388[28] = 0;
   out_2898252461551912388[29] = 0;
   out_2898252461551912388[30] = 1;
   out_2898252461551912388[31] = 0;
   out_2898252461551912388[32] = 0;
   out_2898252461551912388[33] = 0;
   out_2898252461551912388[34] = 0;
   out_2898252461551912388[35] = 0;
   out_2898252461551912388[36] = 0;
   out_2898252461551912388[37] = 0;
   out_2898252461551912388[38] = 0;
   out_2898252461551912388[39] = 0;
   out_2898252461551912388[40] = 1;
   out_2898252461551912388[41] = 0;
   out_2898252461551912388[42] = 0;
   out_2898252461551912388[43] = 0;
   out_2898252461551912388[44] = 0;
   out_2898252461551912388[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2898252461551912388[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2898252461551912388[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2898252461551912388[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2898252461551912388[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2898252461551912388[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2898252461551912388[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2898252461551912388[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2898252461551912388[53] = -9.8000000000000007*dt;
   out_2898252461551912388[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2898252461551912388[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2898252461551912388[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2898252461551912388[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2898252461551912388[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2898252461551912388[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2898252461551912388[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2898252461551912388[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2898252461551912388[62] = 0;
   out_2898252461551912388[63] = 0;
   out_2898252461551912388[64] = 0;
   out_2898252461551912388[65] = 0;
   out_2898252461551912388[66] = 0;
   out_2898252461551912388[67] = 0;
   out_2898252461551912388[68] = 0;
   out_2898252461551912388[69] = 0;
   out_2898252461551912388[70] = 1;
   out_2898252461551912388[71] = 0;
   out_2898252461551912388[72] = 0;
   out_2898252461551912388[73] = 0;
   out_2898252461551912388[74] = 0;
   out_2898252461551912388[75] = 0;
   out_2898252461551912388[76] = 0;
   out_2898252461551912388[77] = 0;
   out_2898252461551912388[78] = 0;
   out_2898252461551912388[79] = 0;
   out_2898252461551912388[80] = 1;
}
void h_25(double *state, double *unused, double *out_6064530161557546717) {
   out_6064530161557546717[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4120468543408452751) {
   out_4120468543408452751[0] = 0;
   out_4120468543408452751[1] = 0;
   out_4120468543408452751[2] = 0;
   out_4120468543408452751[3] = 0;
   out_4120468543408452751[4] = 0;
   out_4120468543408452751[5] = 0;
   out_4120468543408452751[6] = 1;
   out_4120468543408452751[7] = 0;
   out_4120468543408452751[8] = 0;
}
void h_24(double *state, double *unused, double *out_7787437801929080086) {
   out_7787437801929080086[0] = state[4];
   out_7787437801929080086[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1947818944402953185) {
   out_1947818944402953185[0] = 0;
   out_1947818944402953185[1] = 0;
   out_1947818944402953185[2] = 0;
   out_1947818944402953185[3] = 0;
   out_1947818944402953185[4] = 1;
   out_1947818944402953185[5] = 0;
   out_1947818944402953185[6] = 0;
   out_1947818944402953185[7] = 0;
   out_1947818944402953185[8] = 0;
   out_1947818944402953185[9] = 0;
   out_1947818944402953185[10] = 0;
   out_1947818944402953185[11] = 0;
   out_1947818944402953185[12] = 0;
   out_1947818944402953185[13] = 0;
   out_1947818944402953185[14] = 1;
   out_1947818944402953185[15] = 0;
   out_1947818944402953185[16] = 0;
   out_1947818944402953185[17] = 0;
}
void h_30(double *state, double *unused, double *out_8744764383897982549) {
   out_8744764383897982549[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6638801501915701378) {
   out_6638801501915701378[0] = 0;
   out_6638801501915701378[1] = 0;
   out_6638801501915701378[2] = 0;
   out_6638801501915701378[3] = 0;
   out_6638801501915701378[4] = 1;
   out_6638801501915701378[5] = 0;
   out_6638801501915701378[6] = 0;
   out_6638801501915701378[7] = 0;
   out_6638801501915701378[8] = 0;
}
void h_26(double *state, double *unused, double *out_5707978332453649864) {
   out_5707978332453649864[0] = state[7];
}
void H_26(double *state, double *unused, double *out_378965224534396527) {
   out_378965224534396527[0] = 0;
   out_378965224534396527[1] = 0;
   out_378965224534396527[2] = 0;
   out_378965224534396527[3] = 0;
   out_378965224534396527[4] = 0;
   out_378965224534396527[5] = 0;
   out_378965224534396527[6] = 0;
   out_378965224534396527[7] = 1;
   out_378965224534396527[8] = 0;
}
void h_27(double *state, double *unused, double *out_6667846046234024234) {
   out_6667846046234024234[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8862395573099644595) {
   out_8862395573099644595[0] = 0;
   out_8862395573099644595[1] = 0;
   out_8862395573099644595[2] = 0;
   out_8862395573099644595[3] = 1;
   out_8862395573099644595[4] = 0;
   out_8862395573099644595[5] = 0;
   out_8862395573099644595[6] = 0;
   out_8862395573099644595[7] = 0;
   out_8862395573099644595[8] = 0;
}
void h_29(double *state, double *unused, double *out_8141448499221505032) {
   out_8141448499221505032[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7149032846230093562) {
   out_7149032846230093562[0] = 0;
   out_7149032846230093562[1] = 1;
   out_7149032846230093562[2] = 0;
   out_7149032846230093562[3] = 0;
   out_7149032846230093562[4] = 0;
   out_7149032846230093562[5] = 0;
   out_7149032846230093562[6] = 0;
   out_7149032846230093562[7] = 0;
   out_7149032846230093562[8] = 0;
}
void h_28(double *state, double *unused, double *out_2314123125772816531) {
   out_2314123125772816531[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2066633829160562988) {
   out_2066633829160562988[0] = 1;
   out_2066633829160562988[1] = 0;
   out_2066633829160562988[2] = 0;
   out_2066633829160562988[3] = 0;
   out_2066633829160562988[4] = 0;
   out_2066633829160562988[5] = 0;
   out_2066633829160562988[6] = 0;
   out_2066633829160562988[7] = 0;
   out_2066633829160562988[8] = 0;
}
void h_31(double *state, double *unused, double *out_5789336099273040828) {
   out_5789336099273040828[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4151114505285413179) {
   out_4151114505285413179[0] = 0;
   out_4151114505285413179[1] = 0;
   out_4151114505285413179[2] = 0;
   out_4151114505285413179[3] = 0;
   out_4151114505285413179[4] = 0;
   out_4151114505285413179[5] = 0;
   out_4151114505285413179[6] = 0;
   out_4151114505285413179[7] = 0;
   out_4151114505285413179[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5261127957076131017) {
  err_fun(nom_x, delta_x, out_5261127957076131017);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5573080380081729814) {
  inv_err_fun(nom_x, true_x, out_5573080380081729814);
}
void car_H_mod_fun(double *state, double *out_8497018896580574730) {
  H_mod_fun(state, out_8497018896580574730);
}
void car_f_fun(double *state, double dt, double *out_6343601988785741941) {
  f_fun(state,  dt, out_6343601988785741941);
}
void car_F_fun(double *state, double dt, double *out_2898252461551912388) {
  F_fun(state,  dt, out_2898252461551912388);
}
void car_h_25(double *state, double *unused, double *out_6064530161557546717) {
  h_25(state, unused, out_6064530161557546717);
}
void car_H_25(double *state, double *unused, double *out_4120468543408452751) {
  H_25(state, unused, out_4120468543408452751);
}
void car_h_24(double *state, double *unused, double *out_7787437801929080086) {
  h_24(state, unused, out_7787437801929080086);
}
void car_H_24(double *state, double *unused, double *out_1947818944402953185) {
  H_24(state, unused, out_1947818944402953185);
}
void car_h_30(double *state, double *unused, double *out_8744764383897982549) {
  h_30(state, unused, out_8744764383897982549);
}
void car_H_30(double *state, double *unused, double *out_6638801501915701378) {
  H_30(state, unused, out_6638801501915701378);
}
void car_h_26(double *state, double *unused, double *out_5707978332453649864) {
  h_26(state, unused, out_5707978332453649864);
}
void car_H_26(double *state, double *unused, double *out_378965224534396527) {
  H_26(state, unused, out_378965224534396527);
}
void car_h_27(double *state, double *unused, double *out_6667846046234024234) {
  h_27(state, unused, out_6667846046234024234);
}
void car_H_27(double *state, double *unused, double *out_8862395573099644595) {
  H_27(state, unused, out_8862395573099644595);
}
void car_h_29(double *state, double *unused, double *out_8141448499221505032) {
  h_29(state, unused, out_8141448499221505032);
}
void car_H_29(double *state, double *unused, double *out_7149032846230093562) {
  H_29(state, unused, out_7149032846230093562);
}
void car_h_28(double *state, double *unused, double *out_2314123125772816531) {
  h_28(state, unused, out_2314123125772816531);
}
void car_H_28(double *state, double *unused, double *out_2066633829160562988) {
  H_28(state, unused, out_2066633829160562988);
}
void car_h_31(double *state, double *unused, double *out_5789336099273040828) {
  h_31(state, unused, out_5789336099273040828);
}
void car_H_31(double *state, double *unused, double *out_4151114505285413179) {
  H_31(state, unused, out_4151114505285413179);
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
