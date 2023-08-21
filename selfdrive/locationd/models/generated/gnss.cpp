#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_9210557482145146948) {
   out_9210557482145146948[0] = delta_x[0] + nom_x[0];
   out_9210557482145146948[1] = delta_x[1] + nom_x[1];
   out_9210557482145146948[2] = delta_x[2] + nom_x[2];
   out_9210557482145146948[3] = delta_x[3] + nom_x[3];
   out_9210557482145146948[4] = delta_x[4] + nom_x[4];
   out_9210557482145146948[5] = delta_x[5] + nom_x[5];
   out_9210557482145146948[6] = delta_x[6] + nom_x[6];
   out_9210557482145146948[7] = delta_x[7] + nom_x[7];
   out_9210557482145146948[8] = delta_x[8] + nom_x[8];
   out_9210557482145146948[9] = delta_x[9] + nom_x[9];
   out_9210557482145146948[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_462516189378662857) {
   out_462516189378662857[0] = -nom_x[0] + true_x[0];
   out_462516189378662857[1] = -nom_x[1] + true_x[1];
   out_462516189378662857[2] = -nom_x[2] + true_x[2];
   out_462516189378662857[3] = -nom_x[3] + true_x[3];
   out_462516189378662857[4] = -nom_x[4] + true_x[4];
   out_462516189378662857[5] = -nom_x[5] + true_x[5];
   out_462516189378662857[6] = -nom_x[6] + true_x[6];
   out_462516189378662857[7] = -nom_x[7] + true_x[7];
   out_462516189378662857[8] = -nom_x[8] + true_x[8];
   out_462516189378662857[9] = -nom_x[9] + true_x[9];
   out_462516189378662857[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_42018900835236910) {
   out_42018900835236910[0] = 1.0;
   out_42018900835236910[1] = 0;
   out_42018900835236910[2] = 0;
   out_42018900835236910[3] = 0;
   out_42018900835236910[4] = 0;
   out_42018900835236910[5] = 0;
   out_42018900835236910[6] = 0;
   out_42018900835236910[7] = 0;
   out_42018900835236910[8] = 0;
   out_42018900835236910[9] = 0;
   out_42018900835236910[10] = 0;
   out_42018900835236910[11] = 0;
   out_42018900835236910[12] = 1.0;
   out_42018900835236910[13] = 0;
   out_42018900835236910[14] = 0;
   out_42018900835236910[15] = 0;
   out_42018900835236910[16] = 0;
   out_42018900835236910[17] = 0;
   out_42018900835236910[18] = 0;
   out_42018900835236910[19] = 0;
   out_42018900835236910[20] = 0;
   out_42018900835236910[21] = 0;
   out_42018900835236910[22] = 0;
   out_42018900835236910[23] = 0;
   out_42018900835236910[24] = 1.0;
   out_42018900835236910[25] = 0;
   out_42018900835236910[26] = 0;
   out_42018900835236910[27] = 0;
   out_42018900835236910[28] = 0;
   out_42018900835236910[29] = 0;
   out_42018900835236910[30] = 0;
   out_42018900835236910[31] = 0;
   out_42018900835236910[32] = 0;
   out_42018900835236910[33] = 0;
   out_42018900835236910[34] = 0;
   out_42018900835236910[35] = 0;
   out_42018900835236910[36] = 1.0;
   out_42018900835236910[37] = 0;
   out_42018900835236910[38] = 0;
   out_42018900835236910[39] = 0;
   out_42018900835236910[40] = 0;
   out_42018900835236910[41] = 0;
   out_42018900835236910[42] = 0;
   out_42018900835236910[43] = 0;
   out_42018900835236910[44] = 0;
   out_42018900835236910[45] = 0;
   out_42018900835236910[46] = 0;
   out_42018900835236910[47] = 0;
   out_42018900835236910[48] = 1.0;
   out_42018900835236910[49] = 0;
   out_42018900835236910[50] = 0;
   out_42018900835236910[51] = 0;
   out_42018900835236910[52] = 0;
   out_42018900835236910[53] = 0;
   out_42018900835236910[54] = 0;
   out_42018900835236910[55] = 0;
   out_42018900835236910[56] = 0;
   out_42018900835236910[57] = 0;
   out_42018900835236910[58] = 0;
   out_42018900835236910[59] = 0;
   out_42018900835236910[60] = 1.0;
   out_42018900835236910[61] = 0;
   out_42018900835236910[62] = 0;
   out_42018900835236910[63] = 0;
   out_42018900835236910[64] = 0;
   out_42018900835236910[65] = 0;
   out_42018900835236910[66] = 0;
   out_42018900835236910[67] = 0;
   out_42018900835236910[68] = 0;
   out_42018900835236910[69] = 0;
   out_42018900835236910[70] = 0;
   out_42018900835236910[71] = 0;
   out_42018900835236910[72] = 1.0;
   out_42018900835236910[73] = 0;
   out_42018900835236910[74] = 0;
   out_42018900835236910[75] = 0;
   out_42018900835236910[76] = 0;
   out_42018900835236910[77] = 0;
   out_42018900835236910[78] = 0;
   out_42018900835236910[79] = 0;
   out_42018900835236910[80] = 0;
   out_42018900835236910[81] = 0;
   out_42018900835236910[82] = 0;
   out_42018900835236910[83] = 0;
   out_42018900835236910[84] = 1.0;
   out_42018900835236910[85] = 0;
   out_42018900835236910[86] = 0;
   out_42018900835236910[87] = 0;
   out_42018900835236910[88] = 0;
   out_42018900835236910[89] = 0;
   out_42018900835236910[90] = 0;
   out_42018900835236910[91] = 0;
   out_42018900835236910[92] = 0;
   out_42018900835236910[93] = 0;
   out_42018900835236910[94] = 0;
   out_42018900835236910[95] = 0;
   out_42018900835236910[96] = 1.0;
   out_42018900835236910[97] = 0;
   out_42018900835236910[98] = 0;
   out_42018900835236910[99] = 0;
   out_42018900835236910[100] = 0;
   out_42018900835236910[101] = 0;
   out_42018900835236910[102] = 0;
   out_42018900835236910[103] = 0;
   out_42018900835236910[104] = 0;
   out_42018900835236910[105] = 0;
   out_42018900835236910[106] = 0;
   out_42018900835236910[107] = 0;
   out_42018900835236910[108] = 1.0;
   out_42018900835236910[109] = 0;
   out_42018900835236910[110] = 0;
   out_42018900835236910[111] = 0;
   out_42018900835236910[112] = 0;
   out_42018900835236910[113] = 0;
   out_42018900835236910[114] = 0;
   out_42018900835236910[115] = 0;
   out_42018900835236910[116] = 0;
   out_42018900835236910[117] = 0;
   out_42018900835236910[118] = 0;
   out_42018900835236910[119] = 0;
   out_42018900835236910[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_5311281802349817452) {
   out_5311281802349817452[0] = dt*state[3] + state[0];
   out_5311281802349817452[1] = dt*state[4] + state[1];
   out_5311281802349817452[2] = dt*state[5] + state[2];
   out_5311281802349817452[3] = state[3];
   out_5311281802349817452[4] = state[4];
   out_5311281802349817452[5] = state[5];
   out_5311281802349817452[6] = dt*state[7] + state[6];
   out_5311281802349817452[7] = dt*state[8] + state[7];
   out_5311281802349817452[8] = state[8];
   out_5311281802349817452[9] = state[9];
   out_5311281802349817452[10] = state[10];
}
void F_fun(double *state, double dt, double *out_6859155043241242449) {
   out_6859155043241242449[0] = 1;
   out_6859155043241242449[1] = 0;
   out_6859155043241242449[2] = 0;
   out_6859155043241242449[3] = dt;
   out_6859155043241242449[4] = 0;
   out_6859155043241242449[5] = 0;
   out_6859155043241242449[6] = 0;
   out_6859155043241242449[7] = 0;
   out_6859155043241242449[8] = 0;
   out_6859155043241242449[9] = 0;
   out_6859155043241242449[10] = 0;
   out_6859155043241242449[11] = 0;
   out_6859155043241242449[12] = 1;
   out_6859155043241242449[13] = 0;
   out_6859155043241242449[14] = 0;
   out_6859155043241242449[15] = dt;
   out_6859155043241242449[16] = 0;
   out_6859155043241242449[17] = 0;
   out_6859155043241242449[18] = 0;
   out_6859155043241242449[19] = 0;
   out_6859155043241242449[20] = 0;
   out_6859155043241242449[21] = 0;
   out_6859155043241242449[22] = 0;
   out_6859155043241242449[23] = 0;
   out_6859155043241242449[24] = 1;
   out_6859155043241242449[25] = 0;
   out_6859155043241242449[26] = 0;
   out_6859155043241242449[27] = dt;
   out_6859155043241242449[28] = 0;
   out_6859155043241242449[29] = 0;
   out_6859155043241242449[30] = 0;
   out_6859155043241242449[31] = 0;
   out_6859155043241242449[32] = 0;
   out_6859155043241242449[33] = 0;
   out_6859155043241242449[34] = 0;
   out_6859155043241242449[35] = 0;
   out_6859155043241242449[36] = 1;
   out_6859155043241242449[37] = 0;
   out_6859155043241242449[38] = 0;
   out_6859155043241242449[39] = 0;
   out_6859155043241242449[40] = 0;
   out_6859155043241242449[41] = 0;
   out_6859155043241242449[42] = 0;
   out_6859155043241242449[43] = 0;
   out_6859155043241242449[44] = 0;
   out_6859155043241242449[45] = 0;
   out_6859155043241242449[46] = 0;
   out_6859155043241242449[47] = 0;
   out_6859155043241242449[48] = 1;
   out_6859155043241242449[49] = 0;
   out_6859155043241242449[50] = 0;
   out_6859155043241242449[51] = 0;
   out_6859155043241242449[52] = 0;
   out_6859155043241242449[53] = 0;
   out_6859155043241242449[54] = 0;
   out_6859155043241242449[55] = 0;
   out_6859155043241242449[56] = 0;
   out_6859155043241242449[57] = 0;
   out_6859155043241242449[58] = 0;
   out_6859155043241242449[59] = 0;
   out_6859155043241242449[60] = 1;
   out_6859155043241242449[61] = 0;
   out_6859155043241242449[62] = 0;
   out_6859155043241242449[63] = 0;
   out_6859155043241242449[64] = 0;
   out_6859155043241242449[65] = 0;
   out_6859155043241242449[66] = 0;
   out_6859155043241242449[67] = 0;
   out_6859155043241242449[68] = 0;
   out_6859155043241242449[69] = 0;
   out_6859155043241242449[70] = 0;
   out_6859155043241242449[71] = 0;
   out_6859155043241242449[72] = 1;
   out_6859155043241242449[73] = dt;
   out_6859155043241242449[74] = 0;
   out_6859155043241242449[75] = 0;
   out_6859155043241242449[76] = 0;
   out_6859155043241242449[77] = 0;
   out_6859155043241242449[78] = 0;
   out_6859155043241242449[79] = 0;
   out_6859155043241242449[80] = 0;
   out_6859155043241242449[81] = 0;
   out_6859155043241242449[82] = 0;
   out_6859155043241242449[83] = 0;
   out_6859155043241242449[84] = 1;
   out_6859155043241242449[85] = dt;
   out_6859155043241242449[86] = 0;
   out_6859155043241242449[87] = 0;
   out_6859155043241242449[88] = 0;
   out_6859155043241242449[89] = 0;
   out_6859155043241242449[90] = 0;
   out_6859155043241242449[91] = 0;
   out_6859155043241242449[92] = 0;
   out_6859155043241242449[93] = 0;
   out_6859155043241242449[94] = 0;
   out_6859155043241242449[95] = 0;
   out_6859155043241242449[96] = 1;
   out_6859155043241242449[97] = 0;
   out_6859155043241242449[98] = 0;
   out_6859155043241242449[99] = 0;
   out_6859155043241242449[100] = 0;
   out_6859155043241242449[101] = 0;
   out_6859155043241242449[102] = 0;
   out_6859155043241242449[103] = 0;
   out_6859155043241242449[104] = 0;
   out_6859155043241242449[105] = 0;
   out_6859155043241242449[106] = 0;
   out_6859155043241242449[107] = 0;
   out_6859155043241242449[108] = 1;
   out_6859155043241242449[109] = 0;
   out_6859155043241242449[110] = 0;
   out_6859155043241242449[111] = 0;
   out_6859155043241242449[112] = 0;
   out_6859155043241242449[113] = 0;
   out_6859155043241242449[114] = 0;
   out_6859155043241242449[115] = 0;
   out_6859155043241242449[116] = 0;
   out_6859155043241242449[117] = 0;
   out_6859155043241242449[118] = 0;
   out_6859155043241242449[119] = 0;
   out_6859155043241242449[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_7992352266958398705) {
   out_7992352266958398705[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_1846418363923402085) {
   out_1846418363923402085[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1846418363923402085[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1846418363923402085[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1846418363923402085[3] = 0;
   out_1846418363923402085[4] = 0;
   out_1846418363923402085[5] = 0;
   out_1846418363923402085[6] = 1;
   out_1846418363923402085[7] = 0;
   out_1846418363923402085[8] = 0;
   out_1846418363923402085[9] = 0;
   out_1846418363923402085[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_523115360813020629) {
   out_523115360813020629[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_8069596900911168247) {
   out_8069596900911168247[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8069596900911168247[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8069596900911168247[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8069596900911168247[3] = 0;
   out_8069596900911168247[4] = 0;
   out_8069596900911168247[5] = 0;
   out_8069596900911168247[6] = 1;
   out_8069596900911168247[7] = 0;
   out_8069596900911168247[8] = 0;
   out_8069596900911168247[9] = 1;
   out_8069596900911168247[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_7561025526105128004) {
   out_7561025526105128004[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_1972210164998050797) {
   out_1972210164998050797[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1972210164998050797[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1972210164998050797[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1972210164998050797[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1972210164998050797[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1972210164998050797[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1972210164998050797[6] = 0;
   out_1972210164998050797[7] = 1;
   out_1972210164998050797[8] = 0;
   out_1972210164998050797[9] = 0;
   out_1972210164998050797[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_7561025526105128004) {
   out_7561025526105128004[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_1972210164998050797) {
   out_1972210164998050797[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1972210164998050797[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1972210164998050797[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1972210164998050797[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1972210164998050797[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1972210164998050797[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1972210164998050797[6] = 0;
   out_1972210164998050797[7] = 1;
   out_1972210164998050797[8] = 0;
   out_1972210164998050797[9] = 0;
   out_1972210164998050797[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9210557482145146948) {
  err_fun(nom_x, delta_x, out_9210557482145146948);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_462516189378662857) {
  inv_err_fun(nom_x, true_x, out_462516189378662857);
}
void gnss_H_mod_fun(double *state, double *out_42018900835236910) {
  H_mod_fun(state, out_42018900835236910);
}
void gnss_f_fun(double *state, double dt, double *out_5311281802349817452) {
  f_fun(state,  dt, out_5311281802349817452);
}
void gnss_F_fun(double *state, double dt, double *out_6859155043241242449) {
  F_fun(state,  dt, out_6859155043241242449);
}
void gnss_h_6(double *state, double *sat_pos, double *out_7992352266958398705) {
  h_6(state, sat_pos, out_7992352266958398705);
}
void gnss_H_6(double *state, double *sat_pos, double *out_1846418363923402085) {
  H_6(state, sat_pos, out_1846418363923402085);
}
void gnss_h_20(double *state, double *sat_pos, double *out_523115360813020629) {
  h_20(state, sat_pos, out_523115360813020629);
}
void gnss_H_20(double *state, double *sat_pos, double *out_8069596900911168247) {
  H_20(state, sat_pos, out_8069596900911168247);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_7561025526105128004) {
  h_7(state, sat_pos_vel, out_7561025526105128004);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_1972210164998050797) {
  H_7(state, sat_pos_vel, out_1972210164998050797);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_7561025526105128004) {
  h_21(state, sat_pos_vel, out_7561025526105128004);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_1972210164998050797) {
  H_21(state, sat_pos_vel, out_1972210164998050797);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
