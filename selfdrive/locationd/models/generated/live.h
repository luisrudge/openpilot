#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_5763660217246419227);
void live_err_fun(double *nom_x, double *delta_x, double *out_7984073142007517775);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4505929681771980701);
void live_H_mod_fun(double *state, double *out_1600126465576552008);
void live_f_fun(double *state, double dt, double *out_4714363137519275674);
void live_F_fun(double *state, double dt, double *out_6365999196697241563);
void live_h_4(double *state, double *unused, double *out_8654380828745342908);
void live_H_4(double *state, double *unused, double *out_1277555590207695814);
void live_h_9(double *state, double *unused, double *out_7516788848090321369);
void live_H_9(double *state, double *unused, double *out_1036365943578105169);
void live_h_10(double *state, double *unused, double *out_5089843156482325694);
void live_H_10(double *state, double *unused, double *out_1789498124346718583);
void live_h_12(double *state, double *unused, double *out_2468563075831269207);
void live_H_12(double *state, double *unused, double *out_3741900817824265981);
void live_h_35(double *state, double *unused, double *out_9035519326057673557);
void live_H_35(double *state, double *unused, double *out_6487463850149279690);
void live_h_32(double *state, double *unused, double *out_2998224959805740489);
void live_H_32(double *state, double *unused, double *out_2220271252105652031);
void live_h_13(double *state, double *unused, double *out_3624310307560900385);
void live_H_13(double *state, double *unused, double *out_646674378183793147);
void live_h_14(double *state, double *unused, double *out_7516788848090321369);
void live_H_14(double *state, double *unused, double *out_1036365943578105169);
void live_h_33(double *state, double *unused, double *out_5817128039764809933);
void live_H_33(double *state, double *unused, double *out_5239663471803769166);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}