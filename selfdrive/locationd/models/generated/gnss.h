#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9210557482145146948);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_462516189378662857);
void gnss_H_mod_fun(double *state, double *out_42018900835236910);
void gnss_f_fun(double *state, double dt, double *out_5311281802349817452);
void gnss_F_fun(double *state, double dt, double *out_6859155043241242449);
void gnss_h_6(double *state, double *sat_pos, double *out_7992352266958398705);
void gnss_H_6(double *state, double *sat_pos, double *out_1846418363923402085);
void gnss_h_20(double *state, double *sat_pos, double *out_523115360813020629);
void gnss_H_20(double *state, double *sat_pos, double *out_8069596900911168247);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_7561025526105128004);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_1972210164998050797);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_7561025526105128004);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_1972210164998050797);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}