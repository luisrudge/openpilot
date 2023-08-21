#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_5261127957076131017);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5573080380081729814);
void car_H_mod_fun(double *state, double *out_8497018896580574730);
void car_f_fun(double *state, double dt, double *out_6343601988785741941);
void car_F_fun(double *state, double dt, double *out_2898252461551912388);
void car_h_25(double *state, double *unused, double *out_6064530161557546717);
void car_H_25(double *state, double *unused, double *out_4120468543408452751);
void car_h_24(double *state, double *unused, double *out_7787437801929080086);
void car_H_24(double *state, double *unused, double *out_1947818944402953185);
void car_h_30(double *state, double *unused, double *out_8744764383897982549);
void car_H_30(double *state, double *unused, double *out_6638801501915701378);
void car_h_26(double *state, double *unused, double *out_5707978332453649864);
void car_H_26(double *state, double *unused, double *out_378965224534396527);
void car_h_27(double *state, double *unused, double *out_6667846046234024234);
void car_H_27(double *state, double *unused, double *out_8862395573099644595);
void car_h_29(double *state, double *unused, double *out_8141448499221505032);
void car_H_29(double *state, double *unused, double *out_7149032846230093562);
void car_h_28(double *state, double *unused, double *out_2314123125772816531);
void car_H_28(double *state, double *unused, double *out_2066633829160562988);
void car_h_31(double *state, double *unused, double *out_5789336099273040828);
void car_H_31(double *state, double *unused, double *out_4151114505285413179);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}