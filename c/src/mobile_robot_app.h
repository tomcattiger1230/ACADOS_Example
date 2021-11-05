/*
 * @Author: Wei Luo
 * @Date: 2021-03-18 14:08:29
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-11-05 16:57:05
 * @Note: Note
 */
//
// Created by Wei Luo on 2021/3/18.
//

#ifndef ACADOS_TEST_MOBILE_ROBOT_APP_H
#define ACADOS_TEST_MOBILE_ROBOT_APP_H

#include <chrono>
#include <stdio.h>
#include <stdlib.h>

#include <Eigen/Dense>

#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"

#include "acados_sim_solver_mobile_robot.h"
#include "acados_solver_mobile_robot.h"

int status; // acados operation state

int idxbx0[3];

double min_time = 1e12;
double elapsed_time;

double x_target[3];
double x_state[5];
double x_current[3];
double u_current[3];
int N;
int nx;
int nu;

#endif // ACADOS_TEST_MOBILE_ROBOT_APP_H
