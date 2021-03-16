#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-03-15 23:02:08
LastEditors: Wei Luo
LastEditTime: 2021-03-16 00:21:49
Note: Note
'''

import os
import sys
import shutil
import errno

from mobile_robot_model import MobileRobotModel
from acados_template import AcadosOcp, AcadosOcpSolver

import casadi as ca
import numpy as np
import scipy.linalg


def safe_mkdir_recursive(directory, overwrite=False):
    if not os.path.exists(directory):
        try:
            os.makedirs(directory)
        except OSError as exc:
            if exc.errno == errno.EEXIST and os.path.isdir(directory):
                pass
            else:
                raise
    else:
        if overwrite:
            try:
                shutil.rmtree(directory)
            except:
                print('Error while removing directory {}'.format(directory))


class MobileRobotOptimizer(object):
    def __init__(self, m_model, t_horizon, n_nodes):
        model = m_model
        self.T = t_horizon
        self.N = n_nodes

        # Ensure current working directory is current folder
        os.chdir(os.path.dirname(os.path.realpath(__file__)))
        self.acados_models_dir = './acados_models'
        safe_mkdir_recursive(os.path.join(os.getcwd(), self.acados_models_dir))
        acados_source_path = os.environ['ACADOS_SOURCE_DIR']
        sys.path.insert(0, acados_source_path)

        nx = model.x.size()[0]
        nu = model.u.size()[0]
        ny = nx + nu
        n_params = len(model.p)

        # create OCP
        ocp = AcadosOcp()
        ocp.acados_include_path = acados_source_path + '/include'
        ocp.acados_lib_path = acados_source_path + '/lib'
        ocp.model = model
        ocp.dims.N = self.N
        ocp.solver_options.tf = self.T

        # initialize parameters
        ocp.dims.np = n_params
        ocp.parameter_values = np.zeros(n_params)

        # cost type
        Q = np.array([[1.0, 0.0, 0.0], [0.0, 5.0, 0.0], [0.0, 0.0, .1]])
        R = np.array([[0.5, 0.0], [0.0, 0.05]])
        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.W = scipy.linalg.block_diag(Q, R)
        ocp.cost.Vx = np.zeros((ny, nx))
        ocp.cost.Vx[:nx, :nx] = np.eye(nx)
        ocp.cost.Vu = np.zeros((ny, nu))
        ocp.cost.Vu[-nu:, -nu:] = np.eye(nu)

        # set constraints
        v_max = 0.6
        omega_max = np.pi/4.0
        ocp.constraints.lbu = np.array([-v_max, -omega_max])
        ocp.constraints.ubu = np.array([v_max, omega_max])
        ocp.constraints.idxbu = np.array([0, 1])

        x_ref = np.zeros(nx)
        u_ref = np.zeros(nu)
        # initial state
        ocp.constraints.x0 = x_ref
        ocp.cost.yref = np.concatenate((x_ref, u_ref))

        # solver options
        ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        # explicit Runge-Kutta integrator
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = 'SQP'

        # compile acados ocp
        json_file = os.path.join('./'+model.name+'_acados_ocp.json')
        solver = AcadosOcpSolver(ocp, json_file=json_file)


if __name__ == '__main__':
    mobile_robot_model = MobileRobotModel()
    opt = MobileRobotOptimizer(m_model=mobile_robot_model.model,
                               m_constraint=mobile_robot_model.constraint, t_horizon=20, n_nodes=100)
