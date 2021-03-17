#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-03-15 22:43:48
LastEditors: Wei Luo
LastEditTime: 2021-03-17 23:28:55
Note: Note
'''

import numpy as np
import casadi as ca
from acados_template import AcadosModel

class MobileRobotModel(object):
    def __init__(self,):
        model = AcadosModel() #  ca.types.SimpleNamespace()
        constraint = ca.types.SimpleNamespace()
        # control inputs
        v = ca.SX.sym('v')
        omega = ca.SX.sym('omega')
        controls = ca.vertcat(v, omega)
        # n_controls = controls.size()[0]
        # model states
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(x, y, theta)

        rhs = [v*ca.cos(theta), v*ca.sin(theta), omega]

        # function
        f = ca.Function('f', [states, controls], [ca.vcat(rhs)], ['state', 'control_input'], ['rhs'])
        # f_expl = ca.vcat(rhs)
        # acados model
        x_dot = ca.SX.sym('x_dot', len(rhs))
        f_impl = x_dot - f(states, controls)

        model.f_expl_expr = f(states, controls)
        model.f_impl_expr = f_impl
        model.x = states
        model.xdot = x_dot
        model.u = controls
        model.p = []
        model.name = 'mobile_robot'

        # constraint
        constraint.v_max = 0.6
        constraint.v_min = -0.6
        constraint.omega_max = np.pi/4.0
        constraint.omega_min = -np.pi/4.0
        constraint.x_min = -2.
        constraint.x_max = 2.
        constraint.y_min = -2.
        constraint.y_max = 2.
        constraint.expr = ca.vcat([v, omega])

        self.model = model
        self.constraint = constraint