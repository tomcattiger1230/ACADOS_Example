#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-03-15 22:43:48
LastEditors: Wei Luo
LastEditTime: 2021-03-16 00:18:51
Note: Note
'''

import numpy as np
import casadi as ca
from acados_template import AcadosModel

class MobileRobotModel(object):
    def __init__(self,):
        # control inputs
        v = ca.SX.sym('v')
        omega = ca.SX.sym('omega')
        controls = ca.vcat([v, omega])
        n_controls = controls.size()[0]
        # model states
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vcat([x, y, theta])

        rhs = [v*ca.cos(theta), v*ca.sin(theta), omega]

        # function
        f = ca.Function('f', [states, controls], [ca.vcat(rhs)], ['state', 'control_input'], ['rhs'])

        # acados model
        x_dot = ca.SX.sym('x_dot', len(rhs))
        f_impl = x_dot - f(states, controls)
        model = AcadosModel()
        model.f_expl_expr = f(states, controls)
        model.f_impl_expr = f_impl
        model.x = states
        model.xdot = x_dot
        model.u = controls
        model.p = []
        model.name = 'mobile_robot'

        self.model = model