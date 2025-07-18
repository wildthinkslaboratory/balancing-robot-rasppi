import casadi as ca
from casadi import sin, cos

import sys
sys.path.append('..')

from control_lib.model import LQRDModel, LQRModel, LQGModel, LQGDModel
from control_lib.simulator import Simulator, NoisySimulator, KalmanFilterTuner
import numpy as np
from builds import ExperimentalConstants
from utilities import import_data


# We create Casadi symbolic variables for the state
theta = ca.MX.sym('theta')
thetadot = ca.MX.sym('thetadot')
u = ca.MX.sym('u')             # control input
state = ca.vertcat(
    theta,
    thetadot,
)

# create casadi constants
M = ca.MX.sym('M') 
m = ca.MX.sym('m') 
L = ca.MX.sym('L') 
g = ca.MX.sym('g') 
d = ca.MX.sym('d')
ST = ca.MX.sym('ST')
r = ca.MX.sym('r')

constants = ca.vertcat( M, m, L, g, d, ST, r )

# we build the nonlinear function f for the equations of motion
# we begin with some partial expressions to make the formulas easier to build

# # old values when it balanced
# horz_acc = u / 0.4

horz_acc = u

# horz_acc = u * ST / r

# model for the motor torque
# 821 * u + 4.39 converts PWM to grams measured on a scale with lever arm 9 cm
torque = (821 * u + 4.39) * 9.81 * 0.09 / 1000
horz_acc = 2 * torque / 0.0325


denominator = M + m*(sin(theta)**2)
n0 = -m*g*sin(theta)*cos(theta)
n1 = m*L*(sin(theta))*(thetadot)**2 + horz_acc
n2 = (m + M)*g*sin(theta)
RHS = ca.vertcat( 
    thetadot, 
    (n2+(-cos(theta))*(n1)) / (L*denominator)
    )


mc = ExperimentalConstants()
constant_values = [mc.M, mc.m, mc.L, mc.g, mc.d, mc.ST, mc.r]

# I made latex names for my states. They look nice in the simulation plots
my_state_names = ['$\\theta$ ','$\\dot{\\theta}$ ']


# set up the K matrix
goal_state = [mc.balance_point, 0.0]
goal_u = [0.0]

# If we want a Kalman Filter we need to pass in a measurement model
C = np.array([[1, 0], \
            [0, 1]]) 



lqrdBot = LQRDModel(state, 
                RHS, 
                u, 
                constants, 
                constant_values, 
                mc.dt,
                state_names=my_state_names,
                name='balancing robot LQRD')



lqrdBot.set_up_K(mc.Q2V, mc.R2V, goal_state, goal_u)

print(lqrdBot)

lqrBot = LQRModel(state, 
                RHS, 
                u, 
                constants, 
                constant_values, 
                mc.dt,
                state_names=my_state_names,
                name='balancing robot LQR')



lqrBot.set_up_K(mc.Q2V, mc.R2V, goal_state, goal_u)

lqgBot = LQGModel(state, 
                RHS, 
                u, 
                constants, 
                constant_values, 
                mc.dt,
                state_names=my_state_names,
                name='balancing robot LQG')



lqgBot.set_up_K(mc.Q2V, mc.R2V, goal_state, goal_u)
lqgBot.set_up_kalman_filter(C, mc.Q_kf2V, mc.R_kf2V)

print(lqrdBot)

print(lqrBot)

lqgdBot = LQGModel(state, 
                RHS, 
                u, 
                constants, 
                constant_values, 
                mc.dt,
                state_names=my_state_names,
                name='balancing robot LQGD')



lqgdBot.set_up_K(mc.Q2V, mc.R2V, goal_state, goal_u)
lqgdBot.set_up_kalman_filter(C, mc.Q_kf2V, mc.R_kf2V)

if __name__ == "__main__":
    # now we can rum a simulation
    u0 = np.array([0.0])
    x0 = np.array([np.pi - (2 * np.pi / 180), 0.0]) # Initial condition
    sim_length = 0.5 # in seconds

    # simulator = Simulator(lqrdBot, x0, u0, sim_length)
    # simulator.run()

    simulator = Simulator(lqgBot, x0, u0, sim_length)
    simulator.run()



    variances = np.array([0.000015,0.0000025])

    simulator = NoisySimulator(lqgBot, x0, u0, sim_length, noise=variances, nudge=0.0)
    simulator.run()

    simulator = NoisySimulator(lqgdBot, x0, u0, sim_length, noise=variances, nudge=0.0)
    simulator.run()