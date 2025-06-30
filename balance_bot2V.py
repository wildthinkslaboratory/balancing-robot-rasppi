import casadi as ca
from casadi import sin, cos

import sys
sys.path.append('..')

from model import LQRModel, LQGModel, LQRDModel, LQGDModel
from simulator import Simulator, NoisySimulator, KalmanFilterTuner
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


constants = ca.vertcat( M, m, L, g, d )

# we build the nonlinear function f for the equations of motion
# we begin with some partial expressions to make the formulas easier to build


denominator = M + m*(sin(theta)**2)
n0 = -m*g*sin(theta)*cos(theta)
n1 = m*L*(sin(theta))*(thetadot)**2 + u
n2 = (m + M)*g*sin(theta)
RHS = ca.vertcat( 
    thetadot, 
    (n2+(-cos(theta))*(n1)) / (L*denominator)
    )


mc = ExperimentalConstants()
constant_values = [mc.M, mc.m, mc.L, mc.g, mc.d]

# I made latex names for my states. They look nice in the simulation plots
my_state_names = ['$\\theta$ ','$\\dot{\\theta}$ ']


# set up the K matrix
goal_state = [np.pi, 0.0]
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

print(mc.R_kf2V)
print(mc.Q_kf2V)

lqrdBot.set_up_K(mc.Q2V, mc.R2V, goal_state, goal_u)


# print(lqgdBot)
# lqgdBot = LQGDModel(state, 
#                 RHS, 
#                 u, 
#                 constants, 
#                 constant_values, 
#                 mc.dt,
#                 state_names=my_state_names,
#                 name='balancing robot LQGD')

# print(mc.R_kf2V)
# print(mc.Q_kf2V)

# lqgdBot.set_up_K(mc.Q2V, mc.R2V, goal_state, goal_u)
# lqgdBot.set_up_kalman_filter(C, mc.Q_kf2V, mc.R_kf2V)

# print(lqgdBot)

if __name__ == "__main__":
    # now we can rum a simulation
    u0 = np.array([0.0])
    x0 = np.array([np.pi - 0.1, 0.0]) # Initial condition
    sim_length = 4 # in seconds

    simulator = Simulator(lqrdBot, x0, u0, sim_length)
    simulator.run()

    # variances = np.array([0.000015,0.0000025])
    # simulator = NoisySimulator(lqgdBot, x0, u0, sim_length, noise=variances)
    # simulator.run()

    # variances = np.array([0.000015,0.0000025])
    # simulator = NoisySimulator(lqgdBot, x0, u0, sim_length, noise=variances)
    # simulator.run()

    # run_data = import_data('data.json')
    # num_cols = len(run_data[0])
    # sensor_data = []
    # for col in range(num_cols):
    #     A = [d[col] for d in run_data]
    #     sensor_data.append(A)

    # sensor_data = sensor_data[3:5]

    # filter_tuner = KalmanFilterTuner(lqgdBot, sensor_data)
    # filter_tuner.run()