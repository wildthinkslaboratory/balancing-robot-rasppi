import casadi as ca
from casadi import sin, cos

import sys
sys.path.append('..')

from control_lib.model import LQGModel, LQGDModel
from control_lib.simulator import Simulator, NoisySimulator, KalmanFilterTuner
import numpy as np
from builds import ExperimentalConstants
from utilities import import_data


# We create Casadi symbolic variables for the state
x = ca.MX.sym('x')  
xdot = ca.MX.sym('xdot')
theta = ca.MX.sym('theta')
thetadot = ca.MX.sym('thetadot')
u = ca.MX.sym('u')             # control input
state = ca.vertcat(
    x,
    xdot,
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

test = ca.MX.sym('p', 3)
print(test.size1())

constants = ca.vertcat( M, m, L, g, d, ST, r )


# we build the nonlinear function f for the equations of motion
# we begin with some partial expressions to make the formulas easier to build

# model for the motor torque
# 821 * u + 4.39 converts PWM to grams measured on a scale with lever arm 9 cm
torque = (821 * u + 4.39) * 9.81 * 0.09 / 1000
horz_acc = 2 * torque / r
denominator = M + m*(sin(theta)**2)
n0 = -m*g*sin(theta)*cos(theta)
n1 = m*L*(sin(theta))*(thetadot)**2 - d*xdot + horz_acc
n2 = (m + M)*g*sin(theta)
RHS = ca.vertcat( 
    xdot, 
    (n0 + n1) / denominator, 
    thetadot, 
    (n2+(-cos(theta))*(n1)) / (L*denominator)
    )

# the constant values are imported from the build file
pmc = ExperimentalConstants()
constant_values = [pmc.M, pmc.m, pmc.L, pmc.g, pmc.d, pmc.ST, pmc.r]

# I made latex names for my states. They look nice in the simulation plots
my_state_names = ['$x$ ','$\\dot{x}$ ','$\\theta$ ','$\\dot{\\theta}$ ']

lqgBot = LQGModel(state, 
                RHS, 
                u, 
                constants, 
                constant_values, 
                pmc.dt,
                state_names=my_state_names,
                name='balancing robot LQG')

lqgdBot = LQGDModel(state, 
                RHS, 
                u, 
                constants, 
                constant_values, 
                pmc.dt,
                state_names=my_state_names,
                name='balancing robot LQGD')

# set up the K matrix
goal_state = [0.0, 0.0, np.pi, 0.0]
goal_u = [0.0]

# If we want a Kalman Filter we need to pass in a measurement model
C = np.array([[1, 0, 0, 0], \
            [0, 0, 1, 0], \
            [0, 0, 0, 1]]) 


lqgBot.set_up_K(pmc.Q, pmc.R, goal_state, goal_u)
lqgdBot.set_up_K(pmc.Q, pmc.R, goal_state, goal_u)

lqgBot.set_up_kalman_filter(C, pmc.Q_kf, pmc.R_kf)
lqgdBot.set_up_kalman_filter(C, pmc.Q_kf, pmc.R_kf)

print(lqgBot)
print(lqgdBot)

if __name__ == "__main__":
    # now we can rum a simulation
    u0 = np.array([0.0])
    x0 = np.array([1.0,0,np.pi, 0.0]) # Initial condition
    sim_length = 5 # in seconds

    # simulator = Simulator(lqgBot, x0, u0, sim_length)
    # simulator.run()

    # simulator = Simulator(lqgdBot, x0, u0, sim_length)
    # simulator.run()

    variances = np.array([0.004,0.00015,0.0000025])
    # variances = np.array([0.004,0.0015,0.025])
    # simulator = NoisySimulator(lqgBot, x0, u0, sim_length, noise=variances, nudge=0.0)
    # simulator.run()

    simulator = NoisySimulator(lqgdBot, x0, u0, sim_length, noise=variances, nudge=0.0)
    simulator.run()

    run_data = import_data('data.json')
    num_cols = len(run_data[0])
    sensor_i = [5,6,7]
    # run_data = run_data[:300]
    sensor_data = np.empty((len(run_data),len(sensor_i)))
    for j, row in enumerate(run_data):
        record = [row[i] for i in sensor_i]
        sensor_data[j] = record

    filter_tuner = KalmanFilterTuner(lqgdBot, sensor_data)
    filter_tuner.run()
