# This is a two variable version of the inverted pendulum problem 

import casadi as ca
from casadi import sin, cos
from model import LQRModel, LQGModel, LQGDiscreteModel
from simulator import Simulator, NoisySimulator
import numpy as np
import matplotlib.pyplot as plt
from builds import ExperimentalConstants

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

# the constant values are imported from the build file
pmc = ExperimentalConstants()
constant_values = [pmc.M, pmc.m, pmc.L, pmc.g, pmc.d]

# I made latex names for my states. They look nice in the simulation plots
state_names = ['$\\theta$ ','$\\dot{\\theta}$ ']

dt = 0.005

# Now we make our model.
balanceBot = LQRModel(state, 
                              RHS, 
                              u, 
                              constants, 
                              constant_values, 
                              dt,
                              name='2 var balancing robot LQR', 
                              state_names=state_names)

# set the goal state
balanceBot.set_goal_state([np.pi, 0.0])


balanceBot.set_Q(pmc.Q2V)
balanceBot.set_R(pmc.R2V)

# now we can do the set up that will build all our matrices
balanceBot.set_up()

# If we want a Kalman Filter we need to pass in a measurement model
C = np.array([[1, 0], 
              [0, 1]]) 

# disturbance and noise matrices
Q_kf = np.eye(2)
R_kf = np.eye(2)

lqgBot = LQGModel(balanceBot, C, Q_kf, R_kf, name='2 var balancing robot LQG')

lqgdBot = LQGDiscreteModel(lqgBot, name='2 var balancing robot discrete LQG')

if __name__ == "__main__":
    # now we can rum a simulation
    u0 = np.array([0.0])
    x0 = np.array([np.pi + 0.1, 0.0]) # Initial condition
    sim_length = 0.4 # in seconds
    simulator = Simulator(balanceBot, x0, u0, sim_length)
    input_bounds = np.array([[-12, 12]])
    simulator.add_intput_bound(input_bounds)
    simulator.run()
        
    simulator = Simulator(lqgBot, x0, u0, sim_length)
    input_bounds = np.array([[-14,14]])
    simulator.add_intput_bound(input_bounds)
    simulator.run()

    simulator = Simulator(lqgdBot, x0, u0, sim_length)
    input_bounds = np.array([[-14,14]])
    simulator.add_intput_bound(input_bounds)
    simulator.run()

    # # we can make a noisy simulator
    # noisy_sim = NoisySimulator(balanceBot, x0, u0, sim_length, dt)
    # noisy_sim.run()
