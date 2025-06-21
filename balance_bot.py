import casadi as ca
from casadi import sin, cos

import sys
sys.path.append('..')

from control_lib.model import LQRModel, LQGModel, LQRDiscreteModel, LQGDiscreteModel
from control_lib.simulator import Simulator, NoisySimulator
import numpy as np
import matplotlib.pyplot as plt
from builds import ExperimentalConstants


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


constants = ca.vertcat( M, m, L, g, d )

# we build the nonlinear function f for the equations of motion
# we begin with some partial expressions to make the formulas easier to build
denominator = M + m*(sin(theta)**2)
n0 = -m*g*sin(theta)*cos(theta)
n1 = m*L*(sin(theta))*(thetadot)**2 - d*xdot + u
n2 = (m + M)*g*sin(theta)
RHS = ca.vertcat( 
    xdot, 
    (n0 + n1) / denominator, 
    thetadot, 
    (n2+(-cos(theta))*(n1)) / (L*denominator)
    )

# the constant values are imported from the build file
pmc = ExperimentalConstants()
constant_values = [pmc.M, pmc.m, pmc.L, pmc.g, pmc.d]

# I made latex names for my states. They look nice in the simulation plots
state_names = ['$x$ ','$\\dot{x}$ ','$\\theta$ ','$\\dot{\\theta}$ ']


# Now we make our model.
lqrBot = LQRModel(state, 
                RHS, 
                u, 
                constants, 
                constant_values, 
                pmc.dt,
                name='balancing robot LQR', 
                state_names=state_names)

# set the goal state
lqrBot.set_goal_state([0.0, 0.0, np.pi, 0.0])


lqrBot.set_Q(pmc.Q)
lqrBot.set_R(pmc.R)

# now we can do the set up that will build all our matrices
lqrBot.set_up()


# If we want a Kalman Filter we need to pass in a measurement model
C = np.array([[1, 0, 0, 0], \
            [0, 0, 1, 0], \
            [0, 0, 0, 1]]) 

# disturbance and noise matrices
Q_kf = np.eye(4)
R_kf = np.eye(3)

lqgBot = LQGModel(lqrBot, C, Q_kf, R_kf, name='LQG Balance Bot')

# lqrdBot = LQRDiscreteModel(lqrBot)

lqgdBot = LQGDiscreteModel(lqgBot, name='Discrete LQG Balance Bot')


if __name__ == "__main__":
    # now we can rum a simulation
    u0 = np.array([0.0])
    x0 = np.array([1.0,0,np.pi + 0.3, 0.0]) # Initial condition
    sim_length = 4 # in seconds

    simulator = Simulator(lqrBot, x0, u0, sim_length)
    input_bounds = np.array([[-14,14]])
    simulator.add_intput_bound(input_bounds)
    simulator.run()
        
    simulator = Simulator(lqgBot, x0, u0, sim_length)
    input_bounds = np.array([[-14,14]])
    simulator.add_intput_bound(input_bounds)
    simulator.run()

    # simulator = Simulator(lqrdBot, x0, u0, sim_length)
    # simulator.run()

    simulator = Simulator(lqgdBot, x0, u0, sim_length)
    input_bounds = np.array([[-14,14]])
    simulator.add_intput_bound(input_bounds)
    simulator.run()


    # # we can make a noisy simulator
    # noisy_sim = NoisySimulator(lqrBot, x0, u0, sim_length, dt)
    # noisy_sim.run()
