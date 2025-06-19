



import casadi as ca
from casadi import sin, cos
from model import LinearModel
from simulator import Simulator, NoisySimulator
import numpy as np
import matplotlib.pyplot as plt
from builds import *

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
DC = ca.MX.sym('DC')
r = ca.MX.sym('r')


constants = ca.vertcat( M, m, L, g, d, DC, r )

# we build the nonlinear function f for the equations of motion
# we begin with some partial expressions to make the formulas easier to build
denominator = M + m*(sin(theta)**2)
F = (u / DC_) * 2 / r
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
constant_values = [M_, m_, L_, g_, d_, DC_, r_]

# I made latex names for my states. They look nice in the simulation plots
state_names = ['$x$ ','$\\dot{x}$ ','$\\theta$ ','$\\dot{\\theta}$ ']

# Now we make our model.
balanceBot = LinearModel(state, 
                              RHS, 
                              u, 
                              constants, 
                              constant_values, 
                              name='balancing robot LQG', 
                              state_names=state_names)

# set the goal state
balanceBot.set_goal_state([0.0, 0.0, np.pi, 0.0])

# we add our Q and R matrices
Q = np.array([[0.0001, 0, 0, 0],\
            [0, 0.001, 0, 0],\
            [0, 0, 1, 0],\
            [0, 0, 0, 1]])

R = np.array([[1]])
balanceBot.set_Q(Q)
balanceBot.set_R(R)

# now we can do the set up that will build all our matrices
balanceBot.set_up()

# If we want a Kalman Filter we need to pass in a measurement model
C = np.array([[1, 0, 0, 0], \
            [0, 0, 1, 0], \
            [0, 0, 0, 1]]) 

# disturbance and noise matrices
Q_kf = np.eye(4)
R_kf = np.eye(3)

# then we compute all the Kalman Filter matrices
balanceBot.set_up_Kalman_Filter(C, Q_kf, R_kf)
balanceBot.est_method = 'kf'

if __name__ == "__main__":
    # now we can rum a simulation
    u0 = np.array([0.0])
    x0 = np.array([1,0,np.pi + 0.2, 0.0]) # Initial condition
    dt = 0.01
    sim_length = 0.4 # in seconds
    simulator = Simulator(balanceBot, x0, u0, sim_length, dt)
    simulator.run()
        

    # we can make a noisy simulator
    noisy_sim = NoisySimulator(balanceBot, x0, u0, sim_length, dt)
    noisy_sim.run()

# Alternatively, one can analyze the system's eigenvalues and choose 
# a time step that is significantly smaller than the inverse of the 
# fastest eigenvalue's magnitude.
# For example, if the fastest eigenvalue has a magnitude of 10 rad/s, 
# the time step should be significantly smaller than 0.1 seconds. 