import casadi as ca
from casadi import sin, cos

import sys
sys.path.append('..')

from control_lib.model import LQRModel, LQGModel, LQRDModel, LQGDModel
from control_lib.simulator import Simulator, NoisySimulator, KalmanFilterTuner
import numpy as np
from builds import ExperimentalConstants
from utilities import import_data

# create casadi constants
dt = 0.01
g_ = 9.81                # gravity m / s^2
R_ = 0.0325               # wheel radius m
M_ = 0.25435            # mass of body kg
m_ = 0.757536             # mass of wheels kg
L_ = 0.085                 # length from COM body and wheel axis m
I_b_ =  0.008553260629             # inertia of body kg m^2
I_w_ = 0.000062147      # inertia of wheel kg m^2
ST_ = 0.45               # stall torque



# create casadi constants
M = ca.MX.sym('M') 
m = ca.MX.sym('m') 
I_b = ca.MX.sym('I_b') 
I_w = ca.MX.sym('I_w') 
L = ca.MX.sym('L')
g = ca.MX.sym('g')
R = ca.MX.sym('R')
ST = ca.MX.sym('ST')

constants = ca.vertcat( M, m, I_b, I_w, L, g, R, ST)
constant_values = [M_, m_, I_b_, I_w_, L_, g_, R_, ST_]

# We create Casadi symbolic variables for the state
# Note that our angle theta is 0 when the robot is vertical
# and grows positive as the robot rotates clockwise in the forward
# x direction.
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

# I made latex names for my states. They look nice in the simulation plots
my_state_names = ['$x$ ','$\\dot{x}$ ','$\\theta$ ','$\\dot{\\theta}$ ']

# Now we build some shortcuts for building up the equations
s_1 = I_b + m * L**2 
s_2 = m * L
s_3 = I_w / R**2 + M + M

# T = u * ST
# model for the motor torque
# converts PWM to torque.  measured on a scale with lever arm 9 cm

T = (0.821 * u + 0.00439) * 9.81 * 0.09

t_1 = -T + g * s_2 * sin(theta)
t_2 = T / R + s_2 * thetadot**2 * sin(theta)

# Determinant of our matrix M(theta)
detM = s_1 * s_3 - s_2**2 * (cos(theta))**2

RHS = ca.vertcat( 
    xdot, 
    1 /detM * (s_3 * t_1 - s_2 * cos(theta) * t_2), 
    thetadot, 
    1 / detM * (-s_2 * cos(theta) * t_1 + s_1 * t_2)
    )


# set up the K matrix
goal_state = [0.0, 0.0, 0.0, 0.0]
goal_u = [0.0]

# If we want a Kalman Filter we need to pass in a measurement model
C = np.array([[1, 0, 0, 0], \
              [0, 0, 1, 0],
            [0, 0, 0, 1]]) 

Q = np.diag([1,1,1,1])
R = np.diag([5000])

Q_kf = np.diag([1,1,1,1]) / 10000
R_kf = np.diag([0.004,0.00015,0.0000025])

lqrBot = LQRModel(state, 
                RHS, 
                u, 
                constants, 
                constant_values, 
                dt,
                state_names=my_state_names,
                name='balancing robot LQR')

lqrBot.set_up_K(Q, R, goal_state, goal_u)

lqgBot = LQGModel(state, 
                RHS, 
                u, 
                constants, 
                constant_values, 
                dt,
                state_names=my_state_names,
                name='balancing robot LQG')

lqgBot.set_up_K(Q, R, goal_state, goal_u)
lqgBot.set_up_kalman_filter(C, Q_kf, R_kf)

lqgdBot = LQGDModel(state, 
                RHS, 
                u, 
                constants, 
                constant_values, 
                dt,
                state_names=my_state_names,
                name='balancing robot LQG')

lqgdBot.set_up_K(Q, R, goal_state, goal_u)
lqgdBot.set_up_kalman_filter(C, Q_kf, R_kf)
print(lqgBot)

if __name__ == "__main__":
    # now we can rum a simulation
    u0 = np.array([0.0])
    x0 = np.array([-1.0,0, 0.0, 0.0]) # Initial condition
    sim_length = 10 # in seconds

    # simulator = Simulator(lqrBot, x0, u0, sim_length)
    # simulator.run()

    simulator = Simulator(lqgdBot, x0, u0, sim_length)
    simulator.run()

    variances = np.array([0.004, 0.000015,0.0000025])
    simulator = NoisySimulator(lqgBot, x0, u0, sim_length, noise=variances, nudge=0.0)
    simulator.run()

    # run_data = import_data('data.json')
    # num_cols = len(run_data[0])
    # sensor_data = []
    # for col in range(num_cols):
    #     A = [d[col] for d in run_data]
    #     sensor_data.append(A)

    # sensor_data = sensor_data[3:5]

    # filter_tuner = KalmanFilterTuner(lqgdBot, sensor_data)
    # filter_tuner.run()
