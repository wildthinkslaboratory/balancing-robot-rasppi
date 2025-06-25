# This is a two variable version of the inverted pendulum problem 

import casadi as ca
from casadi import sin, cos
import math

import sys
sys.path.append('..')

from control_lib.model import LQRModel, LQGModel, LQGDiscreteModel2, LQGDiscreteModel
from control_lib.simulator import Simulator, Comparison
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

# Now we make our model.
balanceBot = LQRModel(state, 
                              RHS, 
                              u, 
                              constants, 
                              constant_values, 
                              pmc.dt,
                              name='2 var balancing robot LQR', 
                              state_names=state_names)

# set the goal state
balanceBot.set_goal_state([np.pi + (1*np.pi/180), 0.0])


balanceBot.set_Q(pmc.Q2V)
balanceBot.set_R(pmc.R2V)

# now we can do the set up that will build all our matrices
balanceBot.set_up()

# If we want a Kalman Filter we need to pass in a measurement model
C = np.array([[1, 0], 
              [0, 1]]) 


lqgBot = LQGModel(balanceBot, C, pmc.Q_kf2V, pmc.R_kf2V, name='2 var balancing robot LQG')

lqgdBot = LQGDiscreteModel(lqgBot, name='2 var balancing robot discrete LQG')

lqgdBot2 = LQGDiscreteModel2(lqgBot, name='2 var balancing robot discrete LQG model 2')



if __name__ == "__main__":
    # now we can rum a simulation
    u0 = np.array([0.0])
    x0 = np.array([np.pi + 0.2, 0.0]) # Initial condition
    sim_length = 1 # in seconds

    # simulator = Simulator(lqgdBot, x0, u0, sim_length)
    # simulator.run()

    # simulator = Simulator(lqgdBot2, x0, u0, sim_length)
    # simulator.run()

    simulator = Comparison(lqgdBot, lqgdBot2, x0, u0, sim_length)
    simulator.run()

    # tspan = np.arange(0,sim_length,lqgdBot.dt)

    # run_data_noise = np.empty([len(tspan),2])
    # run_data_kf = np.empty([len(tspan),2])
    # run_data_true = np.empty([len(tspan),2])
    
    # # x_noise = x0
    # # x_kf = x0
    # # x_true = x0
    # # y = x0
    # # y_true = x0
    # # y_noise = x0

    # # u = np.array([0.0])
    # # u_noise = u
    # # u_true = u

    # # dummy = 0

    # x = x0
    # u = u0
    # y = x0 # full state
    # x2 = x0
    # u2 = u0
    # y2 = x0 # full state
          
    # for i in range(len(tspan)):
    #     # av_noise = np.random.normal(0.0,0.025)
    #     # a_noise = np.random.normal(0.0,0.25)

    #     # if i == math.floor((sim_length / balanceBot.dt) / 2):
    #     #     # nudge the system
    #     #     x_true = balanceBot.get_next_state_simulator(x_true, np.array([1.0]), dummy)
    #     #     x_noise = balanceBot.get_next_state_simulator(x_noise, np.array([1.0]), dummy)
    #     #     x_kf = lqgdBot.get_next_state(x_kf, np.array([1.0]), y)



    #     x = lqgdBot.get_next_state(x,u,y)
    #     u = lqgdBot.get_control_input(x)
    #     y = x # assume perfect sensors
    #     run_data_true[i] =  x

    #     print('b', x2, u2, y2)
    #     x2 = lqgdBot2.get_next_state(x2,u2,y2)
    #     u2 = lqgdBot2.get_control_input(x2)
    #     y2 = x2 # assume perfect sensors
    #     run_data_kf[i] =  x2



        
        # x_true = lqgdBot.get_next_state(x_true, u_true, y_true) # sensor data same as state
        # u_true = lqgdBot.get_control_input(x_true)
        # y_true = x_true
        # run_data_true[i] = x_true

 
        # x_noise[0] = x_true[0] + a_noise
        # x_noise[1] = x_true[1] + av_noise

        # x_noise = balanceBot.get_next_state_simulator(x_noise, u_true, dummy)
        # run_data_noise[i] = np.reshape(x_noise, (2,))

        # y[0] = x_kf[0] + a_noise
        # y[1] = x_kf[1] + av_noise
        # x_kf = lqgdBot2.get_next_state(x_kf, u, y)
        # u = lqgdBot2.get_control_input(x_kf)
        # y = x_kf
        # run_data_kf[i] = x_kf



    # plt.rcParams['figure.figsize'] = [10, 7]
    # # plt.rcParams.update({'font.size': 18})
    # plt.rcParams.update({
    # "text.usetex": True
    # })
    


    # i=0
    # #plt.plot(tspan,run_data_noise[:,i],linewidth=1,label='$\\theta$ true + noise')
    # plt.plot(tspan,run_data_kf[:,i],linewidth=1,label='$\\theta$ KF')
    # plt.plot(tspan,run_data_true[:,i],linewidth=1,label='$\\theta$ true')
    # plt.title('Two Variable Solution')
    # plt.xlabel('Time')
    # plt.ylabel('State')
    # plt.legend()
    # plt.savefig("KFangle2.pdf", format="pdf", bbox_inches="tight")
    # plt.show()

    # i = 1
    # #plt.plot(tspan,run_data_noise[:,i],linewidth=1,label=('$\\dot{\\theta}$ true + noise'))
    # plt.plot(tspan,run_data_kf[:,i],linewidth=1,label='$\\dot{\\theta}$ KF')
    # plt.plot(tspan,run_data_true[:,i],linewidth=1,label='$\\dot{\\theta}$ true')
    # plt.title('Two Variable Solution')
    # plt.xlabel('Time')
    # plt.ylabel('State')
    # plt.legend()
    # plt.savefig("KFangular_velocity2.pdf", format="pdf", bbox_inches="tight")
    # plt.show()