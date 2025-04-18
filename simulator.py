
import matplotlib.pyplot as plt
from scipy import integrate
import numpy as np
import control as ct

import model
import json
from utilities import import_data
import math


#########################################################
# this function returns the change in state of the robot
# based on the full differential equations
#########################################################
def equations_of_motion(x,t,m,M,L,g,d,uf):
    u = uf(x) # evaluate anonymous function at x
    Sx = np.sin(x[2])
    Cx = np.cos(x[2])
    D = m*L*L*(M+m*(1-Cx**2))
    
    dx = np.zeros(4)
    dx[0] = x[1]
    dx[1] = (1/D)*(-(m**2)*(L**2)*g*Cx*Sx + m*(L**2)*(m*L*(x[3]**2)*Sx - d*x[1])) + m*L*L*(1/D)*u
    dx[2] = x[3]
    dx[3] = (1/D)*((m+M)*m*g*L*Sx - m*L*Cx*(m*L*(x[3]**2)*Sx - d*x[1])) - m*L*Cx*(1/D)*u;
    
    return dx

#########################################################
# These are functions that simulate our system. They may
# use the actual equations of motion or different linear
# systems that approximate the system. These functions all
# have the same parameters so their results can be easily 
# compared.
# 
#########################################################


# use the odeint function and the true equations of motion 
# to simulate the system
def ode_sim(tspan, x0, xr):
    u = lambda x: -(model.K)@(x-xr) 
    return integrate.odeint(equations_of_motion,x0,tspan,args=(model.m,model.M,model.L,model.g,model.d,u))

# use true equations of motion to iteratively move the state forward
def it_ode_sim(tspan, x0, xr):
    run_data = np.empty([len(tspan),4])
    x = x0
    y = np.array([0.0, 0.0])
    dt = tspan[1]-tspan[0]
    u = lambda x: -model.K@(x - xr)
    for i in range(len(tspan)):
        dx = equations_of_motion(x,0.0,model.m,model.M,model.L,model.g,model.d,u)
        x = x + dx*dt
        run_data[i] = x

    return run_data

# iterative linear system simulation
# Ax + Bu. 
def it_ls_sim(tspan, x0, xr):
    
    run_data = np.empty([len(tspan),4])
    run_data[0] = x0
    x = x0
    u = 0.0
    dt = tspan[1]-tspan[0]

    for i in range(len(tspan)):
        dx = (model.A@(x-xr) + (model.B*u).transpose())[0]
        x = x + dx*dt
        u = -model.K@(x - xr)
        run_data[i] = x

    return run_data

# iterative linear system simulation with added noise
# Ax + Bu. 
def it_ls_sim_with_noise(tspan, x0, xr):
    
    run_data = np.empty([len(tspan),4])
    run_data[0] = x0
    x = x0
    y = np.array([1.0, 0.0])
    u = 0.0
    dt = tspan[1]-tspan[0]

    for i in range(len(tspan)):
        dx = (model.A@(x-xr) + (model.B*u).transpose())[0]
        x = x + dx*dt
        u = -model.K@(x - xr)
        x[0] += np.random.normal(0.0, 0.0001)
        x[3] += np.random.normal(0.0,0.0026)
        
        run_data[i] = x

    return run_data

# linear kalman filter simulation
def kf_sim(tspan, x0, xr):
    
    run_data = np.empty([len(tspan),4])
    run_data[0] = x0
    x = x0
    u = 0.0
    uy = np.array([0,1,0]).reshape((3,1))
    dt = tspan[1]-tspan[0]

    for i in range(len(tspan)):
        dx = (model.A_kf@(x-xr) + (model.B_kf@uy).transpose())[0]
        x = x + dx*dt
        u = -model.K@(x - xr)
        uy = np.array([u, x[0], x[3]]).reshape((3,1))
        run_data[i] = x

    return run_data

def kf_sim_with_noise(tspan, x0, xr):
    
    run_data = np.empty([len(tspan),4])
    run_data[0] = x0
    x = x0
    u = 0.0
    uy = np.array([0,1,0]).reshape((3,1))
    dt = tspan[1]-tspan[0]

    for i in range(len(tspan)):
        dx = (model.A_kf@(x-xr) + (model.B_kf@uy).transpose())[0]
        x = x + dx*dt
        u = -model.K@(x - xr)
        uy = np.array([u, x[0], x[3] + np.random.normal(0.0,0.00026)]).reshape((3,1))
        run_data[i] = x

    return run_data

#########################################################
# This is a simple wrapper class for simulation functions
# 
#########################################################
class Method:
    def __init__(self, simulation_function):
        self.method = simulation_function

    def simulate(self, time_series, x0, xr):
        return self.method(time_series, x0, xr)
        
#########################################################
# This function takes two simulation methods, runs them
# and plots comparative graphs for each state component.
#########################################################

def compare_solution_methods(method1, method2, time_series, x0, xr,):

    x_1 = method1.simulate(time_series, x0, xr)
    x_2 = method2.simulate(time_series, x0, xr)

    plt.rcParams['figure.figsize'] = [8, 8]
    plt.rcParams.update({'font.size': 18})

    state_labels = ('x ','v ','a ','av ')
    for i in range(4):
        plt.plot(time_series,x_1[:,i],linewidth=2,label=state_labels[i]+'1')
        plt.plot(time_series,x_2[:,i],linewidth=2,label=state_labels[i]+'2')
        plt.xlabel('Time')
        plt.ylabel('State')
        plt.legend()
        plt.show()





tspan = np.arange(0,10,0.01)
x0 = np.array([1,0,np.pi,0]) # Initial condition
xr = np.array([0,0,np.pi,0])      # Reference position 

method1 = Method(ode_sim)
method2 = Method(kf_sim_with_noise)

compare_solution_methods(method1, method2, tspan, x0, xr)


