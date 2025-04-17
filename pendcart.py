
import matplotlib.pyplot as plt
from scipy import integrate
import numpy as np
import control as ct

import model
import json
from utilities import import_data
import math


## ODE RHS Function Definition
def pendcart(x,t,m,M,L,g,d,uf):
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


def ode_simulation(tspan, x0, xr):
    u = lambda x: -(model.K)@(x-xr) 
    return integrate.odeint(pendcart,x0,tspan,args=(model.m,model.M,model.L,model.g,model.d,u))

def stepwise_ode_simulation(tspan, x0, xr):
    run_data = np.empty([len(tspan),4])
    x = x0
    y = np.array([0.0, 0.0])
    dt = tspan[1]-tspan[0]
    u = lambda x: -model.K@(x - xr)
    for i in range(len(tspan)):
        dx = pendcart(x,0.0,model.m,model.M,model.L,model.g,model.d,u)
        x = x + dx*dt
        run_data[i] = x

    return run_data


def linear_sys_simulation(tspan, x0, xr):
    run_data = np.zeros([len(tspan),4])
    x = x0
    u = 0.0
    dt = tspan[1]-tspan[0]
    sys = ct.ss(model.A, model.B, model.C, 0.0, dt)
    print(sys)
    # for i in range(len(tspan)):
    #     x = sys.dynamics(0.0,x0,u)
    #     u = -model.K@(x - xr)
    #     print(i, x, u)
    #     run_data[i] = x

    return run_data


def kalman_filter_simulation(tspan, x0, xr):
    
    run_data = np.empty([len(tspan),4])
    x = x0
    y = np.array([0.0, 0.0])
    u = 0.0
    dt = tspan[1]-tspan[0]

    for i in range(len(tspan)):
        dx = (model.A@(x-xr) + (model.B*u).transpose() + model.Kf@(y - model.C@(x)))[0]
        x = x + dx*dt
        u = -model.K@(x - xr)
        y = model.C@x + np.array([np.random.normal(0.0, 0.0001), np.random.normal(0.0,0.0000026)])
        run_data[i] = x

    return run_data


class Method:
    def __init__(self, simulation_function):
        self.method = simulation_function

    def simulate(self, time_series, x0, xr):
        return self.method(time_series, x0, xr)
        

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

method1 = Method(ode_simulation)
method2 = Method(stepwise_ode_simulation)

compare_solution_methods(method1, method2, tspan, x0, xr)


