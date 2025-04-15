
import matplotlib.pyplot as plt
from scipy import integrate
import numpy as np
from control.matlab import lqr
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


# this function just runs an LQR simulation
# and plots the resulting state curves
def run_simulation():

    plt.rcParams['figure.figsize'] = [8, 8]
    plt.rcParams.update({'font.size': 18})

    tspan = np.arange(0,10,0.01)
    x0 = np.array([1,0,np.pi,0]) # Initial condition
    wr = np.array([0,0,np.pi,0])      # Reference position 
    u = lambda x: -(model.K)@(x-wr)           # Control law

    x = integrate.odeint(pendcart,x0,tspan,args=(model.m,model.M,model.L,model.g,model.d,u))

    u_data = []
    for state in x:
        u_data.append(u(state))
    
    
    state = x0
    Kf_list = [[state[0]], [state[1]], [state[2]], [state[3]]]
    y = np.array([0.0, 0.0])
    t = 0.0
    uv = 0.0
    for i in range(999):
        dx = (model.A@(state-wr) + (model.B*uv).transpose() + model.Kf@(y - model.C@(state - wr)))[0]
        state = state + dx*(0.01)
        uv = -model.K@(state - wr)
        y = model.C@state + np.array([np.random.normal(0.0, 0.0001), np.random.normal(0.0,0.0000026)])
        for j in range(4):             # collect data
            Kf_list[j].append(state[j])

    plot_labels = ('x','v','theta','omega')
    [plt.plot(tspan,x[:,j],linewidth=2,label=plot_labels[j]) for j in range(4)]
    plot_labels = ('x Kf','v Kf','theta Kf','omega Kf')
    [plt.plot(tspan,Kf_list[j],linewidth=2,label=plot_labels[j]) for j in range(4)]
    # plt.plot(tspan,u_data, label='u')
    plt.xlabel('Time')
    plt.ylabel('State')

    
    plt.legend()
    plt.show()

run_simulation()