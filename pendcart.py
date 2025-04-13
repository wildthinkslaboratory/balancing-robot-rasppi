# note to Izzy
# I think this will eventually be a bunch of testing and debugging scripts.
# It probably needs a different filename. The function pendcart has the actual
# equations of motion for the bot, so we can compare our run data to an ODE
# simulation.

import matplotlib.pyplot as plt
from scipy import integrate
import numpy as np
from control.matlab import lqr
import model
import json

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
    plt.rcParams['animation.html'] = 'jshtml'

    # ## Simulate closed-loop system
    tspan = np.arange(0,10,0.01)
    x0 = np.array([1,0,np.pi,0]) # Initial condition
    wr = np.array([0,0,np.pi,0])      # Reference position
    u = lambda x: -(model.K)@(x-wr)           # Control law


    x = integrate.odeint(pendcart,x0,tspan,args=(model.m,model.M,model.L,model.g,model.d,u))

    plot_labels = ('x','v','theta','omega')
    [plt.plot(tspan,x[:,j],linewidth=2,label=plot_labels[j]) for j in range(4)]
    plt.xlabel('Time')
    plt.ylabel('State')

    plt.legend()
    plt.show()

# DEBUG function
# take the data generated by a live lqe run and
# use it's control values u in an ode simulation
# then we compare the states produce by the ode
# with those produced by the live run
def compare_state_data():

    # read in run data from file
    datafile = 'data.json'
    with open(datafile, 'r') as file:
        data = json.load(file)

    # convert the data to columns so we can plot it
    # this assumes 5 columns, x, v, a , av, and u
    input_data = [[],[],[],[],[]]
    for record in data:
        for i in range(5):
            input_data[i].append(record[i])
    
    dt = 0.01  # we need to know the time window for the generated data
    x = data[0][:4] # starting state

    sim_data = [[],[],[],[]]
    for record in data:
        uf = lambda x: record[4]
        print("record",record) 
        t = 0.0 # this is a dummy parameter, needed for ode call
        dx = pendcart(x,t,model.m,model.M,model.L,model.g,model.d,uf)
        x = x + dx*dt
        print("state", x)
        for i in range(4):
            sim_data[i].append(x[i])

    plot_labels = ('u','x_input','x_sim')
    plt.plot(input_data[4])
    plt.plot(input_data[0])
    plt.plot(sim_data[0])
    plt.xlabel('Time')
    plt.ylabel('state')

    plt.legend()
    plt.show()

compare_state_data()
    