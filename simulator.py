
import matplotlib.pyplot as plt
from scipy import integrate
import numpy as np
import control as ct
from time import perf_counter
import model as md
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
    u = lambda x: -(md.K)@(x-xr) 
    return integrate.odeint(equations_of_motion,x0,tspan,args=(md.m,md.M,md.L,md.g,md.d,u))


# use true equations of motion to iteratively move the state forward
def it_ode_sim(tspan, x0, xr):
    run_data = np.empty([len(tspan),4])
    x = x0
    y = np.array([0.0, 0.0])
    dt = tspan[1]-tspan[0]
    u = lambda x: -md.K@(x - xr)

    start_time = perf_counter()
    for i in range(len(tspan)):
        dx = equations_of_motion(x,0.0,md.m,md.M,md.L,md.g,md.d,u)
        x = x + dx*dt
        run_data[i] = x

    print('Time for 1000 iterations of it_ode_sim: ', perf_counter() - start_time)
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
        dx = (md.A@(x-xr) + (md.B*u).transpose())[0]
        x = x + dx*dt
        u = -md.K@(x - xr)
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
        dx = (md.A@(x-xr) + (md.B*u).transpose())[0]
        x = x + dx*dt
        u = -md.K@(x - xr)
        x[0] += np.random.normal(0.0, 0.0001)
        x[3] += np.random.normal(0.0,math.sqrt(md.angle_vel_var))
        
        run_data[i] = x

    return run_data

# linear kalman filter simulation
def kf_sim(tspan, x0, xr):
    
    run_data = np.empty([len(tspan),4])
    run_data[0] = x0
    x = x0
    u = 0.0
    uy = np.array([0, x0[0], x0[2], x0[3]]).reshape((4,1))
    uy_r = np.array([0, xr[0], xr[2], xr[3]]).reshape((4,1))
    dt = tspan[1]-tspan[0]

    for i in range(len(tspan)):
        dx = (md.A_kf@(x-xr) + (md.B_kf@(uy-uy_r)).transpose())[0]
        x = x + dx*dt
        u = -md.K@(x - xr)
        uy = np.array([u, x[0], x[2], x[3]]).reshape((4,1))
        run_data[i] = x

    return run_data

def kf_sim_with_noise(tspan, x0, xr):
    
    run_data = np.empty([len(tspan),4])
    run_data[0] = x0
    x = x0
    u = 0.0
    uy = np.array([0, x0[0], x0[2], x0[3]]).reshape((4,1))
    uy_r = np.array([0, xr[0], xr[2], xr[3]]).reshape((4,1))
    dt = tspan[1]-tspan[0]

    start_time = perf_counter()
    for i in range(len(tspan)):
        dx = (md.A_kf@(x-xr) + (md.B_kf@(uy-uy_r)).transpose())[0]
        x = x + dx*dt
        u = -md.K@(x - xr)
        position = x[0]
        angle = x[2] + np.random.normal(0.0,math.sqrt(md.angle_var))
        angle_vel = x[3] + np.random.normal(0.0,math.sqrt(md.angle_vel_var))
        uy = np.array([u, position, angle, angle_vel]).reshape((4,1))
        run_data[i] = x


    print('Time for 1000 iterations of kf_sim_with_noise: ', perf_counter() - start_time)
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



def run_comparison():
    tspan = np.arange(0,10,0.01)
    x0 = np.array([1,0,np.pi+0.1,0]) # Initial condition
    xr = np.array([0,0,np.pi,0])      # Reference position 

    method1 = Method(it_ode_sim)
    method2 = Method(kf_sim_with_noise)

    compare_solution_methods(method1, method2, tspan, x0, xr)


#########################################################
# Here we compare three simulations
#     - standard linear system with no noise (the true state)
#     - standard linear system noise
#     - Kalman Filter system with the same noisy values
#########################################################
def kf_comparison_plot():
    tspan = np.arange(0,10,0.01)
    x0 = np.array([1,0,np.pi,0]) # Initial condition
    xr = np.array([0,0,np.pi,0])      # Reference position 
    dt = tspan[1]-tspan[0]

    run_data_noise = np.empty([len(tspan),4])
    run_data_kf = np.empty([len(tspan),4])
    run_data_true = np.empty([len(tspan),4])

    run_data_noise[0] = x0
    run_data_kf[0] = x0
    run_data_true[0] = x0
    
    x_noise = x0
    x_kf = x0
    x_true = x0

    u_kf = 0.0
    uy = np.array([0, x0[0], x0[2], x0[3]])
    uy_r = np.array([0, xr[0], xr[2], xr[3]]).reshape((4,1))
    u_noise = 0.0
    u_true = 0.0

    
    for i in range(len(tspan)):
        av_noise = np.random.normal(0.0,math.sqrt(md.angle_vel_var))
        a_noise = np.random.normal(0.0,0.001)

        dx_true = (md.A@(x_true-xr) + (md.B*u_true).transpose())[0]
        x_true = x_true + dx_true*dt
        u_true = -md.K@(x_true - xr)
        run_data_true[i] = x_true

        x_noise[2] += a_noise
        x_noise[3] += av_noise  
        dx_noise = (md.A@(x_noise-xr) + (md.B*u_noise).transpose())[0]
        x_noise = x_noise + dx_noise*dt
        u_noise = -md.K@(x_noise - xr)
        run_data_noise[i] = x_noise

        dx_kf = (md.A_kf@(x_kf-xr) + (md.B_kf@(uy.reshape((4,1))-uy_r)).transpose())[0]
        x_kf = x_kf + dx_kf*dt
        u_kf = -md.K@(x_kf - xr)

        uy[0] = u_kf
        uy[1] = x_kf[0]
        uy[2] = x_kf[2] + a_noise
        uy[3] = x_kf[3] + av_noise
        run_data_kf[i] = x_kf
       

    plt.rcParams['figure.figsize'] = [8, 8]
    plt.rcParams.update({'font.size': 18})
    
    i = 3
    plt.plot(tspan,run_data_noise[:,i],linewidth=2,label='av measured')
    plt.plot(tspan,run_data_kf[:,i],linewidth=2,label='av k filter')
    plt.plot(tspan,run_data_true[:,i],linewidth=2,label='av true')
    plt.xlabel('Time')
    plt.ylabel('State')
    plt.legend()
    plt.show()

    i=2
    plt.plot(tspan,run_data_noise[:,i],linewidth=2,label='a measured')
    plt.plot(tspan,run_data_kf[:,i],linewidth=2,label='a k filter')
    plt.plot(tspan,run_data_true[:,i],linewidth=2,label='a true')
    plt.xlabel('Time')
    plt.ylabel('State')
    plt.legend()
    plt.show()

#run_comparison()
kf_comparison_plot()