
import matplotlib.pyplot as plt
from scipy import integrate
import numpy as np
from control.matlab import ss, c2d
from time import perf_counter
from pendulum_model import SSPendModel, equations_of_motion, SSPendModelTwoVar, equations_of_motion_two_var
from utilities import import_data, output_data
from math import sqrt


md = SSPendModel()
ma = SSPendModelTwoVar()
angle_vel_var = 0.0000026

#########################################################
# These are functions that simulate our system. They may
# use the actual equations of motion or different linear
# systems that approximate the system. These functions all
# have the same parameters so their results can be easily 
# compared.
# 
#########################################################

# use true equations of motion to iteratively move the state forward
def ode(tspan, x0, xr):
    run_data = np.empty([len(tspan),4])
    x = x0
    y = np.array([0.0, 0.0])
    dt = tspan[1]-tspan[0]
    start_time = perf_counter()
    for i in range(len(tspan)):
        u = -md.K@(x - xr)
        dx = equations_of_motion(x,0.0,u)
        x = x + dx*dt
        run_data[i] = x

    print('Time for', len(tspan), 'iterations of it_ode_sim: ', perf_counter() - start_time)
    return run_data


# linear state space system simulation
# Ax + Bu. 
def lss(tspan, x0, xr):
    
    run_data = np.empty([len(tspan),4])
    run_data[0] = x0
    x = x0
    u = np.array([0.0])
    dt = tspan[1]-tspan[0]

    start_time = perf_counter()
    for i in range(len(tspan)):
        dx = md.A@(x-xr) + md.B@u
        x = x + dx*dt
        u[0] = -md.K@(x - xr)
        run_data[i] = x

    print('Time for', len(tspan), 'iterations of it_ls_sim:', perf_counter() - start_time)
    return run_data




# linear state space system simulation with added noise
# Ax + Bu. 
def lss_with_noise(tspan, x0, xr):
    run_data = np.empty([len(tspan),4])
    run_data[0] = x0
    x = x0
    y = np.array([1.0, 0.0])
    u = np.array([0.0])
    dt = tspan[1]-tspan[0]

    start_time = perf_counter()
    for i in range(len(tspan)):
        dx = md.A@(x-xr) + md.B@u
        x = x + dx*dt
        u[0] = -md.K@(x - xr)
        x[0] += np.random.normal(0.0, 0.0001)
        x[3] += np.random.normal(0.0,sqrt(angle_vel_var))
        run_data[i] = x

    print('Time for', len(tspan), 'iterations of it_ls_sim_with_noise:', perf_counter() - start_time)
    return run_data


# linear state space system simulation with discrete
# state computation
# x_{n} = (A_d)x_{n-1} + (B_d)u_{n-1}
def lss_discrete(tspan, x0, xr):

    # construct discrete system
    dt = tspan[1]-tspan[0]
    sys_c = ss(md.A, md.B, np.eye(4), np.zeros_like(md.B))
    sys_d = c2d(sys_c, dt, 'zoh')

    run_data = np.empty([len(tspan),4])
    run_data[0] = x0
    x = x0
    u = 0.0
    uy = np.array([0, x0[0], x0[2], x0[3]])
    uy_r = np.array([0, xr[0], xr[2], xr[3]])

    start_time = perf_counter()
    for i in range(len(tspan)):
        x = sys_d.A@(x-xr) + sys_d.B@([u]) + xr
        u = -md.K@(x - xr)
        uy = np.array([u, x[0], x[2], x[3]])
        run_data[i] = x

    print('Time for', len(tspan), 'iterations of ls_discrete: ', perf_counter() - start_time)
    return run_data


# linear state space system simulation with Kalman filter
# (A-KfC)x + (B Kf)u. 
# 
# measured variables: position, angle and angular velocity
def kf_sim(tspan, x0, xr):
    
    run_data = np.empty([len(tspan),4])
    run_data[0] = x0
    x = x0
    u = 0.0
    uy = np.array([0, x0[0], x0[2], x0[3]])
    uy_r = np.array([0, xr[0], xr[2], xr[3]])
    dt = tspan[1]-tspan[0]

    start_time = perf_counter()
    for i in range(len(tspan)):
        dx = md.A_kf@(x-xr) + md.B_kf@(uy-uy_r)
        x = x + dx*dt
        u = -md.K@(x - xr)
        uy = np.array([u, x[0], x[2], x[3]])
        run_data[i] = x

    print('Time for', len(tspan), 'iterations of kf_sim: ', perf_counter() - start_time)
    return run_data


# linear state space system simulation with Kalman filter
# with added noise
# (A-KfC)x + (B Kf)u. 
# 
# measured variables: position, angle and angular velocity
def kf_sim_with_noise(tspan, x0, xr):
    
    run_data = np.empty([len(tspan),4])
    run_data[0] = x0
    x = x0
    u = 0.0
    uy = np.array([0, x0[0], x0[2], x0[3]])
    uy_r = np.array([0, xr[0], xr[2], xr[3]])
    dt = tspan[1]-tspan[0]

    start_time = perf_counter()
    for i in range(len(tspan)):
        dx = md.A_kf@(x-xr) + md.B_kf@(uy-uy_r)
        x = x + dx*dt
        u = -md.K@(x - xr)
        position = x[0]
        angle = x[2] + np.random.normal(0.0,sqrt(angle_vel_var))
        angle_vel = x[3] + np.random.normal(0.0,sqrt(angle_vel_var))
        uy = np.array([u, position, angle, angle_vel])
        run_data[i] = x


    print('Time for', len(tspan), 'iterations of kf_sim_with_noise: ', perf_counter() - start_time)
    return run_data


# linear state space system simulation with discrete
# state computation
# x_{n} = ((A-KfC)_d)x_{n-1} + (((B Kf))_d)u_{n-1}
# 
# measured variables: position, angle and angular velocity
def kf_discrete(tspan, x0, xr):
    dt = tspan[1]-tspan[0]
    sys_c = ss(md.A_kf, md.B_kf, md.C_kf, md.D_kf)
    sys_d = c2d(sys_c, dt, 'zoh')

    run_data = np.empty([len(tspan),4])
    run_data[0] = x0
    x = x0
    u = 0.0
    uy = np.array([0, x0[0], x0[2], x0[3]])
    uy_r = np.array([0, xr[0], xr[2], xr[3]])

    start_time = perf_counter()
    for i in range(len(tspan)):
        x = sys_d.A@(x-xr) + sys_d.B@((uy-uy_r)) + xr
        u = -md.K@(x - xr)
        uy = np.array([u, x[0], x[2], x[3]])
        run_data[i] = x

    print('Time for', len(tspan), 'iterations of kf_discrete: ', perf_counter() - start_time)
    return run_data

# ode simulation for two 
# variables: angle, angular velocity
# 
def ode_angle_only(tspan, x0, xr):
    
    run_data = np.empty([len(tspan),4])
    run_data[0] = x0
    x = np.array([x0[2], x0[3]])
    xra = np.array([xr[2], xr[3]])
    u = np.array([0.0])
    dt = tspan[1]-tspan[0]

    start_time = perf_counter()
    for i in range(len(tspan)):
        dx = equations_of_motion_two_var(x,0.0,u[0])
        x = x + dx*dt
        u[0] = -ma.K@(x - xra)
        run_data[i] = np.array([0.0, 0.0, x[0], x[1]])

    print('Time for', len(tspan), 'iterations of it_ls_sim:', perf_counter() - start_time)
    return run_data

# linear state space system simulation for two 
# variables: angle, angular velocity
# Ax + Bu. 
def lss_angle_only(tspan, x0, xr):
    
    run_data = np.empty([len(tspan),4])
    run_data[0] = x0
    x = np.array([x0[2], x0[3]])
    xra = np.array([xr[2], xr[3]])
    u = np.array([0.0])
    dt = tspan[1]-tspan[0]

    start_time = perf_counter()
    for i in range(len(tspan)):
        dx = ma.A@(x-xra) + ma.B@u
        x = x + dx*dt
        u[0] = -ma.K@(x - xra)
        run_data[i] = np.array([0.0, 0.0, x[0], x[1]])

    print('Time for', len(tspan), 'iterations of it_ls_sim:', perf_counter() - start_time)
    return run_data


# linear state space system simulation for two 
# variables: angle, angular velocity
# with a Kalman filter
# Ax + Bu. 
def kf_angle_only(tspan, x0, xr):
    
    run_data = np.empty([len(tspan),4])
    run_data[0] = x0
    x = np.array([x0[2], x0[3]])
    xra = np.array([xr[2], xr[3]])
    uy = np.array([0.0, x0[2], x0[3]])
    ur = np.array([0.0, xr[2], xr[3]])
    dt = tspan[1]-tspan[0]

    start_time = perf_counter()
    for i in range(len(tspan)):
        dx = ma.A_kf@(x-xra) + ma.B_kf@(uy-ur)
        x = x + dx*dt
        u = -ma.K@(x - xra)
        uy = np.array([u, x[0], x[1]])
        run_data[i] = np.array([0.0, 0.0, x[0], x[1]])

    print('Time for', len(tspan), 'iterations of it_ls_sim:', perf_counter() - start_time)
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

    state_labels = ('$x$ ','$v$ ','$\\theta$ ','$\\omega$ ')
    for i in range(4):
        plt.plot(time_series,x_1[:,i],linewidth=2,label=state_labels[i]+'1')
        plt.plot(time_series,x_2[:,i],linewidth=2,label=state_labels[i]+'2')
        plt.xlabel('Time')
        plt.ylabel('State')
        plt.legend()
        plt.show()
    

def run_comparison():
    tspan = np.arange(0,10,0.01)
    x0 = np.array([0,0,np.pi+0.2,0]) # Initial condition
    xr = np.array([0,0,np.pi,0])      # Reference position 

    method1 = Method(ode_angle_only)
    method2 = Method(kf_angle_only)

    compare_solution_methods(method1, method2, tspan, x0, xr)


#########################################################
# four state solution
#
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

    uy = np.array([0.0, x0[0], x0[2], x0[3]])
    uy_r = np.array([0, xr[0], xr[2], xr[3]])
    u_noise = np.array([0.0])
    u_true = np.array([0.0])

    
    for i in range(len(tspan)):
        av_noise = np.random.normal(0.0,sqrt(angle_vel_var))
        a_noise = np.random.normal(0.0,0.0001)

        dx_true = md.A@(x_true-xr) + md.B@u_true
        x_true = x_true + dx_true*dt
        u_true[0] = -md.K@(x_true - xr)
        run_data_true[i] = x_true

        x_noise[2] += a_noise
        x_noise[3] += av_noise  
        dx_noise = md.A@(x_noise-xr) + md.B@u_noise
        x_noise = x_noise + dx_noise*dt
        u_noise[0] = -md.K@(x_noise - xr)
        run_data_noise[i] = x_noise

        dx_kf = md.A_kf@(x_kf-xr) + md.B_kf@(uy-uy_r)
        x_kf = x_kf + dx_kf*dt
        uy[0] = -md.K@(x_kf - xr)
        uy[1] = x_kf[0]
        uy[2] = x_kf[2] + a_noise
        uy[3] = x_kf[3] + av_noise
        run_data_kf[i] = x_kf
       

    # plt.rcParams['figure.figsize'] = [8, 8]
    # plt.rcParams.update({'font.size': 18})
    # plt.rcParams.update({
    # "text.usetex": True,
    # "font.family": "serif"
    # })
    
    i = 3
    plt.plot(tspan,run_data_noise[:,i],linewidth=1,label=(r'$\omega$ true + noise'))
    plt.plot(tspan,run_data_kf[:,i],linewidth=1,label=r'$\omega$ KF')
    plt.plot(tspan,run_data_true[:,i],linewidth=1,label=r'$\omega$ true')
    plt.title('Four Variable Solution')
    plt.xlabel('Time')
    plt.ylabel('State')
    plt.legend()
    plt.savefig("KFangular_velocity.pdf", format="pdf", bbox_inches="tight")
    plt.show()

    i=2
    plt.plot(tspan,run_data_noise[:,i],linewidth=1,label=r'$\theta$ true + noise')
    plt.plot(tspan,run_data_kf[:,i],linewidth=1,label=r'$\theta$ KF')
    plt.plot(tspan,run_data_true[:,i],linewidth=1,label=r'$\theta$ true')
    plt.title('Four Variable Solution')
    plt.xlabel('Time')
    plt.ylabel('State')
    plt.legend()
    plt.savefig("KFangle.pdf", format="pdf", bbox_inches="tight")
    plt.show()


#########################################################
# two state solution
#
# Here we compare three simulations
#     - standard linear system with no noise (the true state)
#     - standard linear system noise
#     - Kalman Filter system with the same noisy values
#########################################################
def kf_comparison_plot_angle_only():
    tspan = np.arange(0,5,0.01)
    x0 = np.array([np.pi+0.2,0]) # Initial condition
    xr = np.array([np.pi,0])      # Reference position 
    dt = tspan[1]-tspan[0]

    sys_c = ss(ma.A_kf, ma.B_kf, ma.C_kf, ma.D_kf)
    sys_d = c2d(sys_c, dt, 'zoh')

    

    run_data_noise = np.empty([len(tspan),2])
    run_data_kf = np.empty([len(tspan),2])
    run_data_true = np.empty([len(tspan),2])
    run_data_kf_d = np.empty([len(tspan),2])

    run_data_noise[0] = x0
    run_data_kf[0] = x0
    run_data_true[0] = x0
    run_data_kf_d[0] = x0
    
    x_noise = x0
    x_kf = x0
    x_true = x0
    x_kf_d = x0

    uy = np.array([0.0, x0[0], x0[1]])
    uy_r = np.array([0.0, xr[0], xr[1]])
    u_noise = np.array([0.0])
    u_true = np.array([0.0])
    uy_d = np.array([0.0, x0[0], x0[1]])
    uy_r_d = np.array([0.0, xr[0], xr[1]])

    for i in range(len(tspan)):
        av_noise = np.random.normal(0.0,0.01)
        a_noise = np.random.normal(0.0,0.001)

        dx_true = ma.A@(x_true-xr) + ma.B@u_true
        x_true = x_true + dx_true*dt
        u_true[0] = -ma.K@(x_true - xr)
        run_data_true[i] = x_true

        x_noise[0] += a_noise
        x_noise[1] += av_noise  
        dx_noise = ma.A@(x_noise-xr) + ma.B@u_noise
        x_noise = x_noise + dx_noise*dt
        u_noise[0] = -ma.K@(x_noise - xr)
        run_data_noise[i] = x_noise

        dx_kf = ma.A_kf@(x_kf - xr) + ma.B_kf@(uy-uy_r)
        x_kf = x_kf + dx_kf*dt
        uy[0] = -ma.K@(x_kf - xr)
        uy[1] = x_kf[0] + a_noise
        uy[2] = x_kf[1] + av_noise
        run_data_kf[i] = x_kf

        x_kf_d = sys_d.A@(x_kf_d - xr) + sys_d.B@(uy_d-uy_r_d) + xr
        uy_d[0] = -ma.K@(x_kf_d - xr)
        uy_d[1] = x_kf_d[0] + a_noise
        uy_d[2] = x_kf_d[1] + av_noise
        run_data_kf_d[i] = x_kf_d

    # plt.rcParams['figure.figsize'] = [8, 8]
    # plt.rcParams.update({'font.size': 18})
    # plt.rcParams.update({
    # "text.usetex": True,
    # "font.family": "serif"
    # })
    
    i = 0
    plt.plot(tspan,run_data_noise[:,i],linewidth=1,label=(r'$\omega$ true + noise'))
    plt.plot(tspan,run_data_kf[:,i],linewidth=1,label=r'$\omega$ KF')
    plt.plot(tspan,run_data_kf_d[:,i],linewidth=1,label=r'$\omega$ KF discrete')
    plt.plot(tspan,run_data_true[:,i],linewidth=1,label=r'$\omega$ true')
    plt.title('Two Variable Solution')
    plt.xlabel('Time')
    plt.ylabel('State')
    plt.legend()
    plt.savefig("KFangular_velocity2.pdf", format="pdf", bbox_inches="tight")
    plt.show()

    i=1
    plt.plot(tspan,run_data_noise[:,i],linewidth=1,label=r'$\theta$ true + noise')
    plt.plot(tspan,run_data_kf[:,i],linewidth=1,label=r'$\theta$ KF')
    plt.plot(tspan,run_data_kf_d[:,i],linewidth=1,label=r'$\theta$ KF discrete')
    plt.plot(tspan,run_data_true[:,i],linewidth=1,label=r'$\theta$ true')
    plt.title('Two Variable Solution')
    plt.xlabel('Time')
    plt.ylabel('State')
    plt.legend()
    plt.savefig("KFangle2.pdf", format="pdf", bbox_inches="tight")
    plt.show()


run_comparison()