import matplotlib.pyplot as plt
from scipy import integrate
import numpy as np
from control.matlab import lqr


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


def run_simulation():

    m = 0.29        # kilograms
    M = 0.765       # kilograms
    L = 0.16        # meters
    g = -9.81       # meters / sec^2
    d = 0           # d is a damping factor

    A = np.array([[0, 1, 0, 0],\
                [0, -d/M, m*g/M, 0],\
                [0, 0, 0, 1],\
                [0, -d/(M*L), -(m+M)*g/(M*L), 0]])

    B = np.array([0,1/M,0,1/(M*L)]).reshape((4,1))

    Q = np.array([[1,0,0,0],\
                [0,1,0,0],\
                [0,0,1,0],\
                [0,0,0,1]])
    R = 1
    K = lqr(A,B,Q,R)[0][0]


    plt.rcParams['figure.figsize'] = [8, 8]
    plt.rcParams.update({'font.size': 18})
    plt.rcParams['animation.html'] = 'jshtml'

    # ## Simulate closed-loop system
    tspan = np.arange(0,10,0.01)
    x0 = np.array([1,0,np.pi,0]) # Initial condition
    wr = np.array([0,0,np.pi,0])      # Reference position
    u = lambda x: -K@(x-wr)           # Control law


    x = integrate.odeint(pendcart,x0,tspan,args=(m,M,L,g,d,u))

    plot_labels = ('x','v','theta','omega')
    [plt.plot(tspan,x[:,j],linewidth=2,label=plot_labels[j]) for j in range(4)]
    plt.xlabel('Time')
    plt.ylabel('State')

    plt.legend()
    plt.show()


run_simulation()