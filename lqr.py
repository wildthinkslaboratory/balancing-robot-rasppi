import json
import numpy as np
from control.matlab import lqr

# checking 
wheel_circ = 0.8        # in meters
pulse_per_rotation = 11 # number of pulses in a wheel rotation

def distance(pulses):
    return pulses/pulse_per_rotation * wheel_circ

def generateK():

    m = 0.2        # kilograms
    M = 0.3        # kilograms
    L = 0.12     # meters
    g = -9.81    # meters / sec^2
    d = 0        # d is a damping factor

    b = 1 # pendulum up (b=1)

    A = np.array([[0,1,0,0],\
                [0,-d/M,b*m*g/M,0],\
                [0,0,0,1],\
                [0,-b*d/(M*L),-b*(m+M)*g/(M*L),0]])

    B = np.array([0,1/M,0,b/(M*L)]).reshape((4,1))

    Q = np.eye(4)
    R = 0.1
    return lqr(A,B,Q,R)[0]



x0 = np.array([-1,0,np.pi+0.1,0]) # Initial condition
wr = np.array([1,0,np.pi,0])      # Reference position
x = x0

K = generateK()

print('K ', K)
u =  -K@(x-wr)  
print(u)

# while True:
#     # read state from sensors 
#     pass