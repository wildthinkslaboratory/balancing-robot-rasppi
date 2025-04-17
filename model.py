# note: to Izzy
# I put the model in a seperate file because we need to 
# access it in multiple places. If we ever need to edit
# the model we wouldn't be able to keep it all synched.

import numpy as np
from control.matlab import lqr, lqe

m = 0.29        # mass of pendulum (kilograms)
M = 0.765       # mass of cart (kilograms)
L = 0.16        # length of pendulum (meters)
g = -9.81       # gravity, (meters / sec^2)
d = 0           # d is a damping factor

# equations of motion linearized about vertical pendulum position
A = np.array([[0, 1, 0, 0],\
            [0, -d/M, m*g/M, 0],\
            [0, 0, 0, 1],\
            [0, -d/(M*L), -(m+M)*g/(M*L), 0]])

# linearization of control matrix
B = np.array([0,1/M,0,1/(M*L)]).reshape((4,1))

Q = np.array([[1,0,0,0],\
            [0,1,0,0],\
            [0,0,1,0],\
            [0,0,0,1]])
R = 1
K = lqr(A,B,Q,R)[0][0]


############################################
# Generate our Kalman Filter

############################################

# C is our measurement model
# we are measuring position and angular velocity
# the position is read from the motor encoders and
# angular velocity is from the gyro
C = np.array([[1, 0, 0, 0], \
              [0, 0, 0, 1]]) 

# This is our state disturbance matrix
# it contains the variance for state disturbances
# Examples of a distrubances are giving the robot
# a push or rolling over a bump in the floor
Vd = np.eye(4) * 0.00001         

# This is our sensor noise matrix
# it contains the variance for our position and gyro sensors
Vn = np.array([[0.000000001, 0], \
               [0, 0.00000026]])  


Kf = lqr(A.transpose(), C.transpose(), Vd, Vn)[0].transpose()

    
