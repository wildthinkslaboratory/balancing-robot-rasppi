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
# Examples of a distrubances are giving the robot
# a push or rolling over a bump in the floor
Vd = np.eye(4) 

# This is our sensor noise matrix
# it contains the variance for our position and gyro sensors
av_var = 0.0000026
Vn = np.array([[1, 0], \
               [0, 1]])

Kf = lqr(A.transpose(), C.transpose(), Vd, Vn)[0].transpose()

# These are our A,B,C,D matrices for the Kalman Filter system
A_kf = A - (Kf @ C)    
B_kf = np.concatenate((B, Kf), axis=1)
C_kf = np.eye(4)
D_kf = np.zeros_like(B_kf)