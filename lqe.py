import json
import numpy as np
from control.matlab import lqr
from time import sleep, perf_counter
from InterruptTimer import InterruptTimer


debug = False
output_data_to_file = True

dT = 0.01
time = 10

############################################
# Generate our K matrix for LQR

############################################

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


############################################
# Generate our Kalman Filter

############################################

# C is our measurement model
# we are measuring position and angular velocity
# the position is read from the motor encoders and
# angular velocity is a combined reading from the
# gyro and accelerometer
C = np.array([[1, 0, 0, 0], \
              [0, 0, 0, 1]]) 

# This is our state disturbance matrix
# it contains the variance for state disturbances
Vd = np.eye(4) * 0.001          

# This is our sensor noise matrix
# it contains the variance for our position and angular velocity sensors
Vn = np.array([[0.001, 0], \
               [0, 0.001]])     
Kf = lqr(A.transpose(), C.transpose(), Vd, Vn)[0].transpose()


############################################
# These are all our loop variables 

############################################
y = np.array([0.0, 0.0])                # y is our raw sensor readings, position and angular velocity
state = np.array([-1,0,np.pi,0])                # this is our estimated state
wr = np.array([0,0,np.pi,0])                    # Reference position / Goal state
u = 0.0                                         # our control variable for motor torque
duty_coeff = 0.18


run_data = list()  # holds data from the run

def output_data():
    # Serializing json
    json_object = json.dumps(run_data, indent=4)
    
    # Writing to sample.json
    with open("data.json", "w") as outfile:
        outfile.write(json_object)

def test_Kfilter():
    for i in range(1000):
        u = -K@(state-wr)
        dstate = (A@(state -wr) + (B*u).transpose() + Kf@(y - C@state))[0] 
        state = state + dstate*0.01
        y = C@state + np.array([np.random.normal(0.0, 0.001), np.random.normal(0.0,0.001)])
        run_data.append([state[0], state[1], state[2], state[3], u])
    output_data()



if output_data_to_file:
    output_data()

