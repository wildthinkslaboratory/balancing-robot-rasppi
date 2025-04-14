# note: to Izzy
# I put the model in a seperate file because we need to 
# access it in multiple places. If we ever need to edit
# the model we wouldn't be able to keep it all synched.

import numpy as np
from control.matlab import lqr

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
Vd = np.eye(4) * 0.001          

# This is our sensor noise matrix
# it contains the variance for our position and gyro sensors
Vn = np.array([[0.0001, 0], \
               [0, 0.00000026]])     
Kf = lqr(A.transpose(), C.transpose(), Vd, Vn)[0].transpose()


############################################
# function to test the Kalman Filter's ability to estimate the state from 
# the gyro and encoder data. Here we just add some noise to our sensor
# data y to simulate a live sensor.
############################################

def test_Kfilter():
    x = np.array([-1,0,np.pi,0])        # pick start and goal states        
    wr = np.array([0,0,np.pi,0]) 
    y = np.array([0.0, 0.0])
    dt = 0.01
    state_data = [[], [], [], []]
    u_data = []
    sensor_data = [[], []]
    run_data = []

    for i in range(1000):              # simulate 
        u = -K@(x - wr)
        dx = (A@x + (B*u).transpose() + Kf@(y - C@x))[0] 

        x = x + dx*dt
        y = C@x + np.array([np.random.normal(0.0, 0.0001), np.random.normal(0.0,0.0000026)])

        for i in range(4):             # collect data
            state_data[i].append(x[i])
        u_data.append(u)
        for i in range(2):
            sensor_data[i].append(y[i])
        run_data.append([x[0], x[1], x[2], x[3], u, y[0], y[1]])

    output_data(run_data, "filter_test.json")
    plot_labels = ['x','v','a','av', 'x dat', 'av dat']
    [plt.plot(sensor_data[j], label=plot_labels[j + 4]) for j in range(2)]
    [plt.plot(state_data[j], label=plot_labels[j]) for j in range(4)]
    plt.plot(u_data, label='u')
    plt.xlabel('Time')
    plt.ylabel('state')

    plt.legend()
    plt.show()
    

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from utilities import output_data
    test_Kfilter()