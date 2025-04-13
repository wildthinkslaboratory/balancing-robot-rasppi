import json
import numpy as np
from control.matlab import lqr
from time import sleep, perf_counter
from InterruptTimer import InterruptTimer
from motors import BRMotors
from imu2 import ImuSensor

debug = False
output_data_to_file = True

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
state = np.array([0,0,np.pi,0])                # this is our estimated state
wr = np.array([0,0,np.pi,0])                    # Reference position / Goal state
u = 0.0                                         # our control variable for motor torque
duty_coeff = 0.18
dT = 0.01
timeout = 10.0


############################################
# function to test the Kalman Filter

############################################
def test_Kfilter():
    x = np.array([-1,0,np.pi,0])        # pick start and goal states        
    wr = np.array([0,0,np.pi,0]) 
    y = np.array([0.0, 0.0])
    dt = 0.01

    for i in range(1000):
        u = -K@(x-wr)
        dx = (A@(x - wr) + (B*u).transpose() + Kf@(y - C@state))[0]  # 
        x = x + dx*dt
        y = C@state + np.array([np.random.normal(0.0, 0.001), np.random.normal(0.0,0.001)])
        run_data.append([x[0], x[1], x[2], x[3], u])
    output_data()


############################################
# LQE implementation
# 
############################################

imu_sensor = ImuSensor(dT)      # mpu6050 gyro/accelorometer access                    
motors = BRMotors(dT)           # DC motors with encoder

def loop_iteration():
    global state
    global u

    # estimate the state
    dstate = (A@(state - wr) + (B*u).transpose() + Kf@(y - C@state))[0]  
    state = state + dstate*dT
   
    # compute the control value u, and update motor duty cycle
    u = K[0]*(x-wr[0]) + K[1]*(v-wr[1]) + K[2]*(a-wr[2]) + K[3]*(av-wr[3])
    motors.run(u * duty_coeff)
    

def read_sensors():
    global y
    y[0] = motors.position()
    y[1] = imu_sensor.get_angular_velocity()


# the main functions are called in timers that
# keep strict time deltas between calls
loop_timer = InterruptTimer(dT, loop_iteration, timeout)
sensor_timer = InterruptTimer(0.01 , read_sensors, timeout)

loop_timer.start()
sensor_timer.start()



# collect runtime data and output it to file

if output_data_to_file:
    run_data = list()  
    while loop_timer.running:
        run_data.append([state[0], state[1], state[2], state[3], u])
        sleep(0.1)

    # Serializing json
    json_object = json.dumps(run_data, indent=4)
    
    # Writing to sample.json
    with open("data.json", "w") as outfile:
        outfile.write(json_object)