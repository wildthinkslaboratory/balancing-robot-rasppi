import numpy as np
from time import sleep
from InterruptTimer import InterruptTimer
from model import A, K, B, Kf, C
from motors import BRMotors
from imu3 import ImuSensor
from utilities import output_data


debug = False
output_data_to_file = True

############################################
# These are all our loop variables 

############################################
y = np.array([0.0, 0.0])                # y is our raw sensor readings, position and angular velocity
state = np.array([0,0,np.pi,0])                # this is our estimated state
wr = np.array([0,0,np.pi,0])                    # Reference position / Goal state
u = 0.0                                         # our control variable for motor torque
duty_coeff = 0.18
dT = 0.01
timeout = 0.5       


############################################
# LQE implementation
# 
############################################

imu_sensor = ImuSensor()      # mpu6050 gyro/accelorometer access                    
motors = BRMotors(dT)           # DC motors with encoder

run_data = list()  

def loop_iteration():
    global state
    global u

    # estimate the state
    dstate = (A@(state - wr) + (B*u).transpose() + Kf@(y - C@state))[0]  
    state = state + dstate*dT

    run_data.append([state[0], state[1], state[2], state[3], u, y[0], y[1]])

   
    # compute the control value u, and update motor duty cycle
    u = -K@(state - wr)  
    # motors.run(u * duty_coeff)
    

def read_sensors():
    global y
    y[0] = motors.position()
    y[1] = imu_sensor.raw_angular_velocity_rad()


# the main functions are called in timers that
# keep strict time deltas between calls
loop_timer = InterruptTimer(dT, loop_iteration, timeout)
sensor_timer = InterruptTimer(dT , read_sensors, timeout)

loop_timer.start()
sensor_timer.start()


# collect runtime data and output it to file
if output_data_to_file:
    while loop_timer.running:
        sleep(dT)
    output_data(run_data)


