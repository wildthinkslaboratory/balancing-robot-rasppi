import numpy as np
from time import sleep
from InterruptTimer import InterruptTimer
from model import A_kf, K, B_kf
from motors import BRMotors
from imu3 import ImuSensor
from utilities import output_data


debug = False
output_data_to_file = True

############################################
# These are all our loop variables 

############################################
x = np.array([0,0,np.pi,0])                   # this is our estimated state
x_r = np.array([0,0,np.pi,0])                 # Reference position / Goal state
u = np.array([0,0,0])                         # our input values [ u, x_sensor, y_sensor ]
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

def read_sensors():
    global u
    u[1] = motors.position()
    u[2] = imu_sensor.raw_angular_velocity_rad()


def loop_iteration():
    global state
    global u

    read_sensors()
    
    # estimate the state
    dx = (A_kf@(x-x_r) + (B_kf@u).transpose())[0]
    x = x + dx*dT

    run_data.append([x[0], x[1], x[2], x[3], u[0], u[1], u[2]])

    # compute the control value u, and update motor duty cycle
    u[0] = -K@(x - x_r)  
    motors.run(u[0] * duty_coeff)
    

# the main functions are called in timers that
# keep strict time deltas between calls
loop_timer = InterruptTimer(dT, loop_iteration, timeout)
# sensor_timer = InterruptTimer(dT , read_sensors, timeout)

loop_timer.start()
# sensor_timer.start()


# collect runtime data and output it to file
if output_data_to_file:
    while loop_timer.running:
        sleep(dT)
    output_data(run_data)


