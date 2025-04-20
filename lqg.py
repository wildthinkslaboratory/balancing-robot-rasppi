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
x = np.array([0.0,0.0,np.pi,0.0])               # this is our estimated state
x_r = np.array([1.0,0.0,np.pi,0.0])             # Reference position / Goal state                        
uy = np.array([0.0, x[0], x[2], x[3]])          # our input values [ u, x_sensor, a_sensor, av_sensor ]   
uy_r = np.array([0.0, x_r[0], x_r[2], x_r[3]]).reshape((4,1))  # input values goal state
duty_coeff = 0.18
dT = 0.01
timeout = 2


############################################
# LQE implementation
# 
############################################

imu_sensor = ImuSensor()        # mpu6050 gyro/accelorometer access                    
motors = BRMotors(dT)           # DC motors with encoder

run_data = list()  

def read_sensors():
    global uy
    uy[1] = motors.position()
    uy[2] = imu_sensor.raw_angle_rad()  # this needs to be with pi in the up position
    uy[3] = imu_sensor.raw_angular_velocity_rad()


def update_run_data():
    global run_data
    run_data.append([x[0], x[1], x[2], x[3], uy[0], uy[1], uy[2], uy[3]])

def loop_iteration():
    global x
    global uy

    uy[1] = motors.position()
    uy[2] = imu_sensor.raw_angle_rad()  # this needs to be with pi in the up position
    uy[3] = imu_sensor.raw_angular_velocity_rad()

    # estimate the state
    dx = (A_kf@(x - x_r) + (B_kf@(uy.reshape((4,1))-uy_r)).transpose())[0]
    x = x + dx*dT

    # compute the control value u, and update motor duty cycle
    uy[0] = -K@(x - x_r)  
    motors.run(uy[0] * duty_coeff)
    

# the main functions are called in timers that
# keep strict time deltas between calls
loop_timer = InterruptTimer(dT, loop_iteration, timeout)
# sensor_timer = InterruptTimer(dT , read_sensors, timeout)
output_timer = InterruptTimer(dT, update_run_data, timeout)

loop_timer.start()
# sensor_timer.start()
output_timer.start()


# collect runtime data and output it to file
if output_data_to_file:
    output_timer.start()
    while loop_timer.running:
        sleep(dT)
    output_data(run_data, "data.json")


