import numpy as np
from time import sleep
from InterruptTimer import InterruptTimer
from model import LQRModel, KalmanFilterXTheta
from motors import BRMotors
from imu3 import ImuSensor
from utilities import output_data

md = LQRModel()
kf = KalmanFilterXTheta()

debug = False
output_data_to_file = True

############################################
# These are all our loop variables 

############################################
x = np.array([0.0,0.0,np.pi,0.0])               # this is our estimated state
x_r = np.array([0.0,0.0,np.pi,0.0])             # Reference position / Goal state                        
uy = np.array([0.0, x[0], x[2], x[3]])          # our input values [ u, x_sensor, a_sensor, av_sensor]   
uy_r = np.array([0.0, x_r[0], x_r[2], x[3]])    # input values goal state
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


def update_run_data():
    global run_data
    run_data.append([x[0], x[1], x[2], x[3], uy[0], uy[1], uy[2]])

def loop_iteration():
    global x
    global uy

    uy[1] = motors.position()
    uy[2] = imu_sensor.raw_angle_rad()  # this needs to be with pi in the up position
    uy[3] = imu_sensor.raw_angular_velocity_rad()

    # estimate the state
    dx = kf.A@(x - x_r) + kf.B@(uy-uy_r)
    x = x + dx*dT

    # compute the control value u, and update motor duty cycle
    # for testing purposes we'll break this into parts
    error = x - x_r
    uy[0] = -(md.K[0]* error[0] + md.K[1]* error[1] + md.K[2]* error[2] + md.K[3]* error[3])
    # uy[0] = -md.K@(x - x_r)  
    motors.run(uy[0] * duty_coeff)
    

# the main functions are called in timers that
# keep strict time deltas between calls
loop_timer = InterruptTimer(dT, loop_iteration, timeout)
output_timer = InterruptTimer(dT, update_run_data, timeout)

loop_timer.start()
output_timer.start()


# collect runtime data and output it to file
if output_data_to_file:
    output_timer.start()
    while loop_timer.running:
        sleep(dT)
    output_data(run_data, "data.json")


