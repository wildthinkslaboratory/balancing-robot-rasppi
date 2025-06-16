import numpy as np
from time import sleep
from InterruptTimer import InterruptTimer
from model import SSPendModelTwoVar
from motors import BRMotors
from imu import ImuSensor
from utilities import output_data
from model_constants import speed_from_u

md = SSPendModelTwoVar()

debug = False
output_data_to_file = True

dT = 0.01
timeout = 30


imu_sensor = ImuSensor()        # mpu6050 gyro/accelorometer access                    
motors = BRMotors(dT)           # DC motors with encoder

############################################
# These are all our loop variables 

############################################
run_data = list()  
sleep(3)

angle_init = imu_sensor.raw_angle_rad()

x = np.array([angle_init,0.0])               # this is our estimated state
x_r = np.array([np.pi+(2*np.pi/180),0.0])             # Reference position / Goal state                        
uy = np.array([0.0, x[0], x[1]])          # our input values [ u, x_sensor, a_sensor, av_sensor]   
uy_r = np.array([0.0, x_r[0], x_r[1]])    # input values goal state



############################################
# LQE implementation
# 
############################################




def update_run_data():
    global run_data
    run_data.append([x[0], x[1], uy[0], uy[1], uy[2]])

def loop_iteration():
    global x
    global uy

    uy[1] = imu_sensor.raw_angle_rad()  # this needs to be with pi in the up position
    uy[2] = imu_sensor.raw_angular_velocity_rad()

    # estimate the state
    dx = md.A_kf@(x - x_r) + md.B_kf@(uy-uy_r)
    x = x + dx*dT

    # compute the control value u, and update motor duty cycle
    # for testing purposes we'll break this into parts
    # error = x - x_r
    # uy[0] = -(md.K[0]* error[0] + md.K[1]* error[1])

    uy[0] = -md.K@(x - x_r)  
    motors.run(speed_from_u(uy[0]))
    

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


