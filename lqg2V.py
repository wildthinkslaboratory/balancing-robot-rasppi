import numpy as np
from time import sleep
from InterruptTimer import InterruptTimer
from motors import BRMotors
from imu import ImuSensor
from balance_bot2V import lqgdBot as bb
from utilities import output_data, clip, countdown
from builds import ModelConstants

mc = ModelConstants()

debug = False
output_data_to_file = True

timeout = 5
dT = bb.dt

imu_sensor = ImuSensor()        # mpu6050 gyro/accelorometer access                    
motors = BRMotors(dT)           # DC motors with encoder

############################################
# These are all our loop variables 

############################################
run_data = list()  
countdown(5)

angle_init = imu_sensor.raw_angle_rad()

x = np.array([angle_init,0.0])               # this is our estimated state                    
u = np.array([0.0])          # our input values 
y = np.array([x[0], x[1]])   # sensor data

############################################
# LQE implementation
# 
############################################




def update_run_data():
    global run_data
    run_data.append([x[0], x[1], u[0], y[0], y[1]])

def loop_iteration():
    global x
    global u
    global y

    y[0] = imu_sensor.raw_angle_rad()  # this needs to be with pi in the up position
    y[1] = imu_sensor.raw_angular_velocity_rad()

    # estimate the state
    x = bb.next_state(x, u, y)
    
    # constrain the input to the allowed motor speeds
    u_unclipped = bb.control_input(x)
    u[0] = clip(u_unclipped[0], -1, 1)
    motors.run(u[0])


    
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


