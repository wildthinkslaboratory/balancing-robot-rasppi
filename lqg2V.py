import numpy as np
from time import sleep
from InterruptTimer import InterruptTimer
from motors import BRMotors
from imu import ImuSensor
from balance_bot2V import lqgdBot as bb
from utilities import output_data, clip
from builds import ModelConstants

mc = ModelConstants()

debug = False
output_data_to_file = True

timeout = 8
dT = bb.dt

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
    x = bb.get_next_state(x, uy)

    # constrain the input to the allowed motor speeds
    uy = bb.get_control_input(x)
    motor_speed = clip(uy[0] / mc.SC, -0.9, 0.9)
    uy[0] = motor_speed * mc.SC    # this is the force our motors can actually apply
    motors.run(motor_speed)


    
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


