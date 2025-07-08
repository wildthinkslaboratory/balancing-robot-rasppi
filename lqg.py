import numpy as np
from time import sleep
from datetime import datetime
from InterruptTimer import InterruptTimer
from motors import BRMotors
from imu import ImuSensor
from segway import lqgdBot as bb
from utilities import output_data, clip, countdown
from builds import SegwayConstants

mc = SegwayConstants()

debug = False
output_data_to_file = True

timeout = mc.timeout
dT = bb.dt

imu_sensor = ImuSensor()        # mpu6050 gyro/accelorometer access                    
motors = BRMotors(dT)           # DC motors with encoder

############################################
# These are all our loop variables 

############################################
run_data = list()  
countdown(5)

angle_init = imu_sensor.raw_angle_rad()

x = np.array([0.0, 0.0, angle_init,0.0])    # this is our estimated state                     
u = np.array([0.0])                         # our input values 
y = np.array([x[0], x[2], x[3]])            # sensor data

u_unclipped = u # for debugging

############################################
# LQE implementation
# 
############################################




def update_run_data():
    global run_data
    run_data.append([x[0], x[1], x[2], x[3], u[0], y[0], y[1], y[2], u_unclipped[0]])

def loop_iteration():
    global x
    global u
    global y
    global u_unclipped


    y[0] = motors.position()
    print('position', y[0])
    y[1] = imu_sensor.raw_angle_rad()  # this needs to be with pi in the up position
    y[2] = imu_sensor.raw_angular_velocity_rad()

    # estimate the state
    x = bb.next_state(x, u, y)

    # constrain the input to the allowed motor speeds
    # divide the torque between the wheels
    u_unclipped = bb.control_input(x) * 1.5
    u[0] = clip(u_unclipped[0], -2, 2)
    motors.run(u[0]/2)
    

    
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
    
    motors.stop()

    # log data with tuning parameters and constants
    log = {}
    t = datetime.now().strftime("%Y-%m-%d%H:%M:%S")
    log['time'] = t
    log['constants'] = mc.dictionary()
    log['model'] = bb.dictionary()

    notes = input('Enter any runtime notes: ')
    log['notes'] = notes

    log['rundata'] = run_data
    output_data(log, 'run_data/' + t + '.json')
    output_data(log, 'run_data/data.json')


