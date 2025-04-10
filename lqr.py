import json
import numpy as np
from control.matlab import lqr
from time import sleep, perf_counter
from motors import BRMotors
from imu2 import ImuSensor
from InterruptTimer import InterruptTimer


debug = False
output_data_to_file = True

dT = 0.006
time = 10

# generate our K matrix for LQR
def generateK():

    m = 0.29        # kilograms
    M = 0.765       # kilograms
    L = 0.16        # meters
    g = -9.81       # meters / sec^2
    d = 0           # d is a damping factor

    A = np.array([[0,1,0,0],\
                [0,-d/M,m*g/M,0],\
                [0,0,0,1],\
                [0,-d/(M*L),-(m+M)*g/(M*L),0]])

    B = np.array([0,1/M,0,1/(M*L)]).reshape((4,1))

    Q = np.array([[1,0,0,0],\
                [0,1,0,0],\
                [0,0,1,0],\
                [0,0,0,1]])
    R = 1
    return lqr(A,B,Q,R)[0]


K = generateK()[0]                                # K (gain vector)
x0 = np.array([0,0,np.pi,0])                      # Initial condition
wr = np.array([0,0,np.pi+(2.5*np.pi/180),0])      # Reference position

imu_sensor = ImuSensor(dT)                          
motors = BRMotors(dT)

x = 0.0              # store the state in some file level variables
v = 0.0
a = np.pi
av = 0.0
u = 0.0
duty_coeff = 0.18


def update_motors():
    global u
    start_time = perf_counter()  # Start timer
    u = K[0]*(x-wr[0]) + K[1]*(v-wr[1]) + K[2]*(a-wr[2]) + K[3]*(av-wr[3])
    motors.run(u * duty_coeff)
    

def update_state():
    global a, av, x, v
    a, av = imu_sensor.complementary_filter()
    a = convert_angle(a)
    x, v = motors.position_data()


def convert_angle(a):
    # convert to radian and shift 180 degrees
    radian = a + np.pi
    if (radian >= 2 * np.pi):
        radian -= 2 * np.pi
    return radian

run_data = list()

# t0 = perf_counter()
# for i in range(1000):
#     imu_sensor.get_raw_data()       # <- actual I2C read
#     update_state()
# print("Total time for 1000 loops:", perf_counter() - t0)

def update_output_data():
    global run_data
    run_data.append([x,v,3*(a - np.pi),av,u])

timer1 = InterruptTimer(dT, update_motors, time)
timer2 = InterruptTimer(dT, update_state, time)
timer3 = InterruptTimer(0.01 , update_output_data, time)

timer1.start()
timer2.start()

if output_data_to_file: 
    timer3.start()

quitTime = perf_counter() + time

try:
    while perf_counter() < quitTime:
        imu_sensor.get_raw_data()
        
except KeyboardInterrupt:
    print("Program Interrupted")



if output_data_to_file:
    # Serializing json
    json_object = json.dumps(run_data, indent=4)
    
    # Writing to sample.json
    with open("data.json", "w") as outfile:
        outfile.write(json_object)

