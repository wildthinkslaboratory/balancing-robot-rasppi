import json
import numpy as np
from control.matlab import lqr
from time import sleep
from motors import BRMotors
from imu import ImuSensor

debug = False

def generateK():

    m = 0.29        # kilograms
    M = 0.765       # kilograms
    L = 0.16     # meters
    g = -9.81    # meters / sec^2
    d = 0        # d is a damping factor
    
    b = 1 # pendulum up (b=1)

    A = np.array([[0,1,0,0],\
                [0,-d/M,b*m*g/M,0],\
                [0,0,0,1],\
                [0,-b*d/(M*L),-b*(m+M)*g/(M*L),0]])

    B = np.array([0,1/M,0,b/(M*L)]).reshape((4,1))

    Q = np.array([[1,0,0,0],\
                [0,1,0,0],\
                [0,0,50,0],\
                [0,0,0,1]])
    R = 0.1
    return lqr(A,B,Q,R)[0]



x0 = np.array([0,0,np.pi,0])      # Initial condition
wr = np.array([0,0,np.pi+(2.5*np.pi/180),0])      # Reference position

K = generateK() 

k1 = -18
k2 = 0

def get_output(state):
    u = (-K@(state - wr))[0]
    output = k1*u + k2
    if abs(output) > 100.0:
        output = 100.0 if output > 0 else -100.0
    return u, output

def convert_angle(a):
    # convert to radian and shift 180 degrees
    radian = (a * np.pi / 180) + np.pi
    if (radian >= 2 * np.pi):
        radian -= 2 * np.pi
    return radian

imu_sensor = ImuSensor()
motors = BRMotors()

run_data = list()

try:
    for i in range(2000):
        (a_deg, av_deg) = imu_sensor.angle_data()
        a = convert_angle(a_deg)
        av = av_deg * np.pi / 180
        (x,v) = motors.position_data()
        state = np.array([x, v, a, av])
        u, output = get_output(state)
        
        if debug:
            run_data.append([x,v,a_deg,av_deg,u,output])
    
        motors.run(output / 100)
        sleep(0.003)


except KeyboardInterrupt:
    print("Program Interrupted")

if debug:
    # Serializing json
    json_object = json.dumps(run_data, indent=4)
    
    # Writing to sample.json
    with open("data.json", "w") as outfile:
        outfile.write(json_object)

