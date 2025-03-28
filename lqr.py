import numpy as np
from control.matlab import lqr
from time import sleep
from motors import BRMotors
from imu import ImuSensor


def generateK():

    m = 0.29        # kilograms
    M = 0.765       # kilograms
    L = 0.14     # meters
    g = -9.81    # meters / sec^2
    d = 0        # d is a damping factor
    
    b = 1 # pendulum up (b=1)

    A = np.array([[0,1,0,0],\
                [0,-d/M,b*m*g/M,0],\
                [0,0,0,1],\
                [0,-b*d/(M*L),-b*(m+M)*g/(M*L),0]])

    B = np.array([0,1/M,0,b/(M*L)]).reshape((4,1))

    Q = np.eye(4)
    R = 0.1
    return lqr(A,B,Q,R)[0]



x0 = np.array([0,0,np.pi,0])      # Initial condition
wr = np.array([0,0,np.pi,0])      # Reference position

K = generateK() 

k1 = 1
k2 = 0

def get_output(state):
    u = -K@(state - wr)
    output = k1*u + k2
    if abs(output) > 100.0:
        output = 100.0 if output > 0 else -100.0
    return output

imu_sensor = ImuSensor()
motors = BRMotors()

try:
    while True:
        (a, av) = imu_sensor.angle_data()
        (x,v) = motors.position_data()
        state = np.array([x, v, a, av])
        output = get_output(state)

        motors.run(output / 100)
        time.sleep(0.01)


except KeyboardInterrupt:
    print("Program Interrupted")
