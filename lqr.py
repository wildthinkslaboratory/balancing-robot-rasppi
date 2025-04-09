import json
import numpy as np
from control.matlab import lqr
from time import sleep
from motors import BRMotors
from imu2 import ImuSensor


debug = False
output_data_to_file = True

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


def update_motors():
    error = state - wr
    u = -(K[0] * error[0] + K[1] * error[1] + K[2] * error[2] + K[3] * error[3]) * 0.5
    

# void update_motors()
# {
 
#   u[0] = ( k[0]*(encoder_set_point  - average_theta)
#           +k[1]*(pitch_set_point-pitch) 
#           +k[2]*(velocity_set_point-average_RPM)
#           +k[3]*(pitch_dot_set_point-pitch_dot));
          
#   u[1] = (k1[0]*yaw +k1[1]*yaw_dot) ;
#   v[0] = (0.5 *(u[0] + u[1])); // for right motor
#   v[1] = (0.5 *(u[0] - u[1])); // for left motor
#   drive_motors();

 
# }




x0 = np.array([0,0,np.pi,0])      # Initial condition
wr = np.array([0,0,np.pi+(2.5*np.pi/180),0])      # Reference position

K = generateK()

k1 = -40
k2 = 0

def get_output(state):
    u = (-K@(state - wr))[0]
    output = k1*u + k2
    if abs(output) > 100.0:
        output = 100.0 if output > 0 else -100.0
    return u, output

def convert_angle(a):
    # convert to radian and shift 180 degrees
    radian = a + np.pi
    if (radian >= 2 * np.pi):
        radian -= 2 * np.pi
    return radian

imu_sensor = ImuSensor()
motors = BRMotors()

run_data = list()

try:
    for i in range(800):
        imu_sensor.get_raw_data()
        a, av = imu_sensor.complementary_filter()
        # (a_deg, av_deg) = imu_sensor.angle_data()
        a = convert_angle(a)
        # av = av_deg * np.pi / 180
        (x,v) = motors.position_data()
        state = np.array([x, v, a, av])
        u, output = get_output(state)
        
        if output_data_to_file:
            run_data.append([x,v,a,av,u,output])
    
        motors.run(output / 100)
        sleep(0.003)


except KeyboardInterrupt:
    print("Program Interrupted")

if output_data_to_file:
    # Serializing json
    json_object = json.dumps(run_data, indent=4)
    
    # Writing to sample.json
    with open("data.json", "w") as outfile:
        outfile.write(json_object)

