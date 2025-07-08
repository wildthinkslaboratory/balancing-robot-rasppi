
import numpy as np
# This is all the constants for the balancing robot.  
# We store them as classes so you can keep multiple
# packages of constants. This allows you to futz with
# the constants during simulation without messing up
# the actual constants for the robot.

class ModelConstants:
    M = 0.05       # wheels plus motors (kilograms) 
    m = 1          # rest of the robot (kilograms)
    L = 0.06        # length of pendulum (meters)
    g = -9.81       # gravity, (meters / sec^2)
    d = 0.01       # d is a damping factor
    r = 0.0325      # radius of the wheels in meters
    R = 4           # motor coil resistance in Ohms
    ST = 0.45        # motor stall torque in Nm 
    t = 1           # this is for tuning the horz force to speed translation
    dt = 0.01
    balance_point = 3.18
    timeout = 2

    # we add our Q and R matrices
    # focus on angle, ignore angular velocity
    # it's volatile because of high magnitude eigenvalue
    Q = np.diag([1, 1, 1, 1])
    R = np.array([[1]])

    Q_kf = np.diag([1,1,1,1])
    R_kf = np.diag([1,1,1])

    Q2V = np.array([[1,0],\
                    [0, 1]])
    R2V = np.array([[1000]])
    Q_kf2V = np.eye(2)
    R_kf2V = np.eye(2)


# inherit all the values of ModelConstants
class ExperimentalConstants(ModelConstants):
    timeout = 3
    # dt = 0.005

    # # old constants
    # M = 0.29       # wheels plus motors (kilograms) 
    # m = 0.765          # rest of the robot (kilograms)

    # L = 0.1

    # Q2V = np.array([[0.01,0],\
    #                 [0, 1]])
    # R2V = np.array([[1000]])

    # Q_kf2V = np.diag([1,1]) / 5000
    # R_kf2V = np.diag([0.000015, 0.0000025])
    pass


class SegwayConstants:
    def __init__(self):

        self.dt = 0.01
        self.g = 9.81                # gravity m / s^2
        self.r = 0.0325               # wheel radius m
        self.M = 0.25435            # mass of body kg
        self.m = 0.757536             # mass of wheels kg
        self.L = 0.08495273492                 # length from COM body and wheel axis m
        self.I_b =  0.008553260629             # inertia of body kg m^2
        self.I_w = 0.000062147      # inertia of wheel kg m^2
        self.ST = 0.45               # stall torque
        self.timeout = 0.5            # how many seconds the robot will run
        self.balance_point = 0.04

        self.Q = np.diag([1,1,100,10])
        self.R = np.diag([1000])

        self.Q_kf = np.diag([10,1,1,1]) / 100000
        self.R_kf = np.diag([0.0004,0.00015,0.0000025])

        # this stuff all helps with making run data plots
        # I made latex names for my states. They look nice in the simulation plots
        self.state_names = ['$x$ ','$\\dot{x}$ ','$\\theta$ ','$\\dot{\\theta}$ ', 'u', '$x$ s', '$\\theta$ s','$\\dot{\\theta}$ s']
        self.state_structure = [4, 1, 3]  # this means there are 4 states, 1 input and 3 sensor readings
        self.sensor_indexes = [0, 2, 3]

    def dictionary(self):
        d = {}
        d['dt'] = self.dt
        d['timeout'] = self.timeout
        d['balance_point'] = self.balance_point
        d['g'] = self.g
        d['r'] = self.r
        d['M'] = self.M
        d['m'] = self.m
        d['L'] = self.L
        d['I_b'] = self.I_b
        d['I_w'] = self.I_w
        d['ST'] = self.ST
        d['Q'] = self.Q.diagonal().tolist()
        d['R'] = self.R.diagonal().tolist()
        d['Q_kf'] = self.Q_kf.diagonal().tolist()
        d['R_kf'] = self.R_kf.diagonal().tolist()
        d['state_names'] = self.state_names
        d['state_structure'] = self.state_structure
        d['sensor_indexes'] = self.sensor_indexes

        return d

