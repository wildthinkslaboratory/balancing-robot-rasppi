
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
    
    # we add our Q and R matrices
    # focus on angle, ignore angular velocity
    # it's volatile because of high magnitude eigenvalue
    Q = np.diag([1, 1, 1, 1])
    R = np.array([[1]])

    Q_kf = np.diag([1,1,1,1])
    R_kf = np.diag([1,1,1])

    Q2V = np.array([[1,0],\
                    [0, 1]])
    R2V = np.array([[1]])
    Q_kf2V = np.eye(2)
    R_kf2V = np.eye(2)


# inherit all the values of ModelConstants
class ExperimentalConstants(ModelConstants):
    dt = 0.01

    Q = np.diag([1, 1, 1, 1])
    R = np.array([[1000]])

    # Q_kf = np.diag([1,1,1,1]) / 5000
    # R_kf = np.diag([0.004,0.0001,0.00015])

    # Q2V = np.array([[1,0],\
    #                 [0, 1]])
    # R2V = np.array([[0.1]])

    Q_kf2V = np.diag([1,1]) / 6000
    R_kf2V = np.diag([0.00015,0.00015])



"""Countdown: 1gyro readings in rad/s
gyro noise variance 2.5439875792968847e-06
average error gyro error:  0.0014338266967203557


testing angular accelleration readings
position your bot upright
Countdown: 1
accelerometer Y variance 0.0014442795206713013
average accelerometer Y:  0.028198192513327443


accelerometer Z variance 0.00200945709510295
average accelerometer Z:  9.80832927173857


readings in radians
accelerometer angle variance 1.5002323219619387e-05
average angle:  3.1417278128868312"""