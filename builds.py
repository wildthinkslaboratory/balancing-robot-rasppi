
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
    r = 0.0325      # radius of the wheels
    R = 4           # motor coil resistance in Ohms
    ST = 0.45        # motor stall torque in Nm
    t = 1           # this is for tuning the horz force to speed translation
    dt = 0.01
    # we add our Q and R matrices
    # focus on angle, ignore angular velocity
    # it's volatile because of high magnitude eigenvalue
    Q = np.diag([0.001, 0.001, 1, 100])

    R = np.array([[1]])

    Q2V = np.array([[1,0],\
                    [0, 1]])

    R2V = np.array([[1]])

    Q_kf = np.diag([1,1,1,10])
    R_kf = np.diag([0.01,0.05,0.1])

    Q_kf2V = np.eye(2)
    R_kf2V = np.eye(2)


# inherit all the values of ModelConstants
class ExperimentalConstants(ModelConstants):

    Q2V = np.diag([1, 1])
    R2V = np.array([[1]])
    
    Q_kf2V  = np.diag([1, 1])

    R_kf2V = np.diag([0.04, 0.0000026])
    
    SC = 0.5 * 0.45 / 0.0325 # this is used to translate horizontal force to speed
    pass