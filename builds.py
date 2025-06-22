
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
    d = 0.001       # d is a damping factor
    r = 0.0325      # radius of the wheels
    R = 4           # motor coil resistance in Ohms
    ST = 0.45        # motor stall torque in Nm
    t = 0.1           # this is for tuning the horz force to speed translation
    SC = t * ST / r   # this is used to translate horizontal force to speed
    dt = 0.01
    # we add our Q and R matrices
    Q = np.array([[1, 0, 0, 0],\
            [0, 1, 0, 0],\
            [0, 0, 1, 0],\
            [0, 0, 0, 1]])

    R = np.array([[1]])
    Q2V = np.array([[1,0],\
                    [0, 1]])

    R2V = np.array([[1]])
    Q_kf = np.eye(2)
    R_kf = np.eye(2)


# inherit all the values of ModelConstants
class ExperimentalConstants(ModelConstants):
    #L = 0.06        # length of pendulum (meters)

    Q2V = np.array([[1,0],\
                    [0, 1]])

    R2V = np.array([[0.001]])
    Q_kf = np.array([[0.001,0],\
                    [0, 0.001]])
    R_kf = np.array([[1,0],\
                    [0,1]])

    pass