
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
    K = 0.0815       # motor constant in Nm / sqrt(W)
    ST = 0.45        # motor stall torque in Nm
    MT = ST * 0.9    # max motor torque allowed in a run Nm
    DC = 1 / MT      # duty coefficient
    dt = 0.005
    # we add our Q and R matrices
    Q = np.array([[0.001, 0, 0, 0],\
            [0, 1, 0, 0],\
            [0, 0, 4 / (5 * np.pi), 0],\
            [0, 0, 0, 1]])

    R = np.array([[1]])
    Q2V = np.array([[4 / (5 * np.pi), 0],\
                    [0, 1]])

    R2V = np.array([[1]])


# inherit all the values of ModelConstants
class ExperimentalConstants(ModelConstants):
    #L = 0.2        # length of pendulum (meters)

    Q = np.array([[100, 0, 0, 0],\
            [0, 1, 0, 0],\
            [0, 0, 4 / (5 * np.pi), 0],\
            [0, 0, 0, 1]])

    R = np.array([[1]])
    Q2V = np.array([[1,0],\
                    [0, 1]])

    R2V = np.array([[1]])

    pass