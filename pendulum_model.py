import numpy as np
from control.matlab import lqr


m = 0.29        # mass of pendulum (kilograms)
M = 0.765       # mass of cart (kilograms)
L = 0.16        # length of pendulum (meters)
g = -9.81       # gravity, (meters / sec^2)
d = 0.001       # d is a damping factor


#########################################################
# this function returns the change in state of the robot
# based on the full differential equations
#
# This is based of Steve Brunton's Control Theory book code
#
#########################################################
def equations_of_motion(x,t,u):
    Sx = np.sin(x[2])
    Cx = np.cos(x[2])
    D = m*L*L*(M+m*(1-Cx**2))
    
    dx = np.zeros(4)
    dx[0] = x[1]
    dx[1] = (1/D)*(-(m**2)*(L**2)*g*Cx*Sx + m*(L**2)*(m*L*(x[3]**2)*Sx - d*x[1])) + m*L*L*(1/D)*u
    dx[2] = x[3]
    dx[3] = (1/D)*((m+M)*m*g*L*Sx - m*L*Cx*(m*L*(x[3]**2)*Sx - d*x[1])) - m*L*Cx*(1/D)*u;
    
    return dx

# State Space Model
class SSPendModelConstants:
    # equations of motion linearized about vertical pendulum position
    A = np.array([[0, 1, 0, 0],\
                [0, -d/M, m*g/M, 0],\
                [0, 0, 0, 1],\
                [0, -d/(M*L), -(m+M)*g/(M*L), 0]])

    # linearization of control matrix
    B = np.array([0,1/M,0,1/(M*L)]).reshape((4,1))

    # C is our measurement model
    # we are measuring position, angle and angular velocity
    # the position is read from the motor encoders 
    # angle is from the accelerometer
    # angular velocity is from the gyro
    C = np.array([[1, 0, 0, 0], \
                [0, 0, 1, 0], \
                [0, 0, 0, 1]]) 
    
    D = np.zeros_like(B)

    Q = np.array([[1, 0, 0, 0],\
                [0, 0.01, 0, 0],\
                [0, 0, 10, 0],\
                [0, 0, 0, 0.1]])
    R = 1
    K = lqr(A,B,Q,R)[0][0] 

    # This is our state disturbance matrix
    # Examples of a distrubances are giving the robot
    # a push or rolling over a bump in the floor
    Vd = np.eye(4) 

    # This is our sensor noise matrix
    # it contains the variance for our position and gyro sensors
    Vn = np.eye(3)

    # Generate our Kalman Filter
    Kf = lqr(A.transpose(), C.transpose(), Vd, Vn)[0].transpose()

    # These are our A,B,C,D matrices for the Kalman Filter system
    A_kf = A - (Kf @ C)    
    B_kf = np.concatenate((B, Kf), axis=1)
    C_kf = np.eye(4)
    D_kf = np.zeros_like(B_kf)

    

# This just makes it easy to access our model constants as 
# a class. This way we can have multiple models and not
# get the A's B's and K's mixed up
class SSPendModel:
    def __init__(self):
        # Set constants from separate classes as attributes
        for cls in [SSPendModelConstants]:
            for key, value in cls.__dict__.items():
                if not key.startswith("__"):
                    self.__dict__.update(**{key: value})

    # these are constants. This keeps them read only
    def __setattr__(self, name, value):
        raise TypeError("Model values are immutable")



def equations_of_motion_two_var(x,t,u):
    Sx = np.sin(x[0])
    Cx = np.cos(x[0])
    D = m*L*L*(M+m*(1-Cx**2))
    
    dx = np.zeros(2)
    dx[0] = x[1]
    dx[1] = (1/D)*((m+M)*m*g*L*Sx - m*L*Cx*(m*L*(x[1]**2)*Sx)) - m*L*Cx*(1/D)*u;

    return dx

class SSPendModelTwoVarConstants:
        # equations of motion linearized about vertical pendulum position
        A = np.array([[0, 1],\
                    [-(m+M)*g/(M*L), 0]])

        # linearization of control matrix
        B = np.array([0,1/(M*L)]).reshape((2,1))

        Q = np.array([[ 1, 0],\
                    [0, 1]])
        R = 1
        K = lqr(A,B,Q,R)[0][0] 
        C = np.eye(2)

        Vd = np.eye(2) 
        Vn = np.eye(2)
        Kf = lqr(A.transpose(), C.transpose(), Vd, Vn)[0].transpose()

        A_kf = A - (Kf @ C)    
        B_kf = np.concatenate((B, Kf), axis=1)
        C_kf = np.eye(2)
        D_kf = np.zeros_like(B_kf)

# This just makes it easy to access our model constants as 
# a class. This way we can have multiple models and not
# get the A's B's and K's mixed up
class SSPendModelTwoVar:
    def __init__(self):
        # Set constants from separate classes as attributes
        for cls in [SSPendModelTwoVarConstants]:
            for key, value in cls.__dict__.items():
                if not key.startswith("__"):
                    self.__dict__.update(**{key: value})

    # these are constants. This keeps them read only
    def __setattr__(self, name, value):
        raise TypeError("Model values are immutable")

