import numpy as np
from control.matlab import lqr
from model_constants import *



#########################################################
# this function returns the change in state of the robot
# based on the full differential equations
#
#########################################################
def equations_of_motion(x,t,u):
    Sx = np.sin(x[2])
    Cx = np.cos(x[2])
    D = (M+m*(Sx**2))

    dx = np.zeros(4)
    dx[0] = x[1]
    dx[1] = (1/D)*(-m*g*Cx*Sx + m*L*(x[3]**2)*Sx - d*x[1] + u)
    dx[2] = x[3]
    dx[3] = (1/(D*L))*((m+M)*g*Sx - Cx*(m*L*(x[3]**2)*Sx - d*x[1]) - Cx*u)
    
    return dx

# State Space Model
class SSPendModelConstants:
    # equations of motion linearized about vertical pendulum position
    A = np.array([[0, 1, 0, 0],\
                [0, -d/M, -m*g/M, 0],\
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

    Q = np.array([[0.1, 0, 0, 0],\
                [0, 0.1, 0, 0],\
                [0, 0, 4 / (5 * np.pi), 0],\
                [0, 0, 0, 0.01]])
    
    R = 1 / max_torque

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
    D = (M+m*(Sx**2))
    
    dx = np.zeros(2)
    dx[0] = x[1]
    dx[1] = (1/(D*L))*((m+M)*g*Sx - Cx*(m*L*(x[1]**2)*Sx) - Cx*u)

    return dx

class SSPendModelTwoVarConstants:
    # equations of motion linearized about vertical pendulum position
    A = np.array([[0, 1],\
                [-(m+M)*g/(M*L), 0]])

    # linearization of control matrix
    B = np.array([0,1/(M*L)]).reshape((2,1))


    Q = np.array([[4 / (5 * np.pi), 0],\
                [0, 0.01]])
    R = 1 / max_torque
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


from control.matlab import obsv
md = SSPendModel()

def print_two_var_eigenvalues():
    md2 = SSPendModelTwoVar()
    K = md2.K.reshape((1,2))
    sys_matrix = md2.A - md2.B @ K
    eigenvalues, eigenvectors = np.linalg.eig(sys_matrix)
    print('A-BK', sys_matrix)
    print('eigenvalues: \n', eigenvalues)


# print the system eigenvalues
# print_two_var_eigenvalues()

# test that the system is observable
# print(np.linalg.matrix_rank(obsv(md.A, md.C).transpose()))