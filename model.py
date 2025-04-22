import numpy as np
from control.matlab import lqr, place

# measured noise in our 
angle_vel_var = 0.0000026
angle_var = 0.0000026


#########################################################
# this function returns the change in state of the robot
# based on the full differential equations
#########################################################
def equations_of_motion(x,t,m,M,L,g,d,uf):
    u = uf(x) # evaluate anonymous function at x
    Sx = np.sin(x[2])
    Cx = np.cos(x[2])
    D = m*L*L*(M+m*(1-Cx**2))
    
    dx = np.zeros(4)
    dx[0] = x[1]
    dx[1] = (1/D)*(-(m**2)*(L**2)*g*Cx*Sx + m*(L**2)*(m*L*(x[3]**2)*Sx - d*x[1])) + m*L*L*(1/D)*u
    dx[2] = x[3]
    dx[3] = (1/D)*((m+M)*m*g*L*Sx - m*L*Cx*(m*L*(x[3]**2)*Sx - d*x[1])) - m*L*Cx*(1/D)*u;
    
    return dx



class LQRModelConstants:

        m = 0.29        # mass of pendulum (kilograms)
        M = 0.765       # mass of cart (kilograms)
        L = 0.16        # length of pendulum (meters)
        g = -9.81       # gravity, (meters / sec^2)
        d = 0.001       # d is a damping factor

        # equations of motion linearized about vertical pendulum position
        A = np.array([[0, 1, 0, 0],\
                    [0, -d/M, m*g/M, 0],\
                    [0, 0, 0, 1],\
                    [0, -d/(M*L), -(m+M)*g/(M*L), 0]])

        # linearization of control matrix
        B = np.array([0,1/M,0,1/(M*L)]).reshape((4,1))

        Q = np.array([[1, 0, 0, 0],\
                    [0, 0.01, 0, 0],\
                    [0, 0, 10, 0],\
                    [0, 0, 0, 0.1]])
        R = 1
        K = lqr(A,B,Q,R)[0][0]


class LQRModel:
    def __init__(self):
        # Set constants from separate classes as attributes
        for cls in [LQRModelConstants]:
            for key, value in cls.__dict__.items():
                if not key.startswith("__"):
                    self.__dict__.update(**{key: value})

    def __setattr__(self, name, value):
        raise TypeError("Model values are immutable")
    

    def dx_from_equations(self,x,u):
        uf = lambda y: u
        return equations_of_motion(x,0.0,self.m,self.M,self.L,self.g,self.d,uf)



###########################################
# Generate our Kalman Filter

###########################################
class KalmanFilterXThetaOmegaConstants:
    model = LQRModel()
    # C is our measurement model
    # we are measuring position and angular velocity
    # the position is read from the motor encoders and
    # angular velocity is from the gyro
    C = np.array([[1, 0, 0, 0], \
                [0, 0, 1, 0], \
                [0, 0, 0, 1]]) 

    # This is our state disturbance matrix
    # Examples of a distrubances are giving the robot
    # a push or rolling over a bump in the floor
    Vd = np.eye(4) 

    # This is our sensor noise matrix
    # it contains the variance for our position and gyro sensors
    Vn = np.eye(3)

    Kf = lqr(model.A.transpose(), C.transpose(), Vd, Vn)[0].transpose()

    # These are our A,B,C,D matrices for the Kalman Filter system
    A_kf = model.A - (Kf @ C)    
    B_kf = np.concatenate((model.B, Kf), axis=1)
    C_kf = np.eye(4)
    D_kf = np.zeros_like(B_kf)


class KalmanFilterXThetaOmega:
    def __init__(self):
        # Set constants from separate classes as attributes
        for cls in [KalmanFilterXThetaOmegaConstants]:
            for key, value in cls.__dict__.items():
                if not key.startswith("__"):
                    self.__dict__.update(**{key: value})

    def __setattr__(self, name, value):
        raise TypeError("Model values are immutable")


class KalmanFilterXThetaConstants:
    model = LQRModel()
    # C is our measurement model
    # we are measuring position and angular velocity
    # the position is read from the motor encoders and
    # angular velocity is from the gyro
    C = np.array([[1, 0, 0, 0], \
                [0, 0, 1, 0]])


    # This is our state disturbance matrix
    # Examples of a distrubances are giving the robot
    # a push or rolling over a bump in the floor
    Vd = np.eye(4) 

    # This is our sensor noise matrix
    # it contains the variance for our position and gyro sensors
    Vn = np.eye(2)

    Kf = lqr(model.A.transpose(), C.transpose(), Vd, Vn)[0].transpose()

    # These are our A,B,C,D matrices for the Kalman Filter system
    A_kf = model.A - (Kf @ C)    
    B_kf = np.concatenate((model.B, Kf), axis=1)
    C_kf = np.eye(4)
    D_kf = np.zeros_like(B_kf)


class KalmanFilterXTheta:
    def __init__(self):
        # Set constants from separate classes as attributes
        for cls in [KalmanFilterXThetaConstants]:
            for key, value in cls.__dict__.items():
                if not key.startswith("__"):
                    self.__dict__.update(**{key: value})

    def __setattr__(self, name, value):
        raise TypeError("Model values are immutable")

