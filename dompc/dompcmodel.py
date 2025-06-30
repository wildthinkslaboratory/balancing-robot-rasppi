import numpy as np
import casadi as ca
from casadi import sin, cos
import do_mpc

import sys
import os
subfolder_path = os.path.join(os.path.dirname(__file__), "../")
sys.path.append(subfolder_path)

M = 0.05       # wheels plus motors (kilograms) 
m = 1          # rest of the robot (kilograms)
L = 0.6        # length of pendulum (meters)
g = -9.81       # gravity, (meters / sec^2)
d = 0.01       # d is a damping factor
r = 0.0325      # radius of the wheels in meters
R = 4           # motor coil resistance in Ohms
ST = 0.45        # motor stall torque in Nm 
t = 1           # this is for tuning the horz force to speed translation
dt = 0.01

def model(symvar_type='SX'):

    # Initialize the model
    model_type = 'continuous' 
    model = do_mpc.model.Model(model_type, symvar_type)

    # state variables
    x = model.set_variable(var_type='_x', var_name='x', shape=(1,1))
    xdot = model.set_variable(var_type='_x', var_name='xdot', shape=(1,1))
    theta = model.set_variable(var_type='_x', var_name='theta', shape=(1,1))
    thetadot = model.set_variable(var_type='_x', var_name='thetadot', shape=(1,1))

    # Input
    u = model.set_variable(var_type='_u', var_name='u')

    # defining the equations of motion for the model
    # we begin with some partial expressions to make the formulas easier to build
    denominator = M + m*(sin(theta)**2)
    n0 = -m*g*sin(theta)*cos(theta)
    n1 = m*L*(sin(theta))*(thetadot)**2 - d*xdot + u
    n2 = (m + M)*g*sin(theta)

    model.set_rhs('x', xdot)
    model.set_rhs('xdot', (n0 + n1) / denominator)
    model.set_rhs('theta', thetadot)
    model.set_rhs('thetadot', (n2+(-cos(theta))*(n1)) / (L*denominator))

    model.setup()


    # Steady state values
    uss = np.array([[0.0]])
    xss = np.array([[0.0],[0.0],[np.pi],[0.0]])

    # Linearize the non-linear model
    linearmodel = do_mpc.model.linearize(model, xss, uss)
    
    # this returns both the non-linear model and the linear model
    return model,linearmodel