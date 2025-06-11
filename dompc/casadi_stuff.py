# This creates the equations of motion function using casadi 
# variables. Then we take the jacobian of the equations
# and linearize the model around the vertical position.
# We compare the generated A and B matrices to our values
# from our original model.


import casadi as ca
from casadi import sin, cos
import numpy as np

import sys
import os
subfolder_path = os.path.join(os.path.dirname(__file__), "../")
sys.path.append(subfolder_path)



# We create Casadi symbolic variables for the state
x = ca.MX.sym('x')  
xdot = ca.MX.sym('xdot')
theta = ca.MX.sym('theta')
thetadot = ca.MX.sym('thetadot')
u = ca.MX.sym('u')             # control input
state = ca.vertcat(
    x,
    xdot,
    theta,
    thetadot,
)

# create casadi constants
M = ca.MX.sym('M') 
m = ca.MX.sym('m') 
L = ca.MX.sym('L') 
g = ca.MX.sym('g') 
d = ca.MX.sym('d')


constants = ca.vertcat( M, m, L, g, d )

# we build the nonlinear function f for the equations of motion
# we begin with some partial expressions to make the formulas easier to build
denominator = M + m*(sin(theta)**2)
n0 = -m*g*sin(theta)*cos(theta)
n1 = m*L*(sin(theta))*(thetadot)**2 - d*xdot + u
n2 = (m + M)*g*sin(theta)
RHS = ca.vertcat( 
    xdot, 
    (n0 + n1) / denominator, 
    thetadot, 
    (n2+(-cos(theta))*(n1)) / (L*denominator)
    )

# make our casadi equation 
f = ca.Function('f', [state, u, constants], [RHS], ['state', 'u', 'constants'], ['dx'])

# compute the jacobian of the system
f_jacobian = f.jacobian()

# these are the numerical value for the constants
import model_constants as mc
constants_ = [mc.M, mc.m, mc.L, mc.g, mc.d]

# linearize in the pendulum up position
linearized_up = f_jacobian(state = [0.0, 0.0, np.pi, 0.0], u=0.0, constants = constants_)

from model import SSPendModel
md = SSPendModel()

# test to see if we get same results as our model
print(linearized_up['jac_dx_state'])
print(md.A)
print(linearized_up['jac_dx_u'])
print(md.B)

