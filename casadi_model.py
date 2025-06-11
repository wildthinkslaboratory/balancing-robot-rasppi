import casadi as ca
from casadi import sin, cos
from model_constants import *
import numpy as np

# helper functions
def DM2Arr(dm):
    return np.array(dm.full())


# First we build up the equations of motion for the problem 
# We create Casadi symbolic variables for the state
x = ca.SX.sym('x')  
xdot = ca.SX.sym('xdot')
theta = ca.SX.sym('theta')
thetadot = ca.SX.sym('thetadot')
state = ca.vertcat(
    x,
    xdot,
    theta,
    thetadot
)
num_states = state.numel()
u = ca.SX.sym('u')             # control input
num_controls = 1

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
f = ca.Function('f', [state, u], [RHS])

from pendulum_model import SSPendModel
md = SSPendModel()

f_linear = md.A @ state + md.B * u

print(f_linear)


class NLMPC:
    def __init__(self, x_0, x_goal, dt, horizon):
        self.x_0 = ca.DM(x_0)       # starting state at beginning of run
        self.x = ca.DM(x_0)         # current state
        self.x_goal = ca.DM(x_goal)
        self.dt = dt
        self.N = horizon

        X = ca.SX.sym('X', num_states, self.N + 1)     # state variables for each time step
        U = ca.SX.sym('U', num_controls, self.N)       # control variables for each time step
        P = ca.SX.sym('P', 2 * num_states)             # stores initial state and target state
        Q = ca.diagcat(1, 0.01, 10, 0.1)   # weights for cost of state errors
        R = ca.diagcat(1)                  # weight for cost of motors

        # now we build up the cost function and the constraints for
        # state consistency g
        g = X[:, 0] - P[:num_states]  
        cost = 0.0
        for k in range(self.N):
            state_k = X[:, k]    # state at time step k
            control_k = U[:, k]  # control at time step k

            # build up the cost function
            state_error_cost = (state_k - P[num_states:]).T @ Q @ (state_k - P[num_states:])
            control_cost = control_k.T @ R @ control_k
            cost = cost + state_error_cost + control_cost

            # now we build up the discrete constraints for the state
            # with the runge kutta method
            next_state = X[:, k+1]
            k1 = f(state_k, control_k)
            k2 = f(state_k + self.dt/2*k1, control_k)
            k3 = f(state_k + self.dt/2*k2, control_k)
            k4 = f(state_k + self.dt * k3, control_k)
            next_state_RK4 = state_k + (self.dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            g = ca.vertcat(g, next_state - next_state_RK4)

        # these are all of our optimization variables
        OPT_variables = ca.vertcat(
            X.reshape((-1, 1)),   # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
            U.reshape((-1, 1))
        )

        # dictionary for defining our solver
        nlp_prob = {
            'f': cost,
            'x': OPT_variables,
            'g': g,
            'p': P
        }

        # dictionary for our solver options
        opts = {
            'ipopt': {
                'max_iter': 2000,
                'print_level': 0,
                'acceptable_tol': 1e-8,
                'acceptable_obj_change_tol': 1e-6
            },
            'print_time': 0
        }
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)
    
    def get_control(self, state):
        self.x = ca.DM(state) 
        self.P = ca.vertcat(
            self.x,   
            self.x_goal   
        )
        # build up our starting guess structure
        # is there a better way to build these?
        x0 = ca.reshape(ca.repmat(self.x, 1, self.N+1), num_states*(self.N+1),1)
        u0 = ca.reshape(ca.DM.zeros((num_controls, self.N)), num_controls*self.N, 1)
        self.starting_guess = ca.vertcat(x0, u0)
        self.solution = self.solver(
            x0=self.starting_guess,
            p=self.P
        )
        u = self.solution['x'][num_states * (self.N+1)]
        return u

    def get_state(self):
        pass




# # define upper and lower bounds for our optimization variables
# lbx = ca.DM.zeros((num_states*(N+1) + num_controls*N, 1))
# ubx = ca.DM.zeros((num_states*(N+1) + num_controls*N, 1))

# lbx[0: num_states*(N+1): num_states] = -ca.inf     # x lower bound
# lbx[1: num_states*(N+1): num_states] = -ca.inf     # xdot lower bound
# lbx[2: num_states*(N+1): num_states] = -ca.inf     # theta lower bound
# lbx[3: num_states*(N+1): num_states] = -ca.inf     # thetadot lower bound

# ubx[0: num_states*(N+1): num_states] = ca.inf     # x upper bound
# ubx[1: num_states*(N+1): num_states] = ca.inf     # xdot upper bound
# ubx[2: num_states*(N+1): num_states] = ca.inf     # theta upper bound
# ubx[3: num_states*(N+1): num_states] = ca.inf     # thetadot upper bound

# lbx[num_states*(N+1):] = -ca.inf                  # u lower bound 
# ubx[num_states*(N+1):] = ca.inf                   # u upper bound 


# args = {
#     'lbg': ca.DM.zeros((num_states*(N+1), 1)),  # constraints lower bound
#     'ubg': ca.DM.zeros((num_states*(N+1), 1)),  # constraints upper bound
#     'lbx': lbx,
#     'ubx': ubx
# }

# def shift_timestep(step_horizon, t0, state_init, u, f):
#     f_value = f(state_init, u[:, 0])
#     next_state = ca.DM.full(state_init + (step_horizon * f_value))

#     t0 = t0 + step_horizon
#     u0 = ca.horzcat(
#         u[:, 1:],
#         ca.reshape(u[:, -1], -1, 1)
#     )

#     return t0, next_state, u0


# t0 = 0
# state_init = ca.DM([0.0, 0.0, np.pi + 0.1, 0.0])        # initial state
# state_target = ca.DM([0.0, 0.0, np.pi, 0.0])            # target state

# # # xx = DM(state_init)
# t = ca.DM(t0)

# u0 = ca.DM.zeros((num_controls, N))        # initial control
# X0 = ca.repmat(state_init, 1, N+1)         # initial state full


# mpc_iter = 0
# cat_states = DM2Arr(X0)
# cat_controls = DM2Arr(u0[:, 0])
# times = np.array([[0]])
# sim_time = 40      # simulation time



# from pendulum_model import equations_of_motion
# from pendulum_model import SSPendModel
# md = SSPendModel()
# x_md = np.array([0.0, 0.0, np.pi + 0.1, 0.0])
# xr_md = np.array([0,0,np.pi,0]) 
# u_md = 0.0
# dt_md = 0.1


    
# ###############################################################################

# if __name__ == '__main__':

#     while ((ca.norm_2(state_init - state_target) > 1e-1) and (mpc_iter < sim_time)):

#         args['p'] = ca.vertcat(
#             state_init,    # current state
#             state_target   # target state
#         )
#         # optimization variable current state
#         args['x0'] = ca.vertcat(
#             ca.reshape(X0, num_states*(N+1), 1),
#             ca.reshape(u0, num_controls*N, 1)
#         )

#         sol = csolver.solver(
#             x0=args['x0'],
#             lbx=args['lbx'],
#             ubx=args['ubx'],
#             lbg=args['lbg'],
#             ubg=args['ubg'],
#             p=args['p']
#         )

#         u = ca.reshape(sol['x'][num_states * (N + 1):], num_controls, N)
#         X0 = ca.reshape(sol['x'][: num_states * (N+1)], num_states, N+1)
       
#         t0, state_init, u0 = shift_timestep(step_horizon, t0, state_init, u, f)
#         X0 = ca.horzcat(
#             X0[:, 1:],
#             ca.reshape(X0[:, -1], -1, 1)
#         )

#         mpc_iter = mpc_iter + 1


#     ss_error = ca.norm_2(state_init - state_target)



class LMPC:
    def __init__(self, x_0, x_goal, dt, horizon):
        self.x_0 = ca.DM(x_0)       # starting state at beginning of run
        self.x = ca.DM(x_0)         # current state
        self.x_goal = ca.DM(x_goal)
        self.dt = dt
        self.N = horizon

        X = ca.SX.sym('X', num_states, self.N + 1)     # state variables for each time step
        U = ca.SX.sym('U', num_controls, self.N)       # control variables for each time step
        P = ca.SX.sym('P', 2 * num_states)             # stores initial state and target state
        Q = ca.diagcat(1, 0.01, 10, 0.1)   # weights for cost of state errors
        R = ca.diagcat(1)                  # weight for cost of motors

        # now we build up the cost function and the constraints for
        # state consistency g
        g = X[:, 0] - P[:num_states]  
        cost = 0.0
        for k in range(self.N):
            state_k = X[:, k]    # state at time step k
            control_k = U[:, k]  # control at time step k

            # build up the cost function
            state_error_cost = (state_k - P[num_states:]).T @ Q @ (state_k - P[num_states:])
            control_cost = control_k.T @ R @ control_k
            cost = cost + state_error_cost + control_cost

            # now we build up the discrete constraints for the state
            # with the runge kutta method
            next_state = X[:, k+1]
            k1 = f(state_k, control_k)
            k2 = f(state_k + self.dt/2*k1, control_k)
            k3 = f(state_k + self.dt/2*k2, control_k)
            k4 = f(state_k + self.dt * k3, control_k)
            next_state_RK4 = state_k + (self.dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            g = ca.vertcat(g, next_state - next_state_RK4)

        # these are all of our optimization variables
        OPT_variables = ca.vertcat(
            X.reshape((-1, 1)),   # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
            U.reshape((-1, 1))
        )

        # dictionary for defining our solver
        nlp_prob = {
            'f': cost,
            'x': OPT_variables,
            'g': g,
            'p': P
        }

        # dictionary for our solver options
        opts = {
            'ipopt': {
                'max_iter': 2000,
                'print_level': 0,
                'acceptable_tol': 1e-8,
                'acceptable_obj_change_tol': 1e-6
            },
            'print_time': 0
        }
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)
    
    def get_control(self, state):
        self.x = ca.DM(state) 
        self.P = ca.vertcat(
            self.x,   
            self.x_goal   
        )
        # build up our starting guess structure
        # is there a better way to build these?
        x0 = ca.reshape(ca.repmat(self.x, 1, self.N+1), num_states*(self.N+1),1)
        u0 = ca.reshape(ca.DM.zeros((num_controls, self.N)), num_controls*self.N, 1)
        self.starting_guess = ca.vertcat(x0, u0)
        self.solution = self.solver(
            x0=self.starting_guess,
            p=self.P
        )
        u = self.solution['x'][num_states * (self.N+1)]
        return u

    def get_state(self):
        pass



    
