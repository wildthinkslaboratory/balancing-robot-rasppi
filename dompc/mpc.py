import numpy as np
import do_mpc


def MPC(model, dt, silence_solver=False):

    mpc = do_mpc.controller.MPC(model)

    # not sure what all of these do yet
    mpc.settings.n_horizon = 10
    mpc.settings.n_robust = 1
    mpc.settings.open_loop = 0
    mpc.settings.t_step = dt
    mpc.settings.state_discretization = 'collocation'
    mpc.settings.collocation_type = 'radau'
    mpc.settings.collocation_deg = 2
    mpc.settings.collocation_ni = 1
    mpc.settings.store_full_solution = True
    
    if silence_solver:
        mpc.settings.supress_ipopt_output()

    Q = np.array([[1, 0, 0, 0],\
                [0, 1, 0, 0],\
                [0, 0, 1, 0],\
                [0, 0, 0, 1]])
    R = 10

    # build up the cost function 
    xr = np.array([0.0,0.0, np.pi, 0.0])
    x = np.array([model.x['x'], model.x['xdot'], model.x['theta'], model.x['thetadot']])
    error = x - xr.transpose()
    lterm = error.transpose() @ Q @ error

    mpc.set_objective(lterm=lterm, mterm=lterm) #

    mpc.set_rterm(u=1)

    # we could add some bounds on variables here
    # hard and soft constraints are allowed

    # we could set uncertainty values here

    mpc.setup()
    return mpc
