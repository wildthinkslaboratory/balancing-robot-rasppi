import numpy as np
import do_mpc


def MPC(model, dt, silence_solver=False):



    mpc = do_mpc.controller.MPC(model)

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

    xr = np.array([0.0,0.0, np.pi, 0.0])
    x = np.array([model.x['x'], model.x['xdot'], model.x['theta'], model.x['thetadot']])
    error = x - xr.transpose()
    lterm = error.transpose() @ Q @ error

    print(lterm)
    mpc.set_objective(lterm=lterm, mterm=lterm) #

    mpc.set_rterm(u=1)


    # mpc.bounds['lower', '_x', 'C_a'] = 0.1
    # mpc.bounds['lower', '_x', 'C_b'] = 0.1
    # mpc.bounds['lower', '_x', 'T_R'] = 50
    # mpc.bounds['lower', '_x', 'T_K'] = 50

    # mpc.bounds['upper', '_x', 'C_a'] = 2
    # mpc.bounds['upper', '_x', 'C_b'] = 2
    # mpc.bounds['upper', '_x', 'T_K'] = 140

    # mpc.bounds['lower', '_u', 'F'] = 5
    # mpc.bounds['lower', '_u', 'Q_dot'] = -8500

    # mpc.bounds['upper', '_u', 'F'] = 100
    # mpc.bounds['upper', '_u', 'Q_dot'] = 0.0

    # # Instead of having a regular bound on T_R:
    # #mpc.bounds['upper', '_x', 'T_R'] = 140
    # # We can also have soft consraints as part of the set_nl_cons method:
    # mpc.set_nl_cons('T_R', model.x['T_R'], ub=140, soft_constraint=True, penalty_term_cons=1e2)


    # alpha_var = np.array([1., 1.05, 0.95])
    # beta_var = np.array([1., 1.1, 0.9])

    # mpc.set_uncertainty_values(alpha = alpha_var, beta = beta_var)

    mpc.setup()
    return mpc
