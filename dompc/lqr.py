import numpy as np
import do_mpc

# an lqr can calculate the input u from the state x
def LQR(model, dt):

    # their lqr only allows discrete models for some reason
    model_dc = model.discretize(dt)
    
    # Initialize the controller
    lqr = do_mpc.controller.LQR(model_dc)
    
    # Initialize the parameters
    lqr.settings.t_step = dt
    lqr.settings.n_horizon = None # infinite horizon
    
    # Setting the objective
    Q = np.identity(4)
    R = np.identity(1)
    # Rdelu = np.identity(1)

    lqr.set_objective(Q=Q, R=R)
    # lqr.set_rterm(delR = Rdelu)
                     
    lqr.setup()

    # set the goal/reference state
    lqr.set_setpoint(xss = np.array([[0.0],[0.0], [np.pi], [0.0]]))   

    return lqr