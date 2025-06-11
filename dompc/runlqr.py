import numpy as np
import matplotlib.pyplot as plt
from do_mpc import graphics
from do_mpc.data import save_results

from dompcmodel import model
from lqr import LQR
from simulator import simulator

from mpc import MPC


store_results = True

dt = 0.01
model,linearmodel = model()
lqr = LQR(linearmodel, dt)
sim = simulator(model, dt)

# initial state
x0 = np.array([[1.0],[0.0],[np.pi],[0.0]])
sim.x0 = x0


for k in range(200):
    u0 = lqr.make_step(x0)      # compute the control input
    y_next = sim.make_step(u0)  # estimate the state with new control input
    x0 = y_next                 # update state


# plot graphics
fig, ax, graphics = graphics.default_plot(sim.data, figsize=(12,8))
graphics.plot_results()
graphics.reset_axes()
plt.savefig("lqr.pdf", format="pdf", bbox_inches="tight")
plt.show()



# Store results:
if store_results:
    save_results([sim], 'two_wheel_balancing_bot_LQR')
    