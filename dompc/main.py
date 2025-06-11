import numpy as np
import matplotlib.pyplot as plt
import do_mpc

from model import model
from lqr import LQR
from simulator import simulator


store_results = True


dt = 0.01
model,linearmodel = model()
lqr = LQR(linearmodel, dt)
sim = simulator(model, dt)


# initial state
x0 = np.array([[1.0],[0.0],[np.pi],[0.0]])
sim.x0 = x0


for k in range(1000):
    u0 = lqr.make_step(x0)
    y_next = sim.make_step(u0)
    x0 = y_next


# plot graphics
fig, ax, graphics = do_mpc.graphics.default_plot(sim.data, figsize=(12,8))
graphics.plot_results()
graphics.reset_axes()
plt.show()


# Store results:
if store_results:
    do_mpc.data.save_results([simulator], 'two_wheel_balancing_bot_LQR')