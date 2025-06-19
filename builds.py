

M_ = 0.05        # wheels plus motors (kilograms) 
m_ = 1       # rest of the robot (kilograms)
L_ = 0.1        # length of pendulum (meters)
g_ = -9.81       # gravity, (meters / sec^2)
d_ = 0.001       # d is a damping factor
r_ = 0.0325      # radius of the wheels
R_ = 4           # motor coil resistance in Ohms
K_ = 0.0815       # motor constant in Nm / sqrt(W)
ST_ = 0.5        # motor stall torque in Nm
MT_ = ST_ * 0.9  # max motor torque allowed in a run Nm
DC_ = 1 / MT_    # duty coefficient
