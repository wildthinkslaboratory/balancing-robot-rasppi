M = 0.05        # wheels plus motors (kilograms) 
m = 1       # rest of the robot (kilograms)
L = 0.06        # length of pendulum (meters)
g = -9.81       # gravity, (meters / sec^2)
d = 0.001       # d is a damping factor
r = 0.0325      # radius of the wheels


# the max torque of the motors is 5 kg cm or approximately 0.5 Nm
# we are setting the max value to 80% of that
max_torque = 0.5 * 0.8
duty_coeff = 0.8 / max_torque

def speed_from_u(u):
    return u * r * duty_coeff