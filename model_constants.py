M = 0.05        # wheels plus motors (kilograms) 
m = 1       # rest of the robot (kilograms)
L = 0.06        # length of pendulum (meters)
g = -9.81       # gravity, (meters / sec^2)
d = 0.001       # d is a damping factor
r = 0.0325      # radius of the wheels


# the max torque of the motors is 5 kg cm or approximately 0.5 Nm
# each motor provides torque so the total max torque is 1 Nm
# we are setting the max value to 80% of that
max_torque = 1.0 * 0.8
duty_coeff = 0.8 / max_torque

# Take the input force u and multiply by radius r
# to get desired torque. Then we scale it to the motor
# speed by muliplying by the duty coefficient. Finally,
# we divide it in half to send half the torque to each 
# motor.
def speed_from_u(u):
    return u * r * duty_coeff * 0.5


# test Izzy's collaborator status