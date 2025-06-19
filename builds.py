

M_ = 0.05        # wheels plus motors (kilograms) 
m_ = 1       # rest of the robot (kilograms)
L_ = 0.06        # length of pendulum (meters)
g_ = -9.81       # gravity, (meters / sec^2)
d_ = 0.001       # d is a damping factor
r = 0.0325      # radius of the wheels

R_ = 4           # motor coil resistance in Ohms
K_ = 0.0815       # motor constant in Nm / sqrt(W)

motor_stall_torque = 0.5
max_torque = motor_stall_torque * 0.9

# this is the maximum input u that we allow. 
# we use this in our R matrix when computing gain K
# it's a soft constraint really
# we want to stay under 80% of the stall torque
# divide by the radius to get the force in the horizontal direction
# we have two motors so double it
max_input_value = ((motor_stall_torque * 0.8) / r) * 2.0

# the duty coeffiecent maps a desired torque range (-0.5, 0.5) to a motor
# speed that is between -1 and 1. 
duty_coeff = 1 / max_torque

# Take the input force u and multiply by radius r
# to get desired torque. Then we scale it to the motor
# speed by muliplying by the duty coefficient. Finally,
# we divide it in half to send half the torque to each 
# motor.
def speed_from_u(u):
    return ((u * r) / 2) * duty_coeff 

def u_from_speed(speed):
    return ((speed / duty_coeff) * 2) / r