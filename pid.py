
from time import sleep
import time
from motors import BRMotors
from imu import ImuSensor

imu_sensor = ImuSensor()

alpha = 0.98

last_error = 0
integral = 0

Kp = 20
Ki = 0
Kd = 0

motors = BRMotors()

try:
    while True:
        (angle, angle_v) = imu_sensor.angle_data()
        error = -1.4 + angle
        integral += error
        derivative = error - last_error

        output = Kp * error + Ki * integral + Kd * derivative
        if output > 100:
            output = 100
        elif output < -100:
            output = -100
        elif (output > 0) and (5 > output):
            output = 5
        elif (output > -5) and (0 > output):
            output = -5


        motors.run(output / 100)
        last_error = error
        time.sleep(0.01)


except KeyboardInterrupt:
    print("Program Interrupted")
