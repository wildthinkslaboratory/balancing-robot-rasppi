
from time import sleep
import time
from motors import BRMotors
from imu import ImuSensor

imu_sensor = ImuSensor()

last_error = 0
integral = 0

Kp = 18
Ki = 2
Kd = 1

motors = BRMotors()

try:
    while True:
        (angle, angle_v) = imu_sensor.angle_data()
        error = - 2 + angle
        integral += error
        derivative = error - last_error

        output = Kp * error + Ki * integral + Kd * derivative
        if output > 100:
            output = 100
        elif output < -100:
            output = -100

        motors.run(output / 100)
        last_error = error
        time.sleep(0.01)


except KeyboardInterrupt:
    print("Program Interrupted")
