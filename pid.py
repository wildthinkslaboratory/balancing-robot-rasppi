
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
        error = -2+ imu_sensor.calculate_angle()
        integral += error
        derivative = error - last_error

        output = Kp * error + Ki * integral + Kd * derivative
        if output > 100:
            output = 100
        elif output < -100:
            output = -100
        print(error)

        motors.run(output / 100)
        last_error = error
        time.sleep(0.01)


except KeyboardInterrupt:
    print("Program Interrupted")
