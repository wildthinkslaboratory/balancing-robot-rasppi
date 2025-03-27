
from time import sleep
from mpu6050 import mpu6050
import math
import time
from motors import BRMotors

sensor = mpu6050(0x68)

alpha = 0.98
angle = 0.0

last_time = time.time()

last_error = 0
integral = 0

Kp = 18
Ki = 2
Kd = 1

def calibrate_gyro(samples=100):
    gyro_bias_x = 0
    for _ in range(samples):
        gyro_data = sensor.get_gyro_data()
        gyro_bias_x += gyro_data['x']
        time.sleep(0.01)
    return gyro_bias_x / samples

def get_accel_angle():
    accel_data = sensor.get_accel_data()
    ax = accel_data['x']
    ay = accel_data['y']
    az = accel_data['z']
    
    accel_angle = math.atan2(ay, math.sqrt(ax**2 + az**2)) * 180 / math.pi  
    return accel_angle  
    
    
def calculate_angle():
    global angle, last_time
    
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time
    
    gyro_data = sensor.get_gyro_data()
    gyro_rate = gyro_data['x'] - gyro_bias_x
    
    accel_angle = get_accel_angle()
    
    angle = alpha * (angle + gyro_rate *dt) + (1 - alpha) * accel_angle
    return(angle)
    


gyro_bias_x = calibrate_gyro() 

motors = BRMotors()

try:
    while True:
        error = -2+ calculate_angle()
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
