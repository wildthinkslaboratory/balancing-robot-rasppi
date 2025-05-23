# note: to Izzy
# I trimmed down this class to have just what we need for the lqe.
# I'm hoping that having less code in this class will help us debug
# the lqe system. We might want to eventually have one imu class 
# that will work for pid, lqr and lqe. Won't know what that looks 
# like until we have everything working.

# I need you to look through this and make sure I didn't miss anything.
# You know more about this sensor than me.

from mpu6050 import mpu6050 
from time import sleep
import math
import json


class ImuSensor:

    def __init__(self):
        self.sensor = mpu6050(0x68)
        self.bias_x = 0.0
        self.calibrate_gyro(500)

    def raw_angular_velocity_rad(self):
        gyro_data = self.sensor.get_gyro_data()
        return  (gyro_data["x"] - self.bias_x)* math.pi / 180 

    def raw_angle_rad(self):
        accelerometer_data = self.sensor.get_accel_data()
        self.ax_raw = accelerometer_data['x']
        self.ay_raw = accelerometer_data['y']
        self.az_raw = accelerometer_data['z']
        accelerometer_angle = math.atan2(self.ay_raw, math.sqrt(self.ax_raw**2 + self.az_raw**2))
        accelerometer_angle = accelerometer_angle + math.pi
        if (accelerometer_angle >= 2 * math.pi):
            accelerometer_angle -= 2 * math.pi
        return  accelerometer_angle
    
    def calibrate_gyro(self, samples):
        cum_bias_x = 0
        for _ in range(samples):
            data = self.sensor.get_gyro_data()
            cum_bias_x += data['x']
            sleep(0.002)
        self.bias_x = cum_bias_x / samples
        print("gyro x bias: ", self.bias_x)



# test the sensor and compute the sensor variance
if __name__ == "__main__":
    from InterruptTimer import InterruptTimer
    import numpy as np  
    import matplotlib.pyplot as plt
    
    print("testing gyro readings")
    imu = ImuSensor() 
    angle_data = []

    def callback():
        global imu
        global angle_data
        raw_angle = imu.raw_angle_rad()
        raw_angular_velocity = imu.raw_angular_velocity_rad()
        angle_data.append([raw_angle, raw_angular_velocity])
        
    # query the gyro in a timer
    timer = InterruptTimer(0.01, callback, 5)
    timer.start()

    while timer.running: # wait for the trial to end
        sleep(1)

    print("gyro noise variance", np.var([a[0] for a in angle_data]))

    plt.plot([a[0] for a in angle_data])
    plt.xlabel('Time')
    plt.ylabel('angular velocity')
    plt.show()

