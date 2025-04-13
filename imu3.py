from mpu6050 import mpu6050 
from time import sleep
import math
import json


class ImuSensor:

    # create a gyro sensor and calibrate it
    def __init__(self):
        self.sensor = mpu6050(0x68)
        self.bias_x = 0.0
        self.calibrate_gyro(500)

    def raw_angular_velocity_rad(self):
        gyro_data = self.sensor.get_gyro_data()
        return  (gyro_data["x"] - self.bias_x)* math.pi / 180 

    
    def calibrate_gyro(self, samples):
        cum_bias_x = 0
        for _ in range(samples):
            data = self.sensor.get_gyro_data()
            cum_bias_x += data['x']
            time.sleep(0.002)
        self.bias_x = cum_bias_x / samples
        print("gyro x bias: ", self.bias_x)




if __name__ == "__main__":
    from InterruptTimer import InterruptTimer
    import numpy as np  
    import matplotlib.pyplot as plt
    
    print("testing gyro readings")
    imu = ImuSensor() 
    gyro_data = []

    def callback():
        global imu
        global gyro_data
        raw_angular_velocity = imu.raw_angular_velocity_rad()
        gyro_data.append(raw_angular_velocity)
        
    # query the gyro in a timer
    timer = InterruptTimer(0.01, callback, 5)
    timer.start()

    while timer.running: # wait for the trial to end
        sleep(1)

    print("gyro noise variance", np.var(gyro_data))

    plt.plot(gyro_data)
    plt.xlabel('Time')
    plt.ylabel('angular velocity')
    plt.show()

