from mpu6050 import mpu6050
import time
import math

class ImuSensor:

    # create a gyro sensor and calibrate it
    def __init__(self, samples=100):
        self.sensor = mpu6050(0x68)
        cum_bias_x = 0
        for _ in range(samples):
            data = self.sensor.get_gyro_data()
            cum_bias_x += data['x']
            time.sleep(0.01)
        self.bias_x = cum_bias_x / samples
        self.last_time = time.time()
        self.angle = 0.0
        self.alpha = 0.98


    def calculate_angle(self):
        # update the time
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # compute the angle
        data = self.sensor.get_gyro_data()
        gyro_rate = data['x'] - self.bias_x
        accel_angle = self.get_accel_angle()   
        self.angle = self.alpha * (self.angle + gyro_rate *dt) + (1 - self.alpha) * accel_angle
        return self.angle


    def get_accel_angle(self):
        accel_data = self.sensor.get_accel_data()
        ax = accel_data['x']
        ay = accel_data['y']
        az = accel_data['z']
        return math.atan2(ay, math.sqrt(ax**2 + az**2)) * 180 / math.pi  
        
        
