from mpu6050 import mpu6050 
import time
import math
import json
from time import perf_counter

class ImuSensor:

    # create a gyro sensor and calibrate it
    def __init__(self,dT):
        self.sensor = mpu6050(0x68)

        self.bias_x = 0.0
        self.calibrate_gyro(500)

        self.ax_raw = 0.0
        self.ay_raw = 0.0
        self.az_raw = 0.0

        self.gx_raw = 0.0
        self.g_previous = 0.0
        self.g_high_pass = 0.0

        self.ax_low_pass = 0.0
        self.ay_low_pass = 0.0
        self.az_low_pass = 0.0

        self.angle_previous = 0.0

        self.angle = 0.0
        self.angular_velocity = 0

        self.dT = dT

        cutoff_frequency = 5
        tau = 1/(2*math.pi*cutoff_frequency)
        self.alpha = tau / (tau + self.dT)
        self.beta = 0.02
        self.gamma = 0.9

        self.angle_nogyro = 0.0

    def raw_angular_velocity_rad(self):
        gyro_data = self.sensor.get_gyro_data()
        return  (gyro_data["x"] - self.bias_x)* math.pi / 180 

    


    def get_raw_data(self):
        
        accelerometer_data = self.sensor.get_accel_data()
        gyro_data = self.sensor.get_gyro_data()
        
        self.ax_raw = accelerometer_data['x']
        self.ay_raw = accelerometer_data['y']
        self.az_raw = accelerometer_data['z']

        self.gx_raw = gyro_data["x"] - self.bias_x

    def calibrate_gyro(self, samples):
        cum_bias_x = 0
        for _ in range(samples):
            data = self.sensor.get_gyro_data()
            cum_bias_x += data['x']
            time.sleep(0.002)
        self.bias_x = cum_bias_x / samples
        print("gyro x bias: ", self.bias_x)

    def low_pass(self):
        self.ax_low_pass = (1 - self.alpha) * self.ax_raw + self.alpha * self.ax_low_pass
        self.ay_low_pass = (1 - self.alpha) * self.ay_raw + self.alpha * self.ay_low_pass
        self.az_low_pass = (1 - self.alpha) * self.az_raw + self.alpha * self.az_low_pass
        return self.ax_low_pass, self.ay_low_pass, self.az_low_pass

    
    def high_pass(self):
        self.g_high_pass = (1 - self.alpha) * self.g_high_pass + (self.alpha) * (self.gx_raw)
        self.g_previous = self.g_high_pass
        return self.g_high_pass
    


    def complementary_filter(self):
        self.angle_previous = self.angle
        self.low_pass()
        self.high_pass()
        gyro_angle = self.angle + self.g_high_pass * self.dT
        accelerometer_angle = math.atan2(self.ay_low_pass, math.sqrt(self.ax_low_pass**2 + self.az_low_pass**2))
        accelerometer_angle = accelerometer_angle * 180 / math.pi  
        self.angle = (1 - self.beta) * gyro_angle + (self.beta)*(accelerometer_angle)
        self.angular_velocity = (self.angle - self.angle_previous) / self.dT
        # self.angular_velocity = (1 - self.gamma) * self.angular_velocity + self.gamma * self.g_high_pass
        # return self.angle * math.pi / 180, self.angular_velocity * math.pi / 180
        return self.angle * math.pi / 180, 0
        # return self.angle * math.pi / 180, self.angular_velocity * math.pi / 180, self.g_high_pass * math.pi / 180


# from InterruptTimer import InterruptTimer

# imu2 = ImuSensor(0.01) 
run_data = []

# def callback():
#     global imu2
#     global run_data
#     imu2.get_raw_data()
#     angle2, angular_velocity2, accelerometer_angle, gyro_angle = imu2.complementary_filter()
#     run_data.append([accelerometer_angle, angular_velocity2, angle2, imu2.gx_raw, imu2.g_high_pass])
    

# timer = InterruptTimer(0.01, callback, 5)
# timer.start()
def output_data():
    imu2 = ImuSensor(0.01) 
    for i in range(5000):
        imu2.get_raw_data()
        angle2, angular_velocity2, accelerometer_angle, gyro_angle = imu2.complementary_filter()
        run_data.append([accelerometer_angle, angular_velocity2, angle2, imu2.gx_raw, imu2.g_high_pass])
# output_data()
# Serializing json
json_object = json.dumps(run_data, indent=4)

# Writing to sample.json
with open("data.json", "w") as outfile:
    outfile.write(json_object)
