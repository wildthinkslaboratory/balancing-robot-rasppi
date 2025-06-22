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
from utilities import countdown


class ImuSensor:

    def __init__(self):
        self.sensor = mpu6050(0x68)
        self.bias_x = 0.0
        self.oY = 0.0
        self.oZ = 0.0
        self.sY = 0.0
        self.sZ = 0.0
        print('Calibrate gyro [y/n]')
        run_gyro = input()
        if run_gyro == 'y':
            print('\nCalibrating gyro')
            self.calibrate_gyro(5000)
            print('\n\t done!')

        print('Calibrate accelerometer [y/n]')
        run_accel = input()
        if run_accel == 'y':
            print('Calibrating accelerometer')
            self.calibrate_accel(500)
            print('\n\t done!')

    def raw_angular_velocity_rad(self):
        gyro_data = self.sensor.get_gyro_data()
        return  (gyro_data["x"] - self.bias_x)* math.pi / 180 

    def raw_angle_rad(self):
        self.raw_accel_data()
        accelerometer_angle = math.atan2(self.ay_raw, math.sqrt(self.ax_raw**2 + self.az_raw**2))
        accelerometer_angle = accelerometer_angle + math.pi
        if (accelerometer_angle >= 2 * math.pi):
            accelerometer_angle -= 2 * math.pi
        return  accelerometer_angle

    def raw_angle_rad2(self):
        self.raw_accel_data()
        angle = math.atan2(-self.ay_raw, -self.az_raw)
        # accelerometer_angle = accelerometer_angle + math.pi
        if (angle >= 2 * math.pi):
            angle -= 2 * math.pi
        return  angle
    
    def raw_accel_data(self):
        accelerometer_data = self.sensor.get_accel_data()
        self.ax_raw = accelerometer_data['x']
        self.ay_raw = (accelerometer_data['y'] - self.oY) / self.sY
        self.az_raw = (accelerometer_data['z'] - self.oZ) / self.sZ

       
    def calibrate_gyro(self, samples):
        cum_bias_x = 0
        for _ in range(samples):
            data = self.sensor.get_gyro_data()
            cum_bias_x += data['x']
            sleep(0.005)
        self.bias_x = cum_bias_x / samples
        print("gyro x bias: ", self.bias_x)

    def calibrate_accel(self, samples):
        # 2 pose calibration
        g = 9.80665
        print('place bot in upright position')
        countdown(8)
        print('calibrating... \n')
        Ay0 = 0.0
        Az0 = 0.0
        for _ in range(samples):
            accelerometer_data = self.sensor.get_accel_data()
            Ay0 += accelerometer_data['y']
            Az0 += accelerometer_data['z']
            sleep(0.005)
        Ay0 = Ay0 / samples 
        Az0 = Az0 / samples 

        print('place bot at - 45 degrees')
        countdown(8)
        print('calibrating... \n')
        Ay45 = 0.0
        Az45 = 0.0
        for _ in range(samples):
            accelerometer_data = self.sensor.get_accel_data()
            Ay45 += accelerometer_data['y']
            Az45 += accelerometer_data['z']
            sleep(0.005)
        Ay45 = Ay45 / samples 
        Az45 = Az45 / samples 

        self.sY = (Ay0 - Ay45) / g * (1 - math.cos(math.pi/4))
        self.oY = Ay0 - self.sY * g
        self.oZ = Az0 
        self.sZ = -(Az45 - self.oZ) / (g * math.cos(math.pi/4))


def verify_accelerometer(imu):   
    print("testing angular accelleration readings")
    print('position your bot upright')
    countdown(8)
    timespan = 2

    for _ in range(timespan * 10):
        imu.raw_accel_data()
        print(imu.ay_raw, imu.az_raw)
        sleep(0.1)


def verify_gyro(imu):
    data = []

    def callback():
        global imu
        global angle_data
        raw_angular_velocity = imu.raw_angular_velocity_rad()
        data.append([raw_angular_velocity, -0.05, 0.05])
        
    # query the gyro in a timer
    timer = InterruptTimer(0.01, callback, 1)
    timer.start()

    while timer.running: # wait for the trial to end
        sleep(1)

    print("gyro noise variance", np.var([a[0] for a in data]))
    print("average error: ", np.average([abs(a[0]) for a in data]))

    for i in range(3):
        plt.plot([a[i] for a in data])
    plt.xlabel('Time')
    plt.ylabel('angular velocity')
    plt.show()

# test the sensor and compute the sensor variance
if __name__ == "__main__":
    from InterruptTimer import InterruptTimer
    import numpy as np  
    import matplotlib.pyplot as plt

    imu = ImuSensor()
    # verify_gyro(imu)
    verify_accelerometer(imu)