from mpu6050 import mpu6050 
from time import sleep
import math
from utilities import countdown, output_data, import_data

g = 9.80665
kick_stand_angle = math.radians(39)

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
        print('Calibrate accelerometer [y/n]')
        run_accel = input()

        if run_gyro == 'y':
            print('\nCalibrating gyro')
            self.calibrate_gyro(1000)
            print('\n\t done!')
        else:
            self.bias_x = import_data('gyro_bias.json')

        if run_accel == 'y':
            print('Calibrating accelerometer')
            self.calibrate_accel(500)
            print('\n\t done!')
        else:
            accel_bias = import_data('accel_bias.json')
            self.oY = accel_bias['oY']
            self.oZ = accel_bias['oZ']
            self.sY = accel_bias['sY']
            self.sZ = accel_bias['sZ']

    def raw_angular_velocity_rad(self):
        gyro_data = self.sensor.get_gyro_data()
        return  (gyro_data["x"] - self.bias_x)* math.pi / 180 

    def raw_angle_rad(self):
        self.raw_accel_data()
        angle_rad = math.atan2(-self.ay_raw, self.az_raw) + math.pi
        if (angle_rad >= 2 * math.pi):
            angle_rad -= 2 * math.pi
        return  angle_rad

    
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
        output_data(self.bias_x, 'gyro_bias.json')
        

    def calibrate_accel(self, samples):
        # 2 pose calibration
        print('place bot in upright position')
        countdown(3)
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
        countdown(5)
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

        self.sZ = (Az45 - Az0) / (g*(math.cos(kick_stand_angle) - 1))
        self.oZ = Az0 - self.sZ*g
        self.oY = Ay0
        self.sY = (Ay45 - Ay0) / (g*math.sin(kick_stand_angle))

        accel_bias = {}
        accel_bias['oY'] = self.oY
        accel_bias['oZ'] = self.oZ
        accel_bias['sY'] = self.sY
        accel_bias['sZ'] = self.sZ
        output_data(accel_bias, 'accel_bias.json')

def verify_accelerometer(imu):   
    print("testing angular accelleration readings")
    print('position your bot upright')
    countdown(5)
    timespan = 5

    data = []
    for _ in range(timespan * 100):
        imu.raw_accel_data()
        angle = math.degrees(imu.raw_angle_rad() - math.pi) 
        data.append([imu.ay_raw, imu.az_raw, angle])
        sleep(0.01)

    print("\naccelerometer Y variance", np.var([(a[0]) for a in data]))
    print("average accelerometer Y: ", np.average([abs(a[0]) for a in data]))
    print('\n')

    print("accelerometer Z variance", np.var([(a[1]) for a in data]))
    print("average accelerometer Z: ", np.average([abs(a[1]) for a in data]))
    print('\n')

    print("accelerometer angle variance", np.var([(a[2]) for a in data]))
    print("average angle: ", np.average([abs(a[2]) for a in data]))
    print('\n')

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
        sleep(5)

    print("gyro noise variance", np.var([a[0] for a in data]))
    print("average error gyro error: ", np.average([abs(a[0]) for a in data]))
    print('\n')


# test the sensor and compute the sensor variance
if __name__ == "__main__":
    from InterruptTimer import InterruptTimer
    import numpy as np  

    imu = ImuSensor()
    print('put bot on block for variance computation')
    countdown(5)
    verify_gyro(imu)
    verify_accelerometer(imu)