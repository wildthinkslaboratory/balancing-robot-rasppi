from mpu6050 import mpu6050 
import time
import math
import json

class ImuSensor:

    # create a gyro sensor and calibrate it
    def __init__(self, samples=500):
        self.sensor = mpu6050(0x68)
        cum_bias_x = 0
        for _ in range(samples):
            data = self.sensor.get_gyro_data()
            cum_bias_x += data['x']
            time.sleep(0.002)
        self.bias_x = cum_bias_x / samples
        self.update_time = time.time()
        self.last_angle = 0.0
        self.angle = 0.0
        self.alpha = 0.98
        print("gyro x bias: ", self.bias_x)


    def angle_data(self):
        # update the time
        current_time = time.time()
        dt = current_time - self.update_time
        self.update_time = current_time

        # compute the angle
        data = self.sensor.get_gyro_data()
        gyro_rate = data['x'] - self.bias_x
        accel_angle = self.get_accel_angle()   
        self.angle = self.alpha * (self.angle + gyro_rate *dt) + (1 - self.alpha) * accel_angle

        self.last_angle = self.angle
        # return (self.angle, gyro_rate)
        return (0, gyro_rate)

    
    def get_accel_angle(self):
        accel_data = self.sensor.get_accel_data()
        ax = accel_data['x']
        ay = accel_data['y']
        az = accel_data['z']
        return math.atan2(ay, math.sqrt(ax**2 + az**2)) * 180 / math.pi  
        

def main():
    imu = ImuSensor()  
    counter = 0  
    while True:
        # print(angle)
        angle, vel = imu.angle_data()
        # if counter % 100 == 0:
        #     print(angle, vel)
        time.sleep(0.01)
        counter += 1

def output_data():
    imu = ImuSensor()  
    counter = 0  
    run_data = []
    for i in range(500):
        angle, vel = imu.angle_data()
        run_data.append([angle,vel,imu.avg_angle, imu.avg_gyro_rate])
        time.sleep(0.01)
    
    # Serializing json
    json_object = json.dumps(run_data, indent=4)
 
    # Writing to sample.json
    with open("gyro.json", "w") as outfile:
        outfile.write(json_object)


if __name__ == "__main__":
    #main()
    output_data()

