from gpiozero import Robot, PWMOutputDevice, DigitalOutputDevice, Button
from time import sleep

ENCODER1_C1 = 23
ENCODER1_C2 = 24
ENCODER2_C1 = 27
ENCODER2_C2 = 22

PWMA = 12
PWMB = 18
AIN1 = 5
AIN2 = 6
BIN1 = 13
BIN2 = 19
STBY = 26

count_per_rotation = 310
wheel_circ = 0.21
distance_per_count = wheel_circ / count_per_rotation



class BRMotors:
	def __init__(self):
		self.motors = Robot((AIN1, AIN2), (BIN1, BIN2))
		self.pwmA = PWMOutputDevice(PWMA)
		self.pwmB = PWMOutputDevice(PWMB)
		self.standby = DigitalOutputDevice(STBY)
		self.encoder1C1 = Button(ENCODER1_C1)
		self.encoder1C2 = Button(ENCODER1_C2)
		self.encoder2C1 = Button(ENCODER2_C1)
		self.encoder2C2 = Button(ENCODER2_C2)
		self.standby.off()
		self.encoder1C1.when_pressed = self.inc_counts
		self.counts = 0
		self.count_inc = 1

	def inc_counts(self):
		self.counts += self.count_inc
		  
	def position(self):
		return self.counts * distance_per_count

	def cleanup(self):
		self.motors.stop()
		self.pwmA.close()
		self.pwmB.close()
		self.standby.close()

	def run(self, speed):
		self.standby.on()
		if speed > 0:
			self.count_inc = 1
			self.motors.forward()
		if speed <= 0:
			self.count_inc = -1
			self.motors.backward()
		self.pwmA.value = abs(speed)
		self.pwmB.value = abs(speed)
		
	def stop(self):
		self.motors.stop()

def main():
	try:
		my_motors = BRMotors()
		my_motors.run(0.25)
		sleep(1)
		my_motors.run(-0.25)
		sleep(1)
		print(my_motors.position())
	finally:
		my_motors.cleanup()

if __name__ == "__main__":
    main()

