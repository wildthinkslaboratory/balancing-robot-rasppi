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

pulses_per_rotation = 11
wheel_circ = 0.35

class BRMotors:
	def __init__(self):
		self.motors = Robot((AIN1, AIN2), (BIN1, BIN2))
		self.pulses = 0
		self.pwmA = PWMOutputDevice(PWMA)
		self.pwmB = PWMOutputDevice(PWMB)
		self.standby = DigitalOutputDevice(STBY)
		self.encoder1C1 = Button(ENCODER1_C1)
		self.encoder1C2 = Button(ENCODER1_C2)
		self.encoder2C1 = Button(ENCODER2_C1)
		self.encoder2C2 = Button(ENCODER2_C2)
		self.standby.off()

	def encoder1_rising(self):
		self.pulses +=1

	def get_position(self):
		return self.pulses / pulses_per_rotation * wheel_circ

	def cleanup(self):
		self.motors.stop()
		self.pwmA.close()
		self.pwmB.close()
		self.standby.close()

	def run(self, speed):
		self.standby.on()
		if speed > 0:
			self.motors.forward()
		if speed <= 0:
			self.motors.backward()
		self.pwmA.value = abs(speed)
		self.pwmB.value = abs(speed)
		
	def stop(self):
		self.motors.stop()

def main():
	try:
		my_motors = BRMotors()
		my_motors.run(0.5)
		sleep(2)
	finally:
		my_motors.cleanup()

if __name__ == "__main__":
    main()

