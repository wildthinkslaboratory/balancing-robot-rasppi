from gpiozero import Motor, PWMOutputDevice, DigitalOutputDevice, Button
from time import sleep

ENCODER2_C1 = 27
ENCODER2_C2 = 22

PWMB = 18
AIN1 = 13
AIN2 = 19
BIN1 = 13
BIN2 = 19
STBY = 26

pulses = 0

# robot = Robot((AIN1, AIN2), (BIN1, BIN2))
motor = Motor(BIN1, BIN2)

pwmB = PWMOutputDevice(PWMB)

standby = DigitalOutputDevice(STBY)
standby.off()

encoder1 = Button(ENCODER2_C1)
encoder2 = Button(ENCODER2_C2)

def encoder1_rising():
	pulses+=1

def get_pulses():
	return pulses


def cleanup():
    motor.stop()
    pwmB.close()
    standby.close()
    print("Cleanup complete")
    
def main():
	try:
		standby.on()
		motor.forward()
		pwmB.value = 0.5
		sleep(2)
	finally:
		cleanup()

if __name__ == "__main__":
    main()

