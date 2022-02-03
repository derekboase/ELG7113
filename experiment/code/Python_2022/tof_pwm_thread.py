import RPi.GPIO as GPIO
import VL53L1X as VL

from time import sleep

def setup():
	global pwm, tof
	tof = VL.VL53L1X(i2c_bus=1, i2c_address=0x29)
	tof.open()
	
	# Setting up the motor control PWM on pin 19 (BCM)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(PWM_PIN, GPIO.OUT)
	GPIO.output(PWM_PIN, GPIO.LOW)
	pwm = GPIO.PWM(PWM_PIN, 1000)
	pwm.start(0)

def reading_tof():
	

if __name__ == "__main__":
	count = 0
	while count < 20:
		tof.start_ranging(1)
		distance_in_mm = tof.get_distance()
		print(distance_in_mm)
		count += 1
	tof.stop_ranging()
