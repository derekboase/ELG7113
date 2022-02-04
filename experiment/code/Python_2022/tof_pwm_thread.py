import RPi.GPIO as GPIO
import VL53L1X as VL

import threading
from time import sleep

PWM_PIN = 19 # Can use 12, 13, 19, 24

def setup():
	global pwm, tof
	tof = VL.VL53L1X(i2c_bus=1, i2c_address=0x29)
	
	# Setting up the motor control PWM on pin 19 (BCM)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(PWM_PIN, GPIO.OUT)
	GPIO.output(PWM_PIN, GPIO.LOW)
	pwm = GPIO.PWM(PWM_PIN, 1000)
	pwm.start(0)


def reading_tof():
	while True:
		distance_in_mm = tof.get_distance()
		print(distance_in_mm)


def changing_pwm(new_val):
	pwm.ChangeDutyCycle(new_val)
	sleep(0.01)


if __name__ == "__main__":
	setup()	
	tof.open()
	tof.start_ranging(1)
	tof_thread = threading.Thread(target=reading_tof, daemon=True)
	tof_thread.start()
	count = 0
	while count <20:
		changing_pwm(float(input('Enter new value of PWM: ')))
		count += 1
	tof.stop_ranging()
