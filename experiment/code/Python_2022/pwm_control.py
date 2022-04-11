import RPi.GPIO as GPIO

from time import sleep

PWM_PIN = 19 # Can use 12, 13, 19, 24


def setup():
	global pwm
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(PWM_PIN, GPIO.OUT)
	GPIO.output(PWM_PIN, GPIO.LOW)
	pwm = GPIO.PWM(PWM_PIN, 1000)
	pwm.start(0)


def varry_duty():
	while True:
		for duty in range(0, 101, 1):
			pwm.ChangeDutyCycle(duty)
			sleep(0.01)
		sleep(1)
		for duty in range(100, -1, -1):
			pwm.ChangeDutyCycle(duty)
			sleep(0.01)
		sleep(1)


def user_input_pwm():
	'''
	Function prompts the user for a value in the range [0, 100] representing the duty cycle of the pwm.
	This is meant to allow the user to determine the baseline pwm signal that keeps the ball levitating
	at the opening.
	:return: None
	'''
	while True:
		duty = float(input('Enter the pwm value: '))
		try:
			pwm.ChangeDutyCycle(duty)
		except ValueError:
			print("[Err]\tPick value in range [0.0, 100.0]")  
		sleep(0.01)
		

def kill_pwm():
	pwm.stop()
	GPIO.output(PWM_PIN, GPIO.LOW)
	GPIO.cleanup()


if __name__ == "__main__":
	setup()
	try:
		user_input_pwm()
		# varry_duty()
	except KeyboardInterrupt:
		kill_pwm()
	
