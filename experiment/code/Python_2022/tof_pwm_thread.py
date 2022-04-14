import RPi.GPIO as GPIO
import VL53L1X as VL

import threading
from time import sleep

PWM_PIN = 19 # Can use 12, 13, 19, 24

def setup():
    global pwm, tof
    tof = VL.VL53L1X(i2c_bus=1, i2c_address=0x29)
    sleep(1)
    
    # Setting up the motor control PWM on pin 19 (BCM)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PWM_PIN, GPIO.OUT)
    GPIO.output(PWM_PIN, GPIO.LOW)
    pwm = GPIO.PWM(PWM_PIN, 1000)
    pwm.start(0)


def reading_tof():
    while True:
        distance_in_mm = tof.get_distance()*1e-3
        print(0.38 - distance_in_mm)


def changing_pwm(new_val):
    pwm.ChangeDutyCycle(new_val)
    sleep(0.01)
    
    
def user_input_pwm():
    '''
    Function prompts the user for a value in the range [0, 100] representing the duty cycle of the pwm.
    This is meant to allow the user to determine the baseline pwm signal that keeps the ball levitating
    at the opening.
    :return: None
    '''
    duty = 0
    while duty >=0:
        duty = float(input('Enter the pwm value: '))
        try:
            pwm.ChangeDutyCycle(duty)
        except ValueError:
            if duty > 100.0:
                print("[Err]\tPick value in range [0.0, 100.0], <0 to kill") 
            else:
                return -1
        sleep(0.01)
        
        
def kill_pwm():
    pwm.stop(0)
    GPIO.output(PWM_PIN, GPIO.LOW)
    GPIO.cleanup()


if __name__ == "__main__":
    setup()
    tof.open()
    tof.start_ranging(1)
    try:
        tof_thread = threading.Thread(target=reading_tof, daemon=True)
        tof_thread.start()
        user_input_pwm() # Set baseline pwm. Control/ID goes after

    except KeyboardInterrupt:
        tof.stop_ranging() # This gets run once the code is stopped
        kill_pwm()
