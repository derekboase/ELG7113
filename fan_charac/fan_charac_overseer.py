import data_collection_class as dc
import RPi.GPIO as gpio
import time

arduino_int0_trig_pin = 16

gpio.setwarnings(False)

gpio.setmode(gpio.BCM)
gpio.setup(arduino_int0_trig_pin, gpio.OUT)
gpio.output(arduino_int0_trig_pin, False)

while True:
    input('Press any key to start fan characterisation test')
    
    print('Test running')
    gpio.output(arduino_int0_trig_pin, True)
    
    time.sleep(30)
    
    gpio.output(arduino_int0_trig_pin, False)
    