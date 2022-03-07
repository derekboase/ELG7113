import data_collection_class as dc
import RPi.GPIO as gpio
import time

arduino_int0_trig_pin = 16

gpio.setwarnings(False)

gpio.setmode(gpio.BCM)
gpio.setup(arduino_int0_trig_pin, gpio.OUT)
gpio.output(arduino_int0_trig_pin, False)

csv_name = 'dataset'
i = 1

while True:
    input('Press any key to start fan characterisation test')
    data_coms = dc.data_collection('/dev/ttyACM0',
                                   './datasets/'+csv_name+f'_{i}.csv',
                                   ['duty_cycle', 'air_vel(m/s)'])
    time.sleep(3) # Gives time for data_coms to initialise
    
    print('Test running')
    gpio.output(arduino_int0_trig_pin, True)
    
    time.sleep(30)
    
    gpio.output(arduino_int0_trig_pin, False)
    data_coms.stop_listening()    
    i+=1