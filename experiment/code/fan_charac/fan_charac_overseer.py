import data_collection_class as dc
import RPi.GPIO as gpio
import time

arduino_int0_trig_pin = 16

# Arduino Variables
# Ensure these variables match those in Arduino script
start_duty = 100 # duty cycle encoded in 1 byte (0 - 255)
num_samples = 100
delay_samples = 200 # in millis

test_time = (255-start_duty)*num_samples*delay_samples/60000 # test time in min

print_interval = 0.5 # in min
print_loops = int(test_time//print_interval)
print(f'Test will take {test_time} min')

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
    
    for i in range(print_loops + 1):
        print(f'{test_time - i*print_interval} minutes left')
        time.sleep(print_interval*60) # Gives time for data_coms to initialise

    
    gpio.output(arduino_int0_trig_pin, False)
    data_coms.stop_listening()    
    i+=1