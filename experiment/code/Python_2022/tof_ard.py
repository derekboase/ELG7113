import numpy as np
import serial
import time
        
port_name = '/dev/ttyACM0' # Uno
# port_name = '/dev/ttyUSB0' # Nano

serialPort = serial.Serial(port_name, 115200, timeout=0.5)

time_arr = []
time_ns = time.time_ns()
serialPort.reset_input_buffer()
for k in range(30):
    while not serialPort.inWaiting():
        pass
    height = np.array([float(serialPort.readline().decode('utf-8').rstrip())])
    time_delta = (time.time_ns() - time_ns)*1e-9
    print(height[0])
    time_arr.append(time_delta)
    time_ns = time.time_ns()
time_np_arr = np.array(time_arr)
print(f'mean={time_np_arr[10:].mean()},\tvar={time_np_arr[10:].var()}')
