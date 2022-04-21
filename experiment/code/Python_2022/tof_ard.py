import numpy as np
import serial
import time

def setup():
    time.sleep(2)
    for i in range(20):
        serialPort.readline()
    print('taken out trash')
    serialPort.flush() 
    baseline = []
    for j in range(50):
        while not serialPort.inWaiting():
            pass
        height = np.array([float(serialPort.readline().decode('utf-8').rstrip())])
        baseline.append(height)
    return baseline
        
port_name = '/dev/ttyACM0' # Uno
#port_name = '/dev/ttyUSB0' # Nano

serialPort = serial.Serial(port_name, 115200, timeout=0.5)

time_arr = []
bottom = np.array(setup())
print(bottom.mean())
time_ns = time.time_ns()
for k in range(50):
    while not serialPort.inWaiting():
        pass
    height = np.array([float(serialPort.readline().decode('utf-8').rstrip())])
    time_delta = (time.time_ns() - time_ns)*1e-9
    print(height[0])
    time_arr.append(time_delta)
    time_ns = time.time_ns()
time_np_arr = np.array(time_arr)
print(f'mean={time_np_arr[10:].mean()},\tvar={time_np_arr[10:].var()}')
